# 从实际整机工程接入公开 C 库

[项目首页](../README.md) · [STM32 周期与回调](stm32_integration.md) · [本次核查记录](porting_review_20261007.md)

这份说明来自对用户私有整机归档的接口核查。整机源码继续由宿主维护，公开库只提供控制核心、协议编码和接入工具。迁移时先接一轴、一种模式，保留明确的旧/新控制选择；每周期只有被选中的控制器更新状态并提交命令。

## 1. 加入现有 CMake 工程

保留宿主的 `project(... LANGUAGES C CXX ASM)`、启动文件、链接脚本、FreeRTOS 和 HAL。创建固件目标后添加：

```cmake
include("${CMAKE_CURRENT_SOURCE_DIR}/third_party/gimbal-lqr-eso/cmake/GimbalController.cmake")
gimbal_add_to_target(your_firmware_target GM6020)
```

DM4310 将第二个参数改为 `DM4310`。目标名换成已有 `add_executable()` 的目标；每目标调用一次。此入口加入基础核心、Pitch 适配、坐标助手、所选协议和 `gimbal_periodic.c`，设置 include 路径、C11 要求和 `libm` 链接。它不会配置本库的主机共享库、测试工程、实验 RLS 或仿真参数。宿主里的 `.c` 仍由 C 编译器处理，`.cpp` 可保持 C++17，公开头文件已有 `extern "C"`。

CPU/FPU/ABI、优化和链接选项沿用宿主，须覆盖新增 C 文件。不要只设置 `CMAKE_CXX_FLAGS`，遗漏 C 源码；不要给本库另开一套不一致的浮点 ABI。若旧工程以 GLOB 收集整个 `third_party/`，先排除本库，避免手工收集与此入口重复。原类实现可以保留作回退，但其输出不能再接到新闭环后面。

使用 `-O2` 或经验证的 `-O3`，保留有限数检查；`-Ofast`、`-ffast-math`、`-ffinite-math-only` 不允许。入口不会悄悄覆盖整车编译选项，危险选项会触发源码编译错误；重关联等未必有宏可检测的选项仍须核查，详见 [编译契约](stm32_integration.md#c11fpu-和实际耗时)。

无需私有 ZIP 的 C++17 链接检查：

```bash
python3 -m unittest discover -s tests -p 'test_cmake_port.py'
```

测试为 GM6020、DM4310 分别创建独立宿主工程，编译并运行公开 C API，检查只加入所选协议，以及重复接入、无效电机名和危险优化被拒绝。测试中的数值是合成 fixture，不是上机默认值。

## 2. 先固定信号、单位与所有权

在宿主私有配置中填写下表，不能用另一台车的轴号、正负号或参数直接代替。

| 宿主信号 | 公开入口 | 必须确认 |
| --- | --- | --- |
| 位置反馈 | `YawFeedback.position_rad` | 连续输出轴 rad；度乘 π/180；位置与参考处于同一坐标 |
| 角速度 | `velocity_rad_s` | 选定角度的导数，机体系 gyro 分量须按安装及姿态关系投影 |
| 手动/视觉参考 | `YawReference` | rad、rad/s、rad/s²；不要先设置速率又在同周期无条件清零 |
| 编码器关节反馈 | `GimbalPose.joint_*` | 独立校准的机械角和速度，不能用 AHRS 世界角代替限位角 |
| 重力模型输入 | `gravity_angle_rad` 或宿主计算的保持力矩 | 模型坐标、零点、定义符号及有效范围与标定一致 |
| 接收/生成时间 | `*_source_us` / `age_s` | 同一单调时钟域；年龄在读快照时计算，源戳在数据产生时更新 |
| 实际授权与驱动状态 | `drive_ready`、宿主停止状态机 | 离线、停机、CAN 失败时作废普通命令；不能仅靠非零反馈使能 |
| 最终总力矩 | 协议编码器 | 方向反转只在明确的坐标映射处做一次；保持 J 为正 |

反馈、参考、有效位和源戳必须成组发布/读取。分别看见“新时间”和“旧角度”仍是无效快照。DWT 只用于实际间隔或耗时测量；越窗周期不能回填 1 ms 后继续输出。`stm32_gimbal_periodic_step()` 已有周期、源时间和停止锁存处理；直接调用核心时，由宿主提供这些年龄、授权和停机动作。

GM6020 输出方向反转时，先将 SI 力矩按方向映射，再调用 `gm6020_torque_to_word()`。只有 `GM6020_OK && word.valid` 才能更新所属组的槽位；由已有唯一 CAN 所有者发布完整 4 槽，保留其他电机的新鲜命令。不要在这里增加第二个发送任务。

控制器力矩上限应不大于协议层的有效应用限值 `min(torque_limit_nm, current_limit_a * Kt)`；运行中降额先调用 `gimbal_controller_set_torque_limit()`。编码后的 `torque_wire_nm` 是量化命令的名义值，不是电机实测力矩。`applied_torque_valid` 只用于确有上一完整周期平均执行力矩的情况，不能拿本帧 CAN 命令或瞬时电流冒充它。

## 3. 迁移宿主已有的重力曲线

内置模型适合 `A*cos(theta)+B*sin(theta)`。已有经标定的多项式/查表曲线，可以使用新增的 `gimbal_controller_step_with_gravity()`，无需重新拟合为正弦，也无需绕过 Pitch 的机械限位。

这个入口的 `holding_torque_nm` 是**平衡重力所需的电机保持力矩**：

```text
J * acceleration = motor_torque - drag - holding_torque + residual
```

若原模型写成 `J*a = u - B*w + tau_g + d`，这里应传 `-tau_g`。实际曲线和系数仍放在宿主私有侧。使用此入口时必须 `pitch_enabled=true`，并把 `gravity_cos_nm`、`gravity_sin_nm` 同时设为零；重复配置内置模型会返回并锁存 `YAW_BAD_CONFIG`。

以下是放在**已有控制任务**里的调用顺序示意，其中快照、曲线和停止函数由宿主实现：

```cpp
// axis/config 在初始化阶段静态分配；本段不重新 init。
// feedback/reference/pose 已完成一致快照、坐标、年龄及有效性校验。
const float hold_nm = calibrated_holding_torque(pose.gravity_angle_rad);
const YawStatus status = gimbal_controller_step_with_gravity(
    &axis, &feedback, &reference, &pose, actual_dt_s, hold_nm, &output);
if (status != YAW_OK && status != YAW_WARMUP) {
    invalidate_pending_axis_command();
    request_host_stop(status);
    return;
}
publish_to_existing_group_owner(output.control.torque_nm);
```

`pose.age_s` 要覆盖曲线涉及的最老输入，原有 pose 有效性要求仍然适用。适配层保存上一周期的模型负载供 ESO 使用；本周期模型值用于本周期前馈，并参与总幅值限制、变化率限制和反算抗饱和。不要再从观测器输入里手工减一次重力，也不要在输出端额外加一次重力。暖机仍输出零，不能据此假定不平衡 Pitch 会被保持。

原 `gimbal_controller_step()` 和 `stm32_gimbal_periodic_step()` 继续使用内置模型；周期回调结构没有新增字段。自定义曲线的宿主直接使用上面的核心入口，并落实 [周期薄层同等的快照、时间和停止责任](stm32_integration.md#薄层回调与状态)。两个入口不要交替作为不同控制模式运行；切换模型或反馈坐标时先完成宿主同步和明确复位。

## 4. 模式切换与辨识不要隐式接通

| 事件 | 宿主应做什么 |
| --- | --- |
| 手动/视觉切换 | 停止旧输出所有者；准备新坐标下的参考和来源时间；同步目标微分/滤波状态；按策略显式 reset，再等待真实周期 |
| 停机、异常周期、失联或 CAN 失败 | 作废排队命令，进入已有停止流程，保留首个故障原因；下一帧正常不能自动清故障 |
| 明确恢复 | 确认停止原因消除、重新同步参考和时钟；核心 reset 或周期薄层 acknowledge 后，首个有效周期暖机输出零 |
| 台架辨识启停 | 由宿主单一开关控制；明确退出/中止条件，保存该轮实际配置；参数输出只作候选，不能在控制周期内热改增益 |

**辨识 PRBS 不是重力负载。** 不要把激励传给 `holding_torque_nm`，否则它会被当作被动负载从 ESO 模型扣除。公开库没有闭环 PRBS 注入入口，本次也没有公开私有辨识实现。

若独立辨识固件在控制器输出之后叠加激励，仅再次夹幅不能保证原力矩变化率上限仍成立，原控制器的输出历史/抗饱和也没有包含这段激励。应使用专门的辨识输出所有者，对**最终总命令**统一幅值、变化率、停止门控，再按实际发送/执行时序对齐回归量；不能靠把即时命令标记成 `applied_torque_valid=true` 来掩盖这个差别。

## 5. 每次移植留一份可复核记录

复制 [空白移植记录](../examples/porting_record_template.md)，记录公开库提交、宿主私有版本/哈希、编译器、编译选项、信号/方向映射、实际启用路径及复跑命令。涉及私有目录、曲线或整机改动的记录留在宿主，公开记录只放通用结论和获准公开的证据。

上机日志另保存目标角、反馈角、编码器关节角、gyro、各自源时间与 dt、模型保持力矩、控制/观测分项、最终组帧命令和启停事件，见 [实机记录规范](test_data_recording.md)。对秒级回移，要能区分目标自身变化、AHRS 与机械角偏离、补偿恢复和饱和退场；只有源码或视频时，把这些列为待检验的解释，不写成已排除其他原因。
