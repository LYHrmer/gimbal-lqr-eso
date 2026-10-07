# 在官方 C 板框架中接入 yaw 与 pitch

[`GimbalController`](../include/gimbal_controller.h) 是单轴控制实例；yaw 和 pitch 各自分配状态、参数和输入。`pitch_enabled=false` 走原 yaw 路径，`pitch_enabled=true` 增加名义重力与机械角检查。底层 `YawFeedback`、`YawReference` 和 `YawStatus` 名称为兼容既有接口保留，其中的 SI 物理量也用于 pitch。

[`examples/stm32/gimbal_periodic.h`](../examples/stm32/gimbal_periodic.h) 和[实现](../examples/stm32/gimbal_periodic.c)提供 HAL 无关薄层，与原 yaw 示例具有相同的真实 dt、快照、故障锁定和显式复位契约。它不创建任务、不初始化 CAN，也不使能电机。将两个实例放入官方 `19.gimbal_task` 的任务层，设备层保有电机模式、协议和总线所有权；源代码依据见 [pitch 资料核对](pitch_sources.md)与[官方框架接入说明](opensource_integration_notes.md)。

## 任务中的两实例接入

下面是宿主结构示意。`verified_*_config`、`*_hooks` 与 `host_*` 都由整车工程提供，示例不包含可直接上电使用的机械参数或板级实现。

```c
static Stm32GimbalPeriodic yaw_axis;
static Stm32GimbalPeriodic pitch_axis;

/* Run once in the existing gimbal task, with drives still inhibited. */
bool host_gimbal_init(void)
{
    const bool yaw_ok = stm32_gimbal_periodic_init(
        &yaw_axis, &verified_yaw_config, yaw_hooks);
    const bool pitch_ok = stm32_gimbal_periodic_init(
        &pitch_axis, &verified_pitch_config, pitch_hooks);
    return yaw_ok && pitch_ok;
}

/* Called in the existing task after coherent feedback/reference publication. */
void host_gimbal_control_tick(void)
{
    host_begin_motor_group_cycle();
    const Stm32GimbalResult yaw_result =
        stm32_gimbal_periodic_step(&yaw_axis);
    const Stm32GimbalResult pitch_result =
        stm32_gimbal_periodic_step(&pitch_axis);
    host_finish_motor_group_cycle(yaw_result, pitch_result);
}
```

`verified_yaw_config.pitch_enabled` 设为 `false`，pitch 实例设为 `true`；两轴不能共享一个控制器的 ESO 或积分状态。模式切换时，由宿主生成同坐标参考并按其状态机管理复位。故障恢复必须经过明确确认后调用 `stm32_gimbal_periodic_acknowledge()`，此调用只清历史与锁定状态，不会使能驱动。恢复后的首个有效周期仍返回 `STM32_GIMBAL_WARMUP` 并提交零力矩。

两个 `submit_torque_nm` 回调只向设备层提交本轴的 N·m 和时间戳。GM6020 同组电机由唯一组帧所有者汇总，完成两轴更新后再发送整帧；按具体槽位保留组内其他合法命令。不得在 yaw 回调中将 pitch 槽位置零后立即发送，也不得让原 pitch PID 与新控制器同时写同一槽位。若任何回调要求停机，宿主先撤销该轴未发送命令，再按整车既定策略决定单轴还是整组停机；不能把上一周期缓存当作本周期的新命令。DM4310 MIT 则由各自设备对象完成力矩编码，仍应统一监测 CAN 发送和驱动状态。

## 快照中的两类角度和时间

pitch 的 `Stm32GimbalSnapshot` 同时包括：

- `feedback` 与 `reference`：选定控制坐标的连续 rad、rad/s 和 rad/s²。参考与反馈的零点、正方向一致；反馈速度必须与所用角度坐标对应。
- `pose.joint_position_rad`、`joint_velocity_rad_s`：编码器定义的机械相对角及相对角速度，配合经过实测的机械边界。
- `pose.gravity_angle_rad`：重力模型坐标中的姿态角。本文约定水平为零、抬头为正；重力系数也必须按这个约定提供。

固定平面台架经过零点校准后，机械角与重力角可能只差一个常数。底盘有俯仰时不应直接用编码器角代替重力角；底盘滚转或转轴倾斜时，也不能任意拼接 Euler 角与单轴陀螺仪。本适配器用平面关系 `joint_reference = joint_position + reference.position - feedback.position` 检查机械可达范围，动态底盘和一般两轴耦合尚未验证。

`feedback_source_us` 和 `reference_source_us` 保留各自数据产生时间。`pose_source_us` 必须取构建 pose 所用全部数据源时间戳的最小值，例如编码器、姿态及所需底盘变换中最旧的一项；不能取最新时间或控制循环读取时间。`read_snapshot` 用短临界区复制或双缓冲取得一致快照，之后薄层读取同一时钟域的 `monotonic_us`，计算真实 dt 和三路 age。薄层会覆盖调用方填写的 age；pitch 姿态的新鲜度由核心按反馈超时配置检查。未来时间戳或不前进的控制时钟导致停机锁定。

yaw 实例不依赖 `pose`：薄层不计算姿态 age、不检查姿态时间戳，并向核心传入空 pose 指针。两个模式都不把瞬时电流采样宣称为上一控制间隔的平均执行力矩，因此薄层清除 `applied_torque_valid`，由核心使用上一周期受限命令预测。

## 重力、机械边界和停机

重力保持力矩 `A*cos(theta_g)+B*sin(theta_g)` 在总输出的硬限幅、变化率限制和积分回算之前参与控制；设备层收到的是已经包含重力补偿的总力矩，不应再叠加一次。GM6020 电流模式和 DM4310 MIT 纯力矩模式各自使用明确的驱动标度与应用限值。pitch 的静态重力会占用持续力矩预算，不能把电机峰值当作可长期保持的力矩。

若宿主已有标定过的多项式或查表曲线，可直接调用 `gimbal_controller_step_with_gravity()`，传入当前所需保持力矩，并将两项内置重力系数设为零；机械角、参考检查和上一周期负载历史仍由适配层管理。符号、输入年龄及调用顺序见[自定义曲线接入](firmware_porting.md#3-迁移宿主已有的重力曲线)。该入口不用于辨识激励，也不会改变默认周期薄层的模型。

本适配器检查机械反馈边界及向内留量后的机械参考区间，违规时拒绝并锁定故障；它不生成刹车轨迹，也不模拟机械硬挡。上层应先生成满足速度、加速度和机械范围约束的参考，不能依靠越界后清零来保证机构不会碰撞。

宿主通过 `gimbal_joint_reference_bounds(&config, &joint_low, &joint_high)` 获取与控制器一致的参考边界，并检查返回值。该函数只验证 Pitch 几何；完整配置仍用 `gimbal_config_valid()` 检查。端点向内舍入为可表示的 float，不放宽机械边界或 `joint_margin_rad`；无法形成两个不同端点时配置无效。不要自行用 float 的 `joint_min + margin` 重算端点，以免与实际留量产生舍入差异。

参考规划应在**机械关节坐标**内使用这两个端点，并在端点处消除外向关节参考速度。之后再转换到控制坐标，保证位置、速度和加速度一致；不能只夹位置而保留继续向外的速度。控制器在内缩端点允许静止或向内参考，在下端点拒绝负向、上端点拒绝正向关节参考速度。原机械反馈硬限位保持不变。

**零力矩不等于 pitch 保持。** 暖机、失能或故障时，重力加载机构仍可能下落。宿主 `request_disable` 应撤销待发命令并执行真实停机路径；配重、支撑或制动由实际机构及设备层负责。本仓库尚无实机保持、停机距离、热或动态底盘验证，仿真配置中的惯量、重力系数和边界都不能直接视为用户硬件参数。
