# DM4310 · MIT 纯力矩版本

[项目首页](../../README.md) · [参数辨识](#参数辨识入口) · [文档导航](../../docs/README.md) · [GM6020 版本](../gm6020/README.md)

本版通过 MIT 纯力矩通道驱动 DM4310，外部位置与速度控制由公共 C 内核完成。Yaw、Pitch 各用独立实例；手瞄和自瞄在上层生成参考后接入同一控制器。

| 项目 | 本版约定 |
| --- | --- |
| 命令通道 | MIT 模式，电机内部 `Kp=Kd=0`，输出写入力矩前馈字段 |
| 协议前提 | 按实物确认 CAN ID、工作模式及 `PMAX/VMAX/TMAX` |
| 参考型号 | 手册中的 24 V DM-J4310-2EC V1.1；其他电压或固件版本需另行核对 |
| 静态库 | `build/libgimbal_dm4310.a` |

本仓库当前提供 **MIT 模式**，尚未实现一拖四模式。库本身不打开设备，也不执行电机使能或失能。

## 需要移植哪些文件

可使用目标 STM32 工具链编译的静态库，也可按所需功能将以下 C 文件与头文件加入原工程；电脑端构建的库不能直接链接到固件。

| 功能 | 源码 | 接口 |
| --- | --- | --- |
| 实时控制内核，必选 | [yaw_controller.c](../../src/yaw_controller.c) | [yaw_controller.h](../../include/yaw_controller.h) |
| MIT 协议适配，必选 | [dm_mit.c](../../src/dm_mit.c) | [dm_mit.h](../../include/dm_mit.h) |
| Pitch 重力与关节边界，可选 | [gimbal_controller.c](../../src/gimbal_controller.c) | [gimbal_controller.h](../../include/gimbal_controller.h) |
| 最近等价圈方向参考，可选 | [gimbal_coordinates.c](../../src/gimbal_coordinates.c) | [gimbal_coordinates.h](../../include/gimbal_coordinates.h) |

已有 STM32 任务与 CAN 驱动可继续使用；[Yaw/Pitch 周期薄层](../../examples/stm32/gimbal_periodic.c) 提供回调对接示例。构建与周期接口见 [STM32 接入说明](../../docs/stm32_integration.md)。

## 接入顺序

1. **确认驱动配置。** 完成 CAN 收发、ID、模式与反馈范围核对，并由上层管理使能、失能和通信看门狗。
2. **准备每轴输入。** 传入连续展开的输出轴角度、速度、源数据年龄，以及一致的位置/速度/加速度参考；Pitch 另需机械关节姿态与重力倾角。
3. **计算并编码力矩。** 仅在上层允许驱动且周期状态为 `YAW_OK` 或 `YAW_WARMUP` 时调用 `dm_mit_encode_torque()`。
4. **提交有效帧。** 编码成功且 `frame.valid=true` 后才提交 CAN；失败帧的 `DLC=0`，不得重发旧 payload。
5. **处理停止与恢复。** 故障进入上层停止/失能流程，原因消除后显式复位；下一帧反馈正常不会自动恢复控制。

协议映射 `TMAX`、电机额定/峰值能力和本次实验力矩上限是三个不同参数。不能修改本地 `TMAX` 来代替正确的运行限额。量化零偏和完整接口见 [DM4310 接入说明](../../docs/dm4310_integration.md)。

## 参数辨识入口

先读 [DM4310 辨识步骤](../../docs/dm4310_identification.md)，再准备 [MIT 空日志模板](../../examples/dm4310_identification_template/README.md)。建议按以下顺序开展：

1. **核准标度与坐标。** 读回实物 `PMAX/VMAX/TMAX`，核对方向、零位及额外传动；MIT `Kp=Kd=0` 时仍保留已稳定的受限外环。
2. **记录实际可得信号。** 保存命令载荷与量化后名义力矩、反馈位置/速度/转矩估计、状态和两路温度，以及实际控制参考与各自时间。转矩回读不是轴端扭矩传感器，CAN 受理也不表示力矩已经施加。
3. **逐轴拟合并独立验证。** 先观察双向摩擦，Pitch 另校准重力倾角；再辨识动态 `J/B`，用另一整次运行验证。标度未确认时只报告有效/归一化模型。

专项页区分已有编解码、LQR 求解与 RLS 实验，以及尚未实现的实测采集和 CSV 拟合。RLS 仅旁路输出候选参数，默认不构建、不热更新控制器。

## 仿真配置与结果

| 轴 | JSON 参数来源 | 生成的 C 参数示例 |
| --- | --- | --- |
| Yaw | [dm4310_24v.json](../../profiles/dm4310_24v.json) | [simulation_config.h](simulation_config.h) |
| Pitch | [dm4310_24v_pitch.json](../../profiles/dm4310_24v_pitch.json) | [pitch_simulation_config.h](pitch_simulation_config.h) |

**两份配置都标为 `simulation_only`。** `dm4310_simulation_config()` 及 Pitch 参数仅用于所声明的仿真负载；上机需替换实际负载辨识值与允许限值。电机型号相同不能确定整车的惯量、阻尼和延迟。

| 验证组 | 对照方式 | 平均配对位置 RMSE 变化 |
| --- | --- | --- |
| Yaw：2 Hz、±5° | 原/新版相同参数，积分关闭 | 降低 **6.90%**，5/5 改善 |
| Pitch：中心 +20°、1 Hz、±5° | 新版重力开启，对同 Ki 原版 | 降低 **72.53%**，5/5 改善 |

上述两组主工况的开发与留出验证均通过预定门槛；结论限于所列单轴合成模型。参数失配下也有退化，仿真尚未验证真实温升、电流环或齿隙。完整证据见 [Yaw 验收](../../docs/validation_report.md) 与 [Pitch 验证](../../docs/pitch_validation.md)。

## Pitch 与多轴使用

Pitch 需独立标定重力角、机械编码器零点和行程；不能照搬 Yaw 参数。重力必须在总力矩限幅与抗饱和之前加入，通用 known-load 接口允许宿主提供自己的模型负载，详见 [Pitch 接入](../../docs/pitch_integration.md)。Warmup 和故障零力矩不提供重力保持，物理保持措施由整车实现。

大 Yaw、小 Yaw、Pitch 可各分配一个实例，上层负责参考分配与耦合负载。三个独立实例的软件支持不代表已实现大小 Yaw 协调或验证三轴闭环，见 [多轴接入边界](../../docs/multiaxis_integration.md)。
