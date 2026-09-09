# GM6020 适配版本

本版使用 GM6020 官方 v1.4 手册描述的**转矩电流控制**，支持独立 Yaw/Pitch 实例；手瞄和自瞄均由上层生成参考后接入。公共 LQR–ESO 内核输出 N·m，适配器按显式转矩常数换算为 A，再编码为电流命令。编译的静态库是 `build/libgimbal_gm6020.a`；源码按使用的接口移植：

- `include/yaw_controller.h`、`src/yaw_controller.c`
- `include/gm6020.h`、`src/gm6020.c`
- Pitch 便利适配：`include/gimbal_controller.h`、`src/gimbal_controller.c`
- 最近等价圈方向目标助手：`include/gimbal_coordinates.h`、`src/gimbal_coordinates.c`

先确认电机固件 `>=1.0.11.2`，并在 RoboMaster Assistant `>=2.7` 中开启电流环。适配器要求显式确认此模式；不能仅根据电机名假定支持，也不自动降级为电压控制。

电机 1–4 的电流组帧是 `0x1FE`，5–7 是 `0x2FE`，指令 ±16384 对应 ±3 A。官方转矩常数参考为 `0.741 N·m/A`；±3 A 是协议范围，不是连续工作电流。仿真配置单独设置 1.62 A / 1.2 N·m 上限，仍不构成实际散热条件下的连续运行保证。

本目录 `simulation_config.h` 由 [仿真 JSON](../../profiles/gm6020_current.json) 生成，`gm6020_simulation_config()` 只用于仿真示例。负载 `J/B` 没有根据型号自动确定，必须针对自己的整车重新辨识。

另有由 [Pitch JSON](../../profiles/gm6020_current_pitch.json) 生成的 `pitch_simulation_config.h`。Pitch 重力角与机械编码器零位分别标定，重力前馈必须进入最终限幅和抗饱和之前；不要把它追加到已限制的电流命令后面。完整接口见 [Pitch 接入](../../docs/pitch_integration.md)。

本次选择慢积分 `Ki=2Kp`、积分上限 0.2 N·m、回算速率 2 s⁻¹，速度误差滤波关闭。主工况收益包含这项配置变化；同积分原版的额外对照和延时退化边界见 [最终验收报告](../../docs/validation_report.md)。这组参数仍需结合实际 CAN 延时与负载重新验证。

上段归因针对 Yaw。独立 Pitch 比较保持原/新相同 Ki：留出组平均配对 RMSE 降低 2.94%，4/5 改善；但开发组只有 1/3 改善且未通过预定门槛，因此 Pitch 总验收仍为 false，不能宣传为 GM Pitch 在全部分组稳定改善。CI 完整重跑实验，性能断言只覆盖预定留出组，并保留开发失败；[完整报告](../../docs/pitch_validation.md)也公开长延迟与高重力退化。

接入时，先对每个电机单独做 N·m→电流编码，再由整车的统一 CAN 调度器收集**同组全部电机**的命令，最后发送完整组帧。不能在 yaw 单轴周期中把其他槽位清零并发送，否则会覆盖同组 pitch 或其他电机的输出。第 5–7 组的第四个槽位保留为零。

反馈编码器 0–8191 对应一圈、速度单位 rpm；适配层转换为 rad/rad/s。实际转矩电流反馈先保留原始整数，只有确认本固件的反馈比例后再转换；命令比例不能无证据地套给反馈。

传统 `0x1FF/0x2FF` 是电压帧。本控制器不会把 N·m 直接缩放成电压指令；如果保留旧固件电压模式，需要额外的电流闭环或经过验证的执行器模型。详细接口见 [GM6020 接入说明](../../docs/gm6020_integration.md)。

接入层可使用 [STM32 Yaw/Pitch 周期薄层](../../examples/stm32/gimbal_periodic.c)。多于两轴时先由宿主完成关节参考/重力分配，不能把同一末端世界误差直接发给两个串联关节；三个独立实例的软件测试尚未验证耦合三轴性能，见 [多轴接入](../../docs/multiaxis_integration.md)。
