# 文档导航

[返回项目首页](../README.md)

本仓库提供运行在 STM32 上的 C 控制器、协议适配和电脑端仿真。按当前任务选择阅读路径即可，不需要从头读完所有报告。

| 你现在要做什么 | 建议阅读顺序 |
| --- | --- |
| 第一次克隆，先在电脑上运行 | [快速开始](quickstart.md)：依次编译 C 库、运行测试和仿真 |
| 先看项目是否适合自己的云台 | [项目首页](../README.md) → [电机版本](#选择电机与协议) → [验收结论](validation_report.md) |
| 接入已有 STM32 工程 | 对应电机接入说明 → [STM32 接入](stm32_integration.md) → [单轴实验步骤](experiment_plan.md) |
| 核查“优化”是否有证据 | [验收总览](validation_report.md) → [Pitch 独立验证](pitch_validation.md) → [复跑仿真](simulation.md) |
| 扩展到大 Yaw、小 Yaw、Pitch | [多轴接入边界](multiaxis_integration.md) → [Pitch 坐标与负载](pitch_integration.md) |

## 了解控制原理

- [原代码解读与改动依据](original_code_review.md)：了解固定版本上游代码的控制结构，以及本库每项改动的原因。
- [控制模型与 C 代码对应](control_design.md)：把动力学前馈、离散 LQR、ESO、积分和限幅对应到实际实现。
- [Pitch 资料与坐标约定](pitch_sources.md)：区分世界姿态、机械关节角和重力角，并追溯重力模型与官方例程依据。

## 选择电机与协议

先进入 [DM4310 MIT 版本](../variants/dm4310/README.md) 或 [GM6020 电流版本](../variants/gm6020/README.md)，查看所需文件、接入顺序和该版本的验证结论。

- [DM4310 接入说明](dm4310_integration.md)：核对 MIT 纯力矩通道、实际映射范围、反馈解析及编码失败处理。
- [GM6020 接入说明](gm6020_integration.md)：核对电流模式前提、力矩换算、完整 CAN 组帧及反馈比例。
- [参数来源与假设](parameter_sources.md)：区分厂商规格、开源参考值和本项目未实测的负载假设。
- [两电机 Yaw 仿真配置](motor_profiles.md)：对照两套参数、协议量化、执行器近似和 GM 积分调参的收益归因。

## 移植到 STM32 与 RoboMaster 框架

- [STM32 接入说明](stm32_integration.md)：说明周期回调、真实 `dt`、一致快照、数据年龄和故障复位的接口契约。
- [官方 C 板例程接入位置](opensource_integration_notes.md)：按官方 `19.gimbal_task` 的任务与设备分层定位替换点。
- [其他 RoboMaster 框架核对](rm_framework_porting.md)：比较已核对开源框架的接口，并注明实际阅读范围。
- [单轴实验步骤与记录](experiment_plan.md)：安排协议核对、负载辨识、逐项开启控制功能和硬件 A/B 测试。

## 阅读验证结果与复跑实验

先读结论，再按需要展开证据：

- [最终仿真验收报告](validation_report.md)：汇总已通过的门槛、GM 配置收益、失败工况及软件验证范围。
- [Pitch 独立验证](pitch_validation.md)：查看同 Ki 对照、重力前馈消融、载荷失配和各分组验收结果。
- [仿真与调参说明](simulation.md)：了解 Python 如何调用真实 C 控制器，以及如何生成参考和复跑实验。
- [性能验收口径](performance_acceptance.md)：核对 Yaw 主工况、开发/验证种子和预先确定的性能门槛。

进一步核查实现和模型：

- [原版缺陷回归对照](regression_evidence.md)：复跑异常输入、在线降额、异常周期与陈旧反馈四项边界测试。
- [参数敏感性与鲁棒性](robustness.md)：检查惯量、阻尼、噪声、延迟和执行器限制变化时的收益与退化。
- [局部离散动力学](local_dynamics.md)：对照解析矩阵和真实 C 的局部线性化，并保留局部不稳定工况。
- [积分步长核验](integration_convergence.md)：检查 Yaw 结果是否对 RK4 子步数从 4 增至 8 敏感。
- [独立检查记录](independent_review.md)：说明交叉检查与 Claude 辅助分析的范围，区分审阅判断和运行证据。

**当前证据边界：** 上述结果来自合成仿真与软件测试，尚无本项目的电机实测。GM6020 Yaw 的主要收益包含积分配置变化；GM6020 Pitch 开发组未通过预定门槛，Pitch 总验收仍为 `false`。

## 接入 Pitch 与扩展多轴

- [Yaw/Pitch 接入与重力负载](pitch_integration.md)：配置独立轴实例，接入重力角、机械边界与前后周期已知负载。
- [多轴参考与连续角](multiaxis_integration.md)：说明连续 Yaw、最近等价圈助手和串联关节的上层参考分配责任。

大 Yaw、小 Yaw、Pitch 可以各分配一个控制器实例；目前公开库验证了实例状态隔离与角度助手，**尚未实现并验证大小 Yaw 协调层及三轴耦合闭环**。
