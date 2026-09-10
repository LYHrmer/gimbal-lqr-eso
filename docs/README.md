# 文档导航

[返回项目首页](../README.md)

第一次接入，按下面五步阅读；已有工程可以直接进入对应环节。仓库提供 C 控制器、协议适配和电脑端实验，宿主工程负责参考与坐标、模式管理及 CAN 调度。

| 步骤 | 首选入口 | 要解决的问题 |
| --- | --- | --- |
| 1. 选择电机 | [DM4310 MIT](../variants/dm4310/README.md) · [GM6020 电流版](../variants/gm6020/README.md) | 驱动模式、协议范围、所需文件是否匹配 |
| 2. 跑通 C 测试 | [快速开始](quickstart.md) → [仿真说明](simulation.md) | 先编译实际 C 代码，再运行电脑端实验 |
| 3. 辨识参数 | [辨识与整定](system_identification.md) → [单轴实验步骤](experiment_plan.md) | 如何校准数据、得到每轴模型并验证候选参数 |
| 4. 接入 STM32 | [周期接口](stm32_integration.md) → [实机前软件检查](prehardware_review.md) | 参考、反馈、力矩提交和故障恢复怎样接入 |
| 5. 核对验证收益 | [Yaw 验证](validation_report.md) · [Pitch 验证](pitch_validation.md) | 改善来自哪里，哪些对照与工况仍未通过 |

## 选择电机与协议

- [DM4310 接入](dm4310_integration.md)：MIT 纯力矩通道、实际映射范围、反馈解析与编码失败处理；当前未实现一拖四模式。
- [GM6020 接入](gm6020_integration.md)：电流模式前提、力矩换算、完整 CAN 组帧与反馈比例。
- [参数来源与假设](parameter_sources.md)：区分厂商规格、开源参考值与未实测的负载假设。
- [两电机 Yaw 仿真配置](motor_profiles.md)：参数、协议量化、执行器近似，以及 GM6020 积分配置收益的归因。

## 跑通测试与理解控制器

- [快速开始](quickstart.md)：从默认 7 项 CTest 开始，安装 Python 依赖，再按目的复跑实验。
- [仿真与调参](simulation.md)：Python 如何调用真实 C 控制器，如何生成参考与合成被控对象。
- [控制模型与 C 实现](control_design.md)：动力学前馈、离散 LQR、ESO、积分和限幅对应哪些代码。
- [原代码解读](original_code_review.md)：固定版本上游的控制结构，以及本库各项改动的依据。

## 模型辨识与参数整定

- [辨识与整定](system_identification.md)：力矩标度、时间对齐、惯量/阻尼/摩擦/重力，以及哪些环节已经实现。
- [DM4310 专项辨识](dm4310_identification.md)：MIT 模式的真实反馈、分阶段辨识、最小二乘与一拖四区别。
- [在线 RLS 实验工程](../experimental/online_rls/README.md)：独立 C 估计器、主机数据窗口、构建与复现命令。
- [RLS 合成验证与反例](../results/online_rls/README.md)：参数恢复、更新冻结，以及噪声、标度和时序错误带来的偏差。

**RLS 是默认关闭的可选实验。** C 估计器只输出候选参数，窗口与激励判断目前运行在 Python；完整实测采集前端、参数采用流程和运行中增益切换尚未实现。参数拟合与闭环性能需要分别验证。

## 接入 STM32 与 RoboMaster 框架

- [STM32 周期接口](stm32_integration.md)：真实 `dt`、一致反馈快照、来源时间、提交回调和故障复位。
- [官方 C 板例程接入位置](opensource_integration_notes.md)：按官方 `19.gimbal_task` 的任务与设备分层定位替换点。
- [其他 RoboMaster 框架核对](rm_framework_porting.md)：已核对框架的接口与实际阅读范围。
- [Yaw / Pitch 接入](pitch_integration.md)：独立轴实例、重力角、机械关节边界与前后周期已知负载。
- [实机前软件检查](prehardware_review.md)：编译选项、Pitch 端点、RLS 异常恢复的失败复现与修复证据，以及仍待硬件验证的项目。
- [单轴实验步骤与记录](experiment_plan.md)：协议核对、负载辨识、逐项开启控制功能和硬件 A/B 测试。
- [实机数据留存与空白模板](test_data_recording.md)：依据 Rudder 原版/移植版逐项核对已有反馈、软件量和取样位置。

三轴方案另见 [多轴参考与连续角](multiaxis_integration.md)。目前已有独立实例状态与角度助手；**大 Yaw → 小 Yaw → Pitch 协调层及三轴耦合闭环尚未实现**。Pitch 的姿态、机械关节角与重力角依据见 [坐标资料](pitch_sources.md)。

## 阅读验证结果

先读结论和对照条件，再展开原始证据：

- [Yaw 仿真验证](validation_report.md)：已通过门槛、GM6020 配置收益、退化工况与软件验证范围。
- [Pitch 独立验证](pitch_validation.md)：同 Ki 对照、重力前馈消融、载荷失配及开发/留出组结果。
- [性能验收口径](performance_acceptance.md)：主工况、开发/验证种子与预先确定的门槛。
- [原版缺陷回归](regression_evidence.md)：异常输入、在线降额、异常周期与陈旧反馈的原版/新版对照。
- [参数敏感性与鲁棒性](robustness.md)：惯量、阻尼、噪声、延迟和执行器限制变化时的收益与退化。
- [局部离散动力学](local_dynamics.md)：解析矩阵、真实 C 的局部线性化与局部不稳定工况。
- [积分步长核验](integration_convergence.md)：Yaw 结果对 RK4 子步数从 4 增至 8 的敏感性。
- [独立检查记录](independent_review.md)：交叉检查与 Claude 辅助分析的范围，以及对应运行证据。

当前证据来自合成仿真与软件测试，尚无本项目的电机实测。**GM6020 Yaw 的主要收益包含积分配置变化；GM6020 Pitch 开发组仍未通过，完整 Pitch 验收仍为 `false`。** 各项百分比与对照条件见 [首页结果表](../README.md#验证结果怎么读)；[实机前修复](prehardware_review.md) 证明的是工程可靠性改善，没有新增跟踪精度提升结论。
