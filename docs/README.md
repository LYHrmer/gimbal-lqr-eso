# 文档导航

[项目首页](../README.md) · [实机图册](hardware_debug_screenshots_20261007.md) · [移植指南](firmware_porting.md) · [快速开始](quickstart.md)

从你要完成的任务开始，先找到步骤，再查接口、模型和验证依据。

## 按任务找入口

| 现在要做什么 | 首选入口 | 下一步 |
| --- | --- | --- |
| **看项目进展** | [实机截图与配置](hardware_debug_screenshots_20261007.md) | [在线辨识与实机视频](gm6020_pitch_identification_followup.md) |
| **第一次运行** | [快速开始](quickstart.md) | [仿真与调参](simulation.md) |
| **接入已有固件** | [实际工程移植](firmware_porting.md) | [周期与回调](stm32_integration.md) → [移植记录模板](../examples/porting_record_template.md) |
| **开始参数辨识** | [GM6020](gm6020_identification.md) / [DM4310](dm4310_identification.md) | [共用原理](system_identification.md) → [数据记录](test_data_recording.md) |
| **理解算法** | [控制模型与 C 实现](control_design.md) | [原代码解读](original_code_review.md) |
| **核对性能结论** | [Yaw 验证](validation_report.md) / [Pitch 验证](pitch_validation.md) | [验收口径](performance_acceptance.md) → [原始结果](../README.md#验证结果怎么读) |

## 实机记录

**GM6020 Pitch 已在整机上实际可用（2026-10-07 用户确认）。** 按材料类型选择入口：

| 材料 | 内容 | 阅读入口 |
| --- | --- | --- |
| **14 张原始截图** | Yaw 的 Q/R、角速度反馈、加速度前馈；Pitch 的 ESO 与 LQG 记录 | [打开图册](hardware_debug_screenshots_20261007.md) |
| **32 秒实机视频** | Pitch 在线辨识、实际机构与调试读数 | [视频与说明](gm6020_pitch_identification_followup.md#4-视频里看到了什么) |
| **整机接入经验** | C++17、重力曲线、时序、方向与输出所有权 | [移植核查](porting_review_20261007.md) |
| **早期测试回顾** | 尚未调定时的参数、旁路、仿真退化与排查过程 | [早期记录](lqr-leso-testing.md) |

这些材料记录下游整机实现；公开 C 内核的性能结果另见本页末尾的软件验证资料。

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
- [GM6020 专项辨识](gm6020_identification.md)：电流模式、命令/反馈比例、Yaw/Pitch 实验与 Rudder 数据取样。
- [DM4310 专项辨识](dm4310_identification.md)：MIT 模式的真实反馈、分阶段辨识、最小二乘与一拖四区别。
- [在线 RLS 实验工程](../experimental/online_rls/README.md)：独立 C 估计器、主机数据窗口、构建与复现命令。
- [RLS 合成验证与反例](../results/online_rls/README.md)：参数恢复、更新冻结，以及噪声、标度和时序错误带来的偏差。

**RLS 是默认关闭的可选实验。** C 估计器只输出候选参数，窗口与激励判断目前运行在 Python；完整实测采集前端、参数采用流程和运行中增益切换尚未实现。参数拟合与闭环性能需要分别验证。

## 接入 STM32 与 RoboMaster 框架

- [实际工程移植步骤](firmware_porting.md)：C++17 宿主的 CMake 接入、自定义重力曲线、模式与辨识边界及信号映射。
- [空白移植记录](../examples/porting_record_template.md)：版本、信号映射、实际功能开关与验证结果。
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

用户已确认 GM6020 Pitch 在整机上实际可用，见[本次记录](porting_review_20261007.md)。针对本仓库 C 内核的可复核性能证据仍来自合成仿真与软件测试，尚无该内核的实机原始日志或硬件 A/B 报告；[早期整机回顾](lqr-leso-testing.md)和[在线辨识跟进](gm6020_pitch_identification_followup.md)记录下游整机的实现与操作。**GM6020 Yaw 的主要收益包含积分配置变化；GM6020 Pitch 合成开发组仍未通过，完整 Pitch 合成验收仍为 `false`。** 各项百分比与对照条件见 [首页结果表](../README.md#验证结果怎么读)；这些软件验收结果不否定用户确认的实机可用性。
