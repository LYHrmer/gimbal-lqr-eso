<p align="center">
  <img src="docs/assets/project-banner.svg" alt="LQR + ESO：面向 STM32 的双轴云台控制，支持 GM6020 与 DM4310" width="100%">
</p>

# 云台控制器 · LQR + ESO

**从模型辨识、离散控制设计，到 STM32 云台上的力矩输出。**

面向 RoboMaster 云台的 C11 控制器库，支持 **Yaw / Pitch、GM6020 电流模式与 DM4310 MIT 模式**。手瞄、自瞄共用控制内核，Python 负责离线设计和仿真。

[![软件测试与 ARM 编译](https://github.com/LYHrmer/gimbal-lqr-eso/actions/workflows/ci.yml/badge.svg)](https://github.com/LYHrmer/gimbal-lqr-eso/actions/workflows/ci.yml) [![C11](https://img.shields.io/badge/C-11-00599C)](include/yaw_controller.h) [![MIT License](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)

[**快速开始**](#先在电脑上跑通) · [**实机进展**](#实机进展) · [**移植指南**](docs/firmware_porting.md) · [**验证结果**](#验证结果怎么读) · [**全部文档**](docs/README.md)

## 实机进展

**GM6020 Pitch 已在用户整机上实际可用。** 下图来自用户提供的部分调试记录，点击可查看原图；完整配置与说明收录于[实机图册](docs/hardware_debug_screenshots_20261007.md)。

<table>
  <tr>
    <td width="50%" valign="top">
      <strong>Yaw · 角速度反馈与加速度前馈</strong><br>
      <a href="docs/evidence/gm6020_hardware_debug_20261007/yaw_07.png"><img src="docs/evidence/gm6020_hardware_debug_20261007/yaw_07.png" alt="Y07：Yaw 两路角度曲线，Q=diag(600,2)、R=1，无 ESO，有实测角速度反馈和加速度前馈" width="100%"></a><br>
      Q = diag(600, 2)，R = 1，单位 rad。<br>ESO 关闭，实测角速度反馈与加速度前馈开启。
    </td>
    <td width="50%" valign="top">
      <strong>Pitch · LQG + ESO 小幅响应</strong><br>
      <a href="docs/evidence/gm6020_hardware_debug_20261007/pitch_07.png"><img src="docs/evidence/gm6020_hardware_debug_20261007/pitch_07.png" alt="P07：用户标注 LQG + ESO 的 Pitch 小幅参考响应，仍有参考与反馈幅值差" width="100%"></a><br>
      LQG = LQR + Kalman 状态估计。<br>保留参考、反馈和调试时的原始波形。
    </td>
  </tr>
</table>

实机可用状态由用户于 **2026-10-07** 确认；已核查整机使用下游 C++ 实现。公开 C 内核目前完成软件回归和 Cortex-M4F 编译检查，尚无该内核的硬件 A/B 报告。这批截图保存调试过程，不作为同轨迹性能对照；详见[移植核查记录](docs/porting_review_20261007.md)。

[查看全部 14 张原图与配置 →](docs/hardware_debug_screenshots_20261007.md) · [在线辨识与实机视频 →](docs/gm6020_pitch_identification_followup.md)

## 项目提供什么

控制结构为 **动力学前馈 + 离散 LQR 反馈 + 残余扰动 ESO + 可选抗饱和积分**。输入参考与反馈，输出总力矩，交由协议层编码。设计与公式见[控制模型](docs/control_design.md)。

| 层次 | 已提供的能力 |
| --- | --- |
| **控制核心** | 静态状态、无动态分配、无 HAL / RTOS 依赖；检查实际周期、数据年龄与异常数值 |
| **Yaw / Pitch** | 连续角度；Pitch 重力与机械行程独立处理，支持宿主自定义重力曲线 |
| **电机与工程接入** | 两种协议适配、CMake 接入入口、STM32 周期示例；宿主保留任务、模式与唯一 CAN 发送者 |
| **离线验证** | Python 调用真实 C 内核，覆盖跟踪、扰动、失配、延迟及故障回归 |
| **可选实验** | 在线 RLS 默认关闭，只输出候选参数；数据窗口与激励判断目前运行在 Python |

三轴扩展已有独立实例与角度助手；大 Yaw → 小 Yaw → Pitch 协调层及耦合闭环仍待实现，见[多轴说明](docs/multiaxis_integration.md)。仓库提供控制库和接入示例，整车固件沿用宿主工程。

## 选择电机

两种电机使用相同控制核心，区别在参数与力矩编码。先确认自己的驱动模式，再进入对应说明。

| | [DM4310 MIT 版 →](variants/dm4310/README.md) | [GM6020 电流版 →](variants/gm6020/README.md) |
| --- | --- | --- |
| 命令方式 | MIT 纯力矩，`Kp=Kd=0` | 已确认开启的电流模式 |
| 协议适配 | [dm_mit.c](src/dm_mit.c) | [gm6020.c](src/gm6020.c) |
| 必须核对 | 实际 `PMAX / VMAX / TMAX`、设备 ID 和应用力矩上限 | 固件与电流环配置、分组与槽位；使用 `0x1FE / 0x2FE` |
| 参数示例 | [Yaw](variants/dm4310/simulation_config.h) · [Pitch](variants/dm4310/pitch_simulation_config.h) | [Yaw](variants/gm6020/simulation_config.h) · [Pitch](variants/gm6020/pitch_simulation_config.h) |

DM4310 当前实现 MIT 通道，未实现一拖四模式。GM6020 传统电压帧 `0x1FF / 0x2FF` 与这里的电流接口不同；详细版本要求见 [GM6020 接入说明](docs/gm6020_integration.md)。所有示例配置均为 **`simulation_only`**，不能直接视作上机参数。

## 先在电脑上跑通

以下为 Linux 主机测试，只需 C 编译器、CMake ≥ 3.16 和数学库；不需要电机、CAN 设备或 Python。

```bash
git clone https://github.com/LYHrmer/gimbal-lqr-eso.git
cd gimbal-lqr-eso
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

预期 **7 个 CTest 全部通过**。这一步生成的是主机测试程序和库；STM32 目标编译需要对应的 ARM 工具链。

Python 环境、两种电机的仿真和固定原版 C 对照命令见 [快速开始](docs/quickstart.md)。其中的命令将新结果写入 `build/`，便于与已发布结果比较。

## 模型辨识与参数整定

先校准力矩标度、时序和机构坐标，再估计每轴的惯量、阻尼、摩擦与 Pitch 重力。现有 LQR 工具可接收辨识后的 `J/B/dt`；权重、ESO 带宽和积分仍需经过独立闭环验证。

[辨识与整定总览](docs/system_identification.md) 解释数据怎样变成模型，以及哪些工作已经实现。实机操作先选下面对应的一行：

| 使用哪种电机 | 从哪里开始 | 数据与标度重点 |
| --- | --- | --- |
| **GM6020 电流模式** | [6020 辨识步骤](docs/gm6020_identification.md) → [Rudder 数据模板](examples/gm6020_rudder_template/README.md) | 电流反馈原码；先核准反馈比例和转矩常数，命令比例不能直接代用 |
| **DM4310 MIT 模式** | [4310 辨识步骤](docs/dm4310_identification.md) → [MIT 数据模板](examples/dm4310_identification_template/README.md) | 位置/速度/转矩回读；先核准 PMAX/VMAX/TMAX，转矩回读不等于轴端实测 |

两条路径都按 **核准信号与标度 → Yaw/Pitch 分步辨识 → 独立验证 → 整定控制器** 进行。Rudder 模板专门对应已核对的两轴 GM6020 工程；其他宿主需重新映射取样点。模板的字段来源见 [实机数据留存](docs/test_data_recording.md)。

[在线 RLS 实验](experimental/online_rls/README.md) 可先验证 C 候选参数递推；[数值验证与反例](results/online_rls/README.md) 同时保留噪声、时序错位和标度错误造成的偏差。

完整实测日志前端、实测参数导出、自动整定与运行中增益切换尚未实现。辨识得到的候选参数仍需独立轨迹和闭环检验：**参数拟合成功不等于控制性能改善**。

## 接入 STM32

在现有 C/C++ 固件目标创建后，添加两行 CMake 即可引入控制源码：

```cmake
include("${CMAKE_CURRENT_SOURCE_DIR}/third_party/gimbal-lqr-eso/cmake/GimbalController.cmake")
gimbal_add_to_target(your_firmware_target GM6020)
```

DM4310 将第二个参数替换为 `DM4310`。完整接入顺序、自定义重力曲线及模式切换见[移植指南](docs/firmware_porting.md)；周期与回调定义见 [STM32 接口](docs/stm32_integration.md)。

```mermaid
flowchart LR
    R["宿主参考与坐标"] --> C["前馈 + LQR + ESO"]
    F["反馈 / 时间 / Pitch 负载"] --> C
    C --> L["总力矩 / 变化率限制"]
    L --> E["电机协议编码"]
    E --> H["宿主 CAN 与驱动管理"]
```

统一使用 **rad、rad/s、rad/s²、N·m、s**。首次有效周期输出零，故障锁存后需显式复位；零力矩不能代替 Pitch 的机械保持。编译须保留有限数检查，禁止 `-ffinite-math-only / -ffast-math / -Ofast`。详见 [Pitch 接入](docs/pitch_integration.md)和[实机前检查](docs/prehardware_review.md)。

<details>
<summary><strong>源码导览 · 控制核心、协议与周期调用</strong></summary>

| 从哪里读代码 | 作用 |
| --- | --- |
| [yaw_controller.h](include/yaw_controller.h) / [yaw_controller.c](src/yaw_controller.c) | 单轴核心：初始化、周期计算、复位和在线力矩降额 |
| [gimbal_controller.h](include/gimbal_controller.h) / [gimbal_controller.c](src/gimbal_controller.c) | Pitch 重力、机械关节范围与负载历史 |
| [gimbal_coordinates.h](include/gimbal_coordinates.h) | 连续角度与单圈方向目标 |
| [周期示例](examples/stm32/gimbal_periodic.c) / [回调定义](examples/stm32/gimbal_periodic.h) | 反馈快照、实际 dt、力矩提交与停止请求 |
| [DM4310](variants/dm4310/README.md) / [GM6020](variants/gm6020/README.md) | 两种电机的仿真参数与移植文件 |
| [仿真说明](docs/simulation.md) / [tests](tests) | 真实 C 调用、合成被控对象与回归测试 |

沿用宿主现有任务、参考管理和 CAN 调度；分层接入位置见 [RoboMaster 官方例程说明](docs/opensource_integration_notes.md)。

</details>

## 验证结果怎么读

下面是合成模型中，五个独立留出种子的**平均配对位置 RMSE 变化**；负号表示误差减小。Yaw 主工况为 2 Hz、±5°；Pitch 为中心 +20°、1 Hz、±5°。不同工况及不同基准不能合并成一个“总体提升率”。

| 场景 | 对照口径 | 留出组 RMSE 变化 | 可以得出的结论 |
| --- | --- | ---: | --- |
| DM4310 · Yaw | 原 / 新版相同参数，Ki=0 | **−6.90%**，5/5 改善 | 声明的主工况通过 |
| GM6020 · Yaw | 新版慢积分配置 vs 原版 Ki=0 | **−33.68%**，5/5 改善 | 有配置收益 |
| GM6020 · Yaw | 原 / 新版使用相同积分参数 | **+4.00%** | 未证明新内核精度更高 |
| DM4310 · Pitch | 同 Ki 原版对照，新版开启重力补偿 | **−72.53%**，5/5 改善 | 开发组与留出组均通过 |
| GM6020 · Pitch | 同 Ki 原版对照，新版开启重力补偿 | **−2.94%**，4/5 改善 | 留出组通过；开发组未通过，整体合成验收仍为 `false` |

**这些结果不代表所有工况或实机都更好。** GM6020 的 Yaw 收益需要区分配置与内核。CI 检查软件正确性、实验有效性及声明的部分性能门槛，绿色状态不等于所有性能验收通过。

用户已确认 GM6020 Pitch 的实机可用性；上表保留特定合成模型的验收结果，不能据开发组未通过推断实机不能使用。

| 想核对什么 | 证据入口 |
| --- | --- |
| Yaw 的改善、归因与退化工况 | [Yaw 验证报告](docs/validation_report.md) · [两电机曲线与数据](results/motor_profiles/README.md) |
| Pitch 的重力开关对照和未通过项 | [Pitch 验证报告](docs/pitch_validation.md) · [曲线与原始指标](results/pitch/README.md) |
| NaN、降额、异常周期、陈旧反馈 | [原版 / 新版缺陷回归](docs/regression_evidence.md) |
| 参数失配、延迟和数值可信度 | [鲁棒性](docs/robustness.md) · [积分步长核验](docs/integration_convergence.md) |
| 实验模型和参数从哪里来 | [仿真假设](assumptions.md) · [参数来源](docs/parameter_sources.md) |
| 用户独立实现的下游整机固件（非本仓库内核） | [早期整机测试回顾](docs/lqr-leso-testing.md) · [在线辨识与超调跟进 + 实机视频](docs/gm6020_pitch_identification_followup.md) |

早期 7 N·m 挑战条件下的 [原版直接对照](results/upstream_comparison/README.md) 与 [新版 ESO 消融](results/README.md) 单独保留；它们不是两种电机主工况，也不是论坛实验的逐点复现。

## 来源与许可

控制思路参考 Combat 战队的 [LamdaDay/YAW_Auto_Controller](https://github.com/LamdaDay/YAW_Auto_Controller/tree/665c5b4ab1067d6cb63122c120822f27502953e5) 与 [论坛文章](https://bbs.robomaster.com/article/1935544?source=1)。本仓库重新实现控制内核，保留来源归属；差异见 [原代码分析](docs/original_code_review.md)。

新写的源码、文档与合成仿真采用 [MIT License](LICENSE)。固定原版源码由脚本下载到忽略目录，仅用于比较；外部原码与手册不随本仓库重新授权或打包。
