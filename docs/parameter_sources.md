# 云台参数证据：DM4310 与 GM6020

核对日期：2026-09-08。机器可读版本见 [`sim/parameter_sets.json`](../sim/parameter_sets.json)。本次只查公开原始资料，没有测量用户硬件。

目前不能给出一个有统计依据的“RoboMaster 正常云台 J/B”。电机规格可以确定到型号与版本，整机惯量、摩擦、线束阻力、间隙和时延则取决于装配。已找到同一作者不同版本的模型参考值，以及其他队伍的任务周期配置；这些不是多台云台的随机实测样本，也不足以建立正常分布。

## 证据分类

| 标签 | 本文含义 |
| --- | --- |
| `hardware_spec` | 对应型号/版本的制造商规格或协议；不自动适用于用户未核对的实物 |
| `team_configuration` | 固定提交中的实际配置常量；证明代码采用该值，不证明物理参数测量准确 |
| `identified` | 作者明确称为辨识结果的候选；本文另外标注原始数据、拟合残差是否可核验 |
| `assumption` | 本项目自行选择的仿真假设或从规格构造的简化模型；敏感性场景另标 `synthetic: true` |

单位换算不增加测量证据。例如 `rpm × 2π/60` 得到 `rad/s`，仍继承原值的来源类别。

## 电机规格与适用版本

### 达妙 DM-J4310-2EC V1.1

制造商手册由 Seeed 镜像提供，封面明确标注 **DM-J4310-2EC V1.1**，文档 **V1.0 / 2023-11-16**。以下为 `hardware_spec`，来源是印刷页 13 / PDF 查看器第 13 页。[手册封面及规格表](https://files.seeedstudio.com/products/Damiao/DM-J4310-en.pdf#page=13)

| 参数 | 数值 | 用于仿真时的含义 |
| --- | --- | --- |
| 额定电压 | 24 V | 不是 48 V 型号的规格 |
| 额定 / 峰值转矩 | 3 / 7 N·m | 峰值不能当成持续可用转矩；表中没有给峰值持续时间 |
| 额定 / 峰值电流 | 2.5 / 7.5 A | 不据此自行推断恒定轴端转矩常数 |
| 额定转速 | 120 rpm = 12.5664 rad/s | 对应额定工况 |
| 最大空载转速 | 200 rpm = 20.9440 rad/s | 不能假设最大转矩与最大空载转速同时可用 |
| 减速比 | 10:1 | 仿真与控制接口统一使用输出轴量 |
| CAN 总线 | 1 Mbit/s | 总线位速率，不是控制回路频率 |

用户仅给出“DM4310”，尚未核对铭牌后缀、电压版和驱动固件，所以不能确认上述规格覆盖用户实物。官方 SDK 固定提交 `0b2ede457bdbf0882e29ab9958ab8fda047b7f4a` 将 `DM4310` 与 `DM4310_48` 分列；示例协议映射分别为 `(12.5 rad, 30 rad/s, 10 N·m)` 和 `(12.5 rad, 50 rad/s, 10 N·m)`。这些是数据编码范围，不能当成电机峰值能力，更不能用 `TMAX=10` 推翻该版本手册的峰值 7 N·m。[官方 SDK 映射表](https://github.com/dmBots/motor-sdk/blob/0b2ede457bdbf0882e29ab9958ab8fda047b7f4a/Python%E4%BE%8B%E7%A8%8B/u2can/DM_CAN.py)

### RoboMaster GM6020

已核对 DJI 官方中文手册 **v1.4 / 2023.10**。下表均为 `hardware_spec`，规格见印刷页 11 / PDF 查看器第 12 页；工作范围的测试条件见印刷页 10。[官方规格及工作范围](https://rm-static.djicdn.com/tem/17348/RoboMaster%20GM6020%E7%9B%B4%E6%B5%81%E6%97%A0%E5%88%B7%E7%94%B5%E6%9C%BA%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8E20231013.pdf#page=12)

| 参数 | 数值 | 注意其物理定义 |
| --- | --- | --- |
| 额定电压 | 24 V | 工作范围图还指定室温 25℃及正常散热 |
| 额定转矩 / 电流 | 1.2 N·m / 1.62 A | 手册将前者称为最大连续转矩 |
| 额定转矩下最高转速 | 132 rpm = 13.8230 rad/s | 不等于空载上限 |
| 最大空载转速 | 320 rpm = 33.5103 rad/s | 不能在此转速同时请求额定转矩 |
| 持续堵转转矩 / 电流 | 0.86 N·m / 0.90 A | 保留厂家独立给出的工况点，不强行令其等于下行常数乘积 |
| 转矩常数 | 0.741 N·m/A | 可作名义电流—转矩换算，实物仍须校核 |
| 转速—转矩梯度 | 156 rpm/(N·m) | 不是装好云台后的机械粘性阻尼 B |
| 机械时间常数 | 3 ms | 不是主控周期，也不是实测电流环纯滞后 |

该表没有单独指定一个可直接使用的“峰值转矩”。电流命令范围乘转矩常数所得 `3 A × 0.741 = 2.223 N·m` 只是**名义协议换算**，不是厂家峰值或连续能力证明。

GM6020 电流控制有版本条件：固件 `>=1.0.11.2`，RoboMaster Assistant `>=2.7`，且通过参数设置打开电流环开关。电流组帧 ID 为 `0x1FE/0x2FE`，整数 `±16384` 对应 `±3 A`；电压模式组帧 `0x1FF/0x2FF` 不能混用。反馈标称发送频率为 1 kHz。这些均来自印刷页 6–7 / 查看器第 7–8 页，属于 `hardware_spec`。[官方电流控制协议](https://rm-static.djicdn.com/tem/17348/RoboMaster%20GM6020%E7%9B%B4%E6%B5%81%E6%97%A0%E5%88%B7%E7%94%B5%E6%9C%BA%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8E20231013.pdf#page=7)

不能用电机转子惯量、机械时间常数或转速—转矩梯度替代包含支架、发射机构、相机和线束的云台总惯量/阻尼。特别是电压驱动下的速度—转矩斜率包含电气效应，不能直接当成力矩输入模型 `J·θ̈+B·θ̇=τ+d` 的机械 B。

## 原作者模型：三个参考点，不能合并为一个“真实值”

| 参考点 | J (kg·m²) | B (N·m·s/rad) | 周期 | 分类与证据边界 |
| --- | --- | --- | --- | --- |
| 独立控制器仓库调参脚本 | 0.039 | 0.30 | 1 ms | `team_configuration`；本项目原始对照的名义点 |
| 同账号整车库当前固件宏 | 0.0610521813 | 0.513734025 | 1 ms | `team_configuration`；宏定义证明采用值，在线 Ozone 调参仍可能覆盖 |
| 同提交整车库调参候选 | 0.0422563489 | 0.286217284 | 1 ms | `identified`，仅限作者报告的候选；未独立复验 |

第一行来自 `YAW_Auto_Controller` 固定提交 `665c5b4ab1067d6cb63122c120822f27502953e5` 的 `yaw_auto_lqr_tune.m` 第 37–42 行。[独立脚本](https://github.com/LamdaDay/YAW_Auto_Controller/blob/665c5b4ab1067d6cb63122c120822f27502953e5/yaw_auto_lqr_tune.m#L37-L42)

第二行来自 `Infantry_RMUC_2026` 固定提交 `f67ef28a8a6aedb047f1e65837330034313e0be8` 的 `modules/lqr_eso/lqr_eso.c` 第 31、46–54 行，另有库仑摩擦补偿 **0.3 N·m** 和代码力矩限额 **7 N·m**。第三行来自**同一提交**的 `tools/yaw_auto_lqr_tune.m` 第 23–26 行。两处注释指向 `yawww.csv` 的 stage-2 `motor_tor_nm` 拟合，但数值不同；不能任取一个声称已经复现了原作者整机辨识。[当前固件常量](https://github.com/LamdaDay/Infantry_RMUC_2026/blob/f67ef28a8a6aedb047f1e65837330034313e0be8/modules/lqr_eso/lqr_eso.c#L31-L54)、[辨识候选脚本](https://github.com/LamdaDay/Infantry_RMUC_2026/blob/f67ef28a8a6aedb047f1e65837330034313e0be8/tools/yaw_auto_lqr_tune.m#L23-L26)

本次固定仓库树未找到 `yawww.csv` 原始样本；虽有采样/拟合工具，却没有足以复核上述数值的该批样本、残差、置信区间和负载姿态记录。因此这里没有用户实物的 `identified` 参数，也没有独立验证通过的整机 J/B 数据集。作者在论坛还明确说明这台步兵云台惯量较大，yaw 用一台 DM4310、pitch 用两台；它不能代表所有步兵云台。[固定文件树](https://github.com/LamdaDay/Infantry_RMUC_2026/tree/f67ef28a8a6aedb047f1e65837330034313e0be8)、[作者原文](https://bbs.robomaster.com/article/1935544)

整车固件的 `_GimbalTask` 在 `GimbalTask_YAW/PITCH` 后 `osDelay(1)`，RTOS tick 为 1000 Hz；这支持名义 1 ms 任务配置，但不是实测执行周期或时延分布。[实际任务函数](https://github.com/LamdaDay/Infantry_RMUC_2026/blob/f67ef28a8a6aedb047f1e65837330034313e0be8/application/robot_task.h#L103-L123)、[tick 配置](https://github.com/LamdaDay/Infantry_RMUC_2026/blob/f67ef28a8a6aedb047f1e65837330034313e0be8/Core/Inc/FreeRTOSConfig.h#L68)

## 其他队伍配置：确认周期差异，不推造惯量

| 队伍/项目 | 固定提交 | 查到的证据 | 分类 |
| --- | --- | --- | --- |
| 北京科技大学 Reborn，2024 轮腿步兵云台 | `74866ed236a093f3930ec85c38a594d5c23ec7b1` | README 列 GM6020×2；`GimbalTask()` 后 `osDelay(1)`，tick=1000 Hz | `team_configuration`；名义约 1 ms，不是时序实测 |
| 河北工业大学，2023 RMUL 步兵 | `184be192088e5cc1f2b049250409fa80344f0d9f` | `GIMBAL_PERIOD=5`，`osDelayUntil` 调度，tick=1000 Hz | `team_configuration`；计划 5 ms，不保证实际期限均满足 |

Reborn 的证据：[硬件清单](https://github.com/JerryGong0911/RM2024_Reborn_LegWheel_Gimbal/blob/74866ed236a093f3930ec85c38a594d5c23ec7b1/README.md#L4-L10)、[任务调用](https://github.com/JerryGong0911/RM2024_Reborn_LegWheel_Gimbal/blob/74866ed236a093f3930ec85c38a594d5c23ec7b1/Core/Src/freertos.c#L194-L202)、[tick 配置](https://github.com/JerryGong0911/RM2024_Reborn_LegWheel_Gimbal/blob/74866ed236a093f3930ec85c38a594d5c23ec7b1/Core/Inc/FreeRTOSConfig.h#L64)。河北工业大学的证据：[周期宏](https://github.com/BriMonzZY/mas-infantry-firmware/blob/184be192088e5cc1f2b049250409fa80344f0d9f/application/gimbal/gimbal_task.c#L36-L37)、[周期调用](https://github.com/BriMonzZY/mas-infantry-firmware/blob/184be192088e5cc1f2b049250409fa80344f0d9f/application/gimbal/gimbal_task.c#L239-L242)、[tick 配置](https://github.com/BriMonzZY/mas-infantry-firmware/blob/184be192088e5cc1f2b049250409fa80344f0d9f/bsp/cubemx/Core/Inc/FreeRTOSConfig.h#L64)。

本次检查的这些配置文件没有给出带完整辨识证据的 yaw 总 J/B。河北工业大学这一行只用于证明任务周期配置，不据此推定电机型号。PID 增益、整数电压/电流上限不等价于物理惯量、阻尼或轴端转矩。

也检查了湖南大学跃鹿基础框架：它的电机任务延时 1 tick，但该固定提交中的 `GimbalInit/GimbalTask` 被注释。因而没有把它列成“已经运行的 1 ms 云台”样本。[被注释的调用](https://github.com/HNUYueLuRM/basic_framework/blob/1a136eb1ed2f101110348d5a5cd182272eada38e/application/robot.c#L32-L53)

## 如何约束仿真而不伪装实测

1. 保留独立原版脚本的 `J=0.039/B=0.30` 作为历史公平对照的名义点；把整车宏和候选分别加入明确命名的参数失配场景。同一场景两版控制器接受相同植物参数、参考、扰动和噪声，不能只为新版重新拟合。
2. 将 DM4310 的额定/峰值、GM6020 的额定/持续堵转，以及协议映射范围分开。恒定转矩硬限幅只是简化模型，不能证明热平衡、瞬态持续能力或完整速度—转矩包络。
3. `parameter_sets.json` 中的倍率、噪声、延迟与滞后是 `assumption` 且 `synthetic: true`；它们是未来敏感性测试目录，是否实际运行以结果报告为准。它们不是“正常波动百分比”或置信区间。
4. 原文图名中的 `20deg` 未在本次查阅中建立单边峰值/峰峰值的明确约定。本项目正弦参考必须显式写明振幅定义。对假设的单边振幅 A，有 `ω_peak=2πfA`、`α_peak=(2πf)²A`；线性模型所需转矩振幅为 `sqrt((J·α_peak)²+(B·ω_peak)²)`，另需考虑摩擦。

举例：**假设**单边振幅 20°、`J=0.039/B=0.30`，3 Hz 和 5 Hz 的上述线性需求分别约 5.224 和 13.833 N·m。后者超过该版本 DM4310 手册的 7 N·m 峰值；这是参考可行性计算，不能用来否定或重新解释作者未完全定义的图名。这两个计算结果分类为 `assumption`。

用户硬件还未搭好，因此当前没有最终上机参数。之后应记录实际型号/供电、整机负载与 pitch 角、输出轴方向和零点、执行命令及测量时间戳、实际周期/延迟，再用独立验证数据评估辨识模型；不能仅凭电机型号从本目录直接选一个总 J/B。

## 固定资料摘要

两份 PDF 均实际下载，规格页经图像核对，SHA256 如下；GitHub 文件使用上面的固定提交链接，机器可读目录另记录关键文件哈希与逐项来源。

- DM 手册：`ee371c9197935544d62eb6249ffc449fffad5fc6d687100cda066c49eb131cd5`
- GM6020 v1.4 手册：`8843f783a3a476dc0eb20347995a604e156a6b00d4e41551e0aba0d7e02262a9`
