# GM6020 部分实机调试截图 · 2026-10-07 收录

[项目首页](../README.md) · [移植核查](porting_review_20261007.md) · [图片清单与哈希](evidence/gm6020_hardware_debug_20261007/manifest.json)

用户提供以下 14 张 MemRW3 调试截图，并说明“这些是一部分测试数据”。原图逐字节保留；本页记录配置注释和可见现象。收录日期不代表实际测试日期，尚未取得对应的逐样本日志或每张图的固件版本。

用户同时确认 **GM6020 Pitch 已在整机上实际可用**。以下保存的是部分调试过程，不是统一工况下的最终性能验收。

## Yaw：Q、角速度反馈与加速度前馈

用户已确认，`400,1` / `600,2` 是 Q 的两项，**R = 1**，图中角度单位为 **rad**，“速度反馈”指**实测角速度反馈**。目标速度前馈是否启用未说明，须与角速度反馈分别记录。当时的模型、采样周期及实际 K 尚未逐图绑定，不能只由 Q/R 还原上机增益；关闭角速度反馈的组还需记录实际使用的控制律。

| 编号与原图 | Q | ESO | 实测角速度反馈 | 加速度前馈 | 图片内容 |
| --- | --- | --- | --- | --- | --- |
| [Y01](evidence/gm6020_hardware_debug_20261007/yaw_01.png) | diag(400,1) | 关 | 关 | 关 | `recepakge.yaw` 与 `gimbal.gimbal_lqr_.yaw_ang_` 两路曲线 |
| [Y02](evidence/gm6020_hardware_debug_20261007/yaw_02.png) | diag(400,1) | 关 | 开 | 关 | 同名两路曲线，单独的一段采集窗口 |
| [Y03](evidence/gm6020_hardware_debug_20261007/yaw_03.png) | 未分配 | 未分配 | 未分配 | 未分配 | 原注释为“分割线”；画面显示 `gimbal.err_yaw_`，不擅自归入前后参数组 |
| [Y04](evidence/gm6020_hardware_debug_20261007/yaw_04.png)、[Y05](evidence/gm6020_hardware_debug_20261007/yaw_05.png) | diag(600,2) | 关 | 开 | 关 | 分别为两路曲线与误差信号 |
| [Y06](evidence/gm6020_hardware_debug_20261007/yaw_06.png)、[Y07](evidence/gm6020_hardware_debug_20261007/yaw_07.png) | diag(600,2) | 关 | 开 | 开 | 分别为误差信号与两路曲线 |

Y01/Y02 中两路曲线有可见相位差；Y04/Y07 中两路波形更接近，但仍有峰值、相位或局部偏差。由于图间中心角、幅值、时间窗口及运行配置并未完整配对，本页不把这种视觉差别换算成误差下降百分比，也不把改善唯一归因于某一个开关。

<details>
<summary>展开 Yaw 原图：Q=diag(400,1)</summary>

![Y01：无 ESO、无角速度反馈、无加速度前馈](evidence/gm6020_hardware_debug_20261007/yaw_01.png)

![Y02：无 ESO、有角速度反馈、无加速度前馈](evidence/gm6020_hardware_debug_20261007/yaw_02.png)

</details>

<details>
<summary>展开 Yaw 原图：Q=diag(600,2)，无加速度前馈</summary>

![Y04：两路 Yaw 曲线](evidence/gm6020_hardware_debug_20261007/yaw_04.png)

![Y05：Yaw 误差信号](evidence/gm6020_hardware_debug_20261007/yaw_05.png)

</details>

<details>
<summary>展开 Yaw 原图：Q=diag(600,2)，有加速度前馈</summary>

![Y06：Yaw 误差信号](evidence/gm6020_hardware_debug_20261007/yaw_06.png)

![Y07：两路 Yaw 曲线](evidence/gm6020_hardware_debug_20261007/yaw_07.png)

</details>

## Pitch：小幅参考响应、姿态估计对比与模式记录

用户确认下列三元组属于 **ESO 参数**，并确认 **LQG = LQR + Kalman 状态估计**。三个 ESO 参数的具体名称、排列顺序、单位，以及 P01–P03 与前两组设置的逐图对应关系尚未明确，按原注释保留。LQG 的具体实现和 Kalman 参数尚未绑定到对应固件版本。

| 编号与原图 | 用户配置注释 | 可见内容 |
| --- | --- | --- |
| [P01](evidence/gm6020_hardware_debug_20261007/pitch_01.png) | 前三张共同注释为 ESO 参数：`100,1000,0.03；60,900,0.08`，逐图对应待确认 | `set_pitch_` 与控制器内部 Pitch 角的小幅方波响应；存在直流偏移 |
| [P02](evidence/gm6020_hardware_debug_20261007/pitch_02.png) | 同上 | `INS.Pitch` 与 `QEKF_INS.Roll` 的姿态估计对比；不是目标角与实际角的控制跟踪图 |
| [P03](evidence/gm6020_hardware_debug_20261007/pitch_03.png) | 同上 | `pitch_ang_ref_` 与 `pitch_ang_`；可见参考跳变、有限上升时间及偏移 |
| [P04](evidence/gm6020_hardware_debug_20261007/pitch_04.png) | ESO：`100,1000,0.03` | 小幅方波参考响应，带可读时间轴 |
| [P05](evidence/gm6020_hardware_debug_20261007/pitch_05.png) | ESO：`100,2500,0.03` | 多周期小幅方波参考响应；截图未显示完整时间刻度 |
| [P06](evidence/gm6020_hardware_debug_20261007/pitch_06.png) | `lqg`：LQR + Kalman | 反馈在零附近响应，参考与反馈仍有幅值差；界面另有调试探针断连提示 |
| [P07](evidence/gm6020_hardware_debug_20261007/pitch_07.png) | `lqg+eso`：LQR + Kalman + ESO | 零附近小幅参考响应；保留为另一模式的试验片段 |

P02 的两个变量名称分属 Pitch/Roll，记录中应补足传感器安装与轴映射后再解释差值；曲线接近不能单独证明两套解算的绝对精度或已消除机动加速度污染。P06 的断连提示是采集上下文，不能直接归因为电机或控制器故障。Pitch 各图单位尚待确认，不将其统一按 rad 或度换算。

<details>
<summary>展开 Pitch 原图：早期小幅响应与姿态对比</summary>

![P01：小幅参考响应](evidence/gm6020_hardware_debug_20261007/pitch_01.png)

![P02：INS 与 QEKF 姿态对比](evidence/gm6020_hardware_debug_20261007/pitch_02.png)

![P03：Pitch 参考与反馈](evidence/gm6020_hardware_debug_20261007/pitch_03.png)

</details>

<details>
<summary>展开 Pitch 原图：两组 ESO 参数</summary>

![P04：100,1000,0.03](evidence/gm6020_hardware_debug_20261007/pitch_04.png)

![P05：100,2500,0.03](evidence/gm6020_hardware_debug_20261007/pitch_05.png)

</details>

<details>
<summary>展开 Pitch 原图：lqg / lqg+eso 标签</summary>

![P06：lqg](evidence/gm6020_hardware_debug_20261007/pitch_06.png)

![P07：lqg+eso](evidence/gm6020_hardware_debug_20261007/pitch_07.png)

</details>

## 已纳入移植记录的经验

1. **分别记录反馈与前馈。** `-Kω·ω` 的实测角速度反馈、`+Kω·ω_ref` 的目标速度项，以及 `J·a_ref` 的加速度前馈分别列开关，避免一句“速度开了”掩盖实际接线。
2. **配置与图绑定。** 每次记录 Q 的状态顺序、完整 R、实际 K、ESO/估计器开关、测量及参考滤波、限幅和固件哈希。此批图的调试符号与私有归档并不完全相同，不能把归档的全部参数自动套在每张图上。
3. **保留采集时间语义。** 界面显示的 `Hz` 不直接代表固件控制周期。DWT 的 dt、CAN/IMU 源戳和监控读取时间应分别记录；截图中的“无 log”也不能替代逐样本日志。
4. **保留中间状态。** 直流偏移、相位差、误差尖峰及采集断连都留在记录里，用来说明后续改动的来由；有完整原始时序后再计算统一口径的 RMSE、峰值、延迟和调节时间。

原图及 SHA256 见本页清单。后续日志可通过相同编号关联，补充配置和统计结果。
