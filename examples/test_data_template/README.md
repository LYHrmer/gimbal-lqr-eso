# Rudder 两轴云台：空白实机记录模板

[完整字段来源与取样位置](../../docs/test_data_recording.md)

适用于已核对的 Rudder 原版和 LQR 移植版：两轴都是 GM6020。这里没有实测示例值、记录器或 CSV 导入程序。**复制模板后还需要在固件中实现对应取样与导出；复制文件不会开始采集。** DM4310 使用[单独辨识方案](../../docs/dm4310_identification.md)。

在仓库根目录执行，例如：

```bash
mkdir -p hardware_data
cp -r examples/test_data_template hardware_data/my-run-001
```

每次上电/实验使用新目录名。`hardware_data/` 已被 Git 忽略。

| 文件 | 什么时候保存 |
| --- | --- |
| [metadata.json](metadata.json) | 必需：固件、实物、测试分组、时间含义、采样策略及缺失说明 |
| [config.json](config.json) | 必需：实际生效的完整参数或所附参数快照文件 |
| [gimbal.csv](gimbal.csv) | 最小主日志：实际采用的目标、姿态、陀螺、电机原码和计算命令 |
| [notes.md](notes.md) | 必需：动作顺序、完成状态、异常和丢记录情况 |
| [feedback.csv](feedback.csv) | 可选：在完整 CAN 收包处复制的原始反馈帧 |
| [commands.csv](commands.csv) | 核对主控发送请求时增加：CanTask 实际发送实参及可取得的 HAL 受理状态 |
| [lqr.csv](lqr.csv) | 移植版诊断时增加：本次 C 控制输入、输出分项与保护状态 |
| [vision.csv](vision.csv) | 自瞄输入排查时增加：CRC 通过处的原始视觉目标 |
| [events.csv](events.csv) | 可选：主控已取得的模式、保护或日志溢出事件 |

**模板中的 null、空对象和表头不是已完成的数据集。** 完成元信息/配置后将 `template_only` 改为 false，`streams_recorded` 只列真实采集的流；未实现的流可删除或保留表头，但在缺失说明中写明。运行失败也保留记录，不能用 0 补齐未测量项。

## 填写约定

- CSV 为 UTF-8、逗号分隔；带逗号或换行的说明按 CSV 规则加引号。缺失值为空，JSON 用 null；布尔日志用 0/1。
- `sample_seq` 从 0 开始，每次**尝试取样**都递增，丢记录保留缺口。`gimbal.csv` 与 `lqr.csv` 只有来自同次控制取样时才共用序号；后者同周期有 yaw/pitch 两行。反馈、发送与视觉流分别按事件记录，不能靠行号与主日志配对。
- `record_tick_ms` 保留 `HAL_GetTick()` 的 uint32 原值，同毫秒多行合法。每次重启换 `boot_id`，回绕与数据缺口另记；离线展开时间，不能当微秒时戳。移植版真实控制间隔记录为 `lqr.csv` 的 `dt_s`。
- 三个 `*_offline_state` 取 `offline_detect.GetState()` 对应 INS/Yaw/Pitch 设备：Ignore=0、Online=1、Offline=2。检测器初始化即置 Online，任务延迟 20 s 启动、随后约每 100 ms 更新；不能据此证明首帧已到或本次新鲜，也不能用将 Ignore 当 true 的 `IsOnline()` 替代。有效性仍需生产者实际记录，移植版保存 input valid/age。
- `axis_id` 用 yaw/pitch，CAN 总线用 CAN1/CAN2，ID 用 `0x206` 形式。`payload_hex` 保存 8 字节，即 16 个十六进制字符；保留前导零。
- deg、rad、rad/s 以列名为准；`*_raw` 和控制字保持整数。电流 raw 不擅自换算为 A/N·m。单温度 byte 6 保留 raw，协议单位另记。
- 主日志 `*_ref_deg` 是实际采用的参考：原版手瞄取 `angle_[i].ref_`，自瞄取 `acc_[i].ref_` 并转成 deg，需要在私有字段所属位置增加只读取样；移植版取当次 `input.reference`。旧 `set_*` 某些路径与实际参考不同，拿不到实际值就留空，不能替代填入。
- 原版和移植版的 `yaw_control_word/pitch_control_word` 均记录该次控制函数返回后的 `output_speed_`，不是 `GetSpd()`；LQR 的 `command_raw` 则是 C 核心 `result.command[i]`，两者可能因 bridge 保护不同。最终发送仍看 `commands.csv`。
- `lqr.csv` 的 `torque_nm` 等为软件命令/估计分项，不是真实轴端测量；`position_error_rad/velocity_error_rad_s` 来自 `result.axis[i]`，后者已过可选误差滤波。`flags` 为位掩码，`status` 用当前 [YawStatus](../../include/yaw_controller.h) 的整数值；不要与 HAL 状态或电机状态混用。
- 启动、停止、等待、故障或自瞄 Yaw 被禁止时，部分诊断零只表示未计算；`status==0` 也不保证正常控制。误差统计从有效新鲜的本次输入计算，并按轴区分实际控制、等待和禁止分支，不能把未计算的零纳入正常跟踪指标。
- `hal_submit_status` 用 HAL_OK/HAL_BUSY/HAL_ERROR/HAL_TIMEOUT 文本；原版函数没有返回值时留空。受理不等于发送完成。
- `events.csv` 只写主控取得的事件；人工估计时刻写在 notes.md，并说明精度。原工程没有自动事件日志。

`metadata.json` 的 `stream_statistics` 按启用流填写 `attempted_records / written_records / dropped_records / high_watermark`；它们需记录器实际计数，没有实现就留空并说明。供电设定值来自实验条件，不是固件反馈的母线电压。模式枚举、单位换算、记录补丁与解码脚本一并保留，便于复查。

`config.json` 不是参数加载器：原版保存两轴 PID、增强项、滤波、参考范围及自瞄模型；移植版保存两轴完整 YawConfig、Pitch 三次保持曲线、命令映射、限额、超时和构建开关。可以把实际参数头/配置快照附到目录，在 `parameter_snapshot_files` 列出相对路径，不能只填写调过的几个增益。原版不存在的构建开关填 null 并解释实际分支行为。

原版自瞄 Yaw 被强制置零，移植版默认也可禁止：必须记录 `RUDDER_AUTO_YAW_ENABLE` 及实际抑制状态，不能将使能差异算成控制器收益。
