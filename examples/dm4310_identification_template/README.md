# DM4310 辨识空日志模板

[DM4310 辨识步骤](../../docs/dm4310_identification.md)

这里包含只有表头的 [events.csv](events.csv) 和待填写的 [metadata.json](metadata.json)。没有实测或合成样本，也没有板级采集程序；此事件格式尚未接入实测拟合工具。

将模板复制到被 Git 忽略的 `hardware_data/` 下，每次独立运行使用新文件夹。先填实物映射、机构坐标、基线与限额，再按下面契约实现记录。元信息中的 `null` 表示未知，不是数值零。

每份模板对应一个物理轴，在元信息中填写轴角色和 CAN 总线；多轴分别记录并保存时钟关联。随数据附完整生效配置、固件版本/未提交补丁及解码脚本，不能只保存最后调过的几个增益。

副本开始写入真实采集数据后，将 `data_kind` 改为 `measured_data`；仓库里的空模板保持 `empty_template_not_measurements`。实验结束填写按事件流统计的 `logging.dropped_events_by_stream` 及计数方法，确认没有丢记录才填 `0`，不可观测时保留 `null`。本机记录队列丢事件计数不等于已完整测得 CAN/驱动侧丢帧。

## 每行是一个事件

所有事件必须带 `run_id、segment_id、event、host_tick`。保留实际时钟计数，在元信息 `timing.host_clock` 中填写来源、单位、分辨率、位宽及回绕/复位规则。只有 `HAL_GetTick()` 时记录其 32 位毫秒计数；不能乘以 1000 冒充微秒精度。有真实微秒计时器时才声明微秒单位。重启需分段，回绕按声明规则展开。

`resolution_s` 填实际记录时钟的量化步长；它不等于任务调度准确度或驱动内部采样精度，三者应分别解释。

每类事件独立保留 `sequence`，每次尝试记录时递增，丢记录不重编号；只有 `TX_SUBMIT/TX_COMPLETE` 共享同一命令序号。并发事件可以时间相同，不得改时间制造先后关系。

| event | 记录时机与字段 |
| --- | --- |
| `TX_SUBMIT` | 交给 CAN 驱动时，保存 CAN 字段、实际编码载荷、`command_requested_nm / command_limited_nm / command_wire_nm` 和 `submit_ok` |
| `TX_COMPLETE` | 驱动能提供发送完成事件时记录，带对应命令序号；不能获得时不生成这一行 |
| `RX` | 每次 CAN 接收时记录原始 CAN 字段、`decode_ok`；成功解码再填状态、位置、速度、转矩估计及温度 |
| `CONTROL` | 每次控制计算时记录真正采用的三个参考量、`actual_dt_s`、控制状态和限幅标记；计算跳过/参考丢弃时注明原因，不用源参考更新冒充实际采用 |
| `GRAVITY` | Pitch 倾角更新时记录 `gravity_angle_rad`；已对齐到主控时钟的来源计数填 `gravity_source_tick`，无法确定则留空并记录延迟假设 |

不是每行都需要填所有列。未观测或不适用字段留空，布尔值用 `0/1`；无效包也保留原始载荷，不复制上一帧的解码值冒充新测量。更详细的故障原因可写入 `notes`。

`submit_ok=1` 只表示 HAL/驱动受理，即使返回 `HAL_OK` 也只能记录为命令代理；`0` 表示提交失败，无法判断时留空。失败或未知区间不作为“本次力矩已执行”的辨识输入，记录保留待分析；不要补零猜测电机力矩。

## 字段含义

| 字段组 | 约定 |
| --- | --- |
| `can_id / dlc / is_extended / is_remote / payload_hex` | CAN 原始信息；正常 MIT 数据载荷为连续 16 个十六进制字符，不含 USB 转接器封装；可由完整载荷重解三个原码，因此不另设原码列 |
| `command_*_nm` | 分别为请求、主控受限值、编码后的名义力矩；提交成功不表示已在电机上施加 |
| `decode_ok / status_raw` | 本库解析结果与完整状态半字节原值；解析成功不等于允许运行 |
| `position_rad / velocity_rad_s / torque_estimate_nm` | 直接来自本库 MIT 解码；转矩估计不是扭矩传感器测量 |
| `mos_temperature_c / rotor_temperature_c` | 保留本库两个反馈温度字段，名称与接口一致 |
| `joint_angle_rad / joint_velocity_rad_s` | 若宿主已完成校准，则在 RX 行填连续关节坐标；元信息写清输出角、方向、额外传动、零位与展开规则 |
| `reference_position_rad / reference_velocity_rad_s / reference_acceleration_rad_s2` | 记录本次控制器实际采用的参考，使用上述关节坐标和 rad、rad/s、rad/s²；与被夹限/丢弃的源参考区分 |
| `actual_dt_s` | 本次传给控制器的实际周期，单位 s；保留所用时钟的分辨率，不伪造细分时间 |
| `gravity_angle_rad` | 本库 Pitch 约定：水平为零、抬头为正，与机械关节零位分开校准 |
| `controller_status / limit_flags` | 主控已有控制状态、限幅标记；位定义/枚举随基线配置一起保存 |

若有额外传动，在元信息的坐标变换和标度说明中写明拟合使用哪一端。`q=r*theta_m+q0`（`r` 含方向）对应 `omega_q=r*omega_m`，理想力矩为 `tau_q=tau_m/r`；效率/摩擦另作处理。`command_*_nm / torque_estimate_nm` 保留电机协议原值，不能与已转换的 `joint_*` 直接混用拟合。先核实反馈已包含哪一级内置减速比，避免重复换算；转换在后处理完成，当前模板不执行计算。

RX 的主控接收时间不是驱动内部采样时间。提交、发送完成、接收与姿态来源时间分别保留；后处理依据时序证据对齐，不能按行号配对或重复使用陈旧帧。

**没有真实力矩、独立相电流、母线电压或电机内部时间戳的必填列。** 若外接设备能够测得这些量，可另存原始记录与同步依据，不能用名义命令填成“实测值”。正反向、不同姿态和载荷的原始记录应保留，数据划分按完整运行填写到元信息中。
