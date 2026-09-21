# Rudder / GM6020：实机测试数据留存

[项目首页](../README.md) · [GM6020 辨识](gm6020_identification.md) · [DM4310 辨识](dm4310_identification.md) · [空白记录模板](../examples/gm6020_rudder_template/README.md)

**先保留一份主日志、实际配置和测试说明。字段以老工程真正接收或计算的量为准。** 本页核对了用户提供的 Rudder 云台源码及此前交付的 LQR 移植版，追踪实际赋值路径，而不只看结构体声明。公开仓库提供字段映射和空白模板，整车源码不在此仓库中。

当前两轴为 GM6020：Yaw 是 `motor_[0]`，CAN1 反馈 `0x206`；Pitch 是 `motor_[1]`，CAN2 反馈 `0x205`。原版和移植版都向 `0x1FE` 发命令，分别用第 2、第 1 槽；不能描述为原版电压帧、移植版才改电流帧。实物固件及电流模式仍需现场确认。工程里的 DM 驱动用于拨弹支路，不能把它的反馈当成这两轴的反馈。

**“变量已有”不等于“已经自动记录”。** 主日志中的序号、记录时间和导出动作仍需加入记录器；私有字段要在其所属函数内复制或提供只读诊断接口。本次只提供记录规范与模板，未修改整车固件或交付 ZIP。

2026-09-21 补充了用户另行提供的 [早期 LQR + LESO 整机源码与仿真回顾](lqr-leso-testing.md)。该 ZIP 尚未调定，手动分支存在补偿旁路，接口也不同于下文的 `RudderLqrCompute` 移植版；复用本记录规范时须按实际源码重新映射，不能把配置常量或仿真表当作已经导出的实机日志。

## 1. 老工程实际能拿到哪些量

| 数据 | 已核对的代码入口 | 含义与限制 |
| --- | --- | --- |
| 连续 Yaw、Pitch、Roll | `INS.YawTotalAngle / Pitch / Roll`，`INS_Task()` 每次更新 | 单位 deg；Yaw 已展开。它们是 IMU 融合姿态，不是电机编码器角 |
| 三轴陀螺数据 | `INS.Gyro[0..2]` | 安装轴映射、偏置处理后的 body 角速度，单位 rad/s；头文件注释不能把它变成世界欧拉角导数 |
| 电机展开角 | `gimbal.motor_[i].GetAngle()` | deg，由编码器码、圈数和零偏计算；不代表机械零位已经标定 |
| 电机速度原码 | `GetSpd()` | int16 原码，工程按 RPM 协议约定提供换算常量；主日志保留原码，不混成 IMU 角速度 |
| 电流反馈原码 | `GetCurrent()` | int16 原码；老工程没有反馈侧原码到 A 的标定换算，不命名为真实安培或力矩 |
| 编码器原码与单温度字节 | `DjiMotor::Update()` 解码 `rx_buff[0..1]` 和 `[6]` | 数据确实存在，但字段私有且温度没有 getter；可在完整收包回调记录原始帧，或补只读接口。不是 MOS/转子两路温度 |
| 实际参考位置 | 原版当前被调用 PID 的 `ref_`；移植版 `input.reference[i].position_rad` | 原版手瞄 `angle_[i]` 为 deg、自瞄 `acc_[i]` 为 rad；字段私有，需在调用处加只读取样。`set_yaw_ / set_pitch_` 只是目标成员，部分路径不等于实际采用参考；没有现成 `GetRef()` |
| 自瞄原始目标 | `recepakge` 的 yaw/pitch 位置、速度、加速度及 mode | 下行参考为 rad、rad/s、rad/s²；接收值与限幅、跨圈处理后真正采用的参考分别记录 |
| 模式和停止条件 | `gimbalmode`、`aimingFlag`、`board_comm.GetGimbalStopFlag()` | 软件状态；老版自瞄是否执行还取决于模式与 aimingFlag 的组合 |
| 离线检测状态 | `offline_detect.GetState(DeviceId::DEV_INS / DEV_YAW_MOTOR / DEV_PITCH_MOTOR)` | 已接入 Init/Feed 的粗粒度状态；不是收到首帧或数据新鲜的保证，见下文 |
| 控制命令原码 | `gimbal.output_speed_[0/1]` | int16 命令，不是速度反馈；还要区分它与 CanTask 最后真正提交的值 |

原版 `INS.dGyro` 虽有声明，未发现更新路径，不列为已有角加速度反馈。视觉下行的 `yaw_acc/pitch_acc` 是**目标加速度**，也不是实际测得的角加速度。

以下量**当前工程没有，第一轮不要求填写**：真实轴端力矩、电机电压、电机真实采样时间、CAN 发送完成时间、两路驱动温度，以及已标定的独立 Pitch 机械关节角/重力倾角。移植版保留原 IMU 参考范围和原重力拟合式，没有因此补出这些测量。

## 2. 第一轮最小主日志

模板 [gimbal.csv](../examples/gm6020_rudder_template/gimbal.csv) 一行同时保留两轴，字段均对应上表已有数据，另加记录器自身的序号和记录时刻：

| 字段组 | CSV 字段 |
| --- | --- |
| 记录与模式 | `sample_seq,record_tick_ms,gimbal_mode,aiming_flag,stop_flag` |
| 离线检测上下文 | `ins_offline_state,yaw_motor_offline_state,pitch_motor_offline_state` |
| 参考与融合角 | `yaw_ref_deg,pitch_ref_deg,yaw_total_deg,pitch_deg,roll_deg` |
| 陀螺数据 | `gyro_x_rad_s,gyro_y_rad_s,gyro_z_rad_s` |
| 电机反馈 | `yaw_motor_angle_deg,pitch_motor_angle_deg,yaw_motor_speed_raw,pitch_motor_speed_raw,yaw_current_raw,pitch_current_raw` |
| 控制计算命令 | `yaw_control_word,pitch_control_word` |

`sample_seq` 在每次记录机会递增，丢记录不能偷偷重编号。`record_tick_ms` 可取本工程已有的 `HAL_GetTick()`，是 **32 位主控毫秒记录时刻**，不是传感器采样时刻，也不是微秒精度。同毫秒的多行允许存在；保持序号，记录重启/回绕。上电后开启新 `run_id/boot_id`，不能把多次上电拼成一条时间连续试验。

三个 `*_offline_state` 保存 `Ignore=0 / Online=1 / Offline=2`。旧检测器 Init 就置 Online，检测任务先延迟 20 s、随后约每 100 ms 刷新；INS 的 Feed 条件也不等同于新桥接的有效性检查。因此 Online 不能证明收到首帧或当前数据新鲜；`IsOnline()` 还会将 Ignore 当作 true，不能替代原始状态。严格判定需保存实际生产者首帧/接收时刻，移植版优先记录下文已有的 input valid/age。

在使用位置附近复制这些值，再由低优先级任务导出。原版 INS、参考和控制不是完整原子事务，不能靠在另一个任务随手读取所有全局变量就声称拿到了同一次控制的快照。旧版必须在 PID 调用位置提供只读的实际 `ref_` 取样：手瞄取 `angle_[i]`，自瞄取 `acc_[i]` 并转换 rad→deg。若尚未接出实际参考，`*_ref_deg` 留空；请求目标可另加 `*_requested_deg`，不能冒充实际参考计算 RMSE。

例如老版在 `aimingFlag=1`、视觉 `mode=0` 的部分回退路径中，手动 setter 更新 `set_* / angle_`，但控制分支仍可能采用 `acc_` 之前的参考。因此这项取样对正常误差统计就有必要。若要逐周期复演 PID，还需同时保留当次 measure、滤波结果和实际周期。

移植版主日志的参考和反馈，应从 `RudderLqrCompute()` 当次 `input` 快照转换成 deg 后写入相同列；不要在计算结束后重新读取可能已经变化的全局目标。三轴 gyro/roll 等辅助量应来自对应完整 INS 发布记录，无法保证同一来源时，另存 IMU 原生流并说明关联方式，不把异步辅助值称为控制输入。

第一轮可用主日志查看位置跟踪、跨圈、模式切换和命令变化。定量 A/B 还需确认参考确实被采用、反馈来源一致且有效，并保留异常/未控制区间；缺少这些依据时只作初步观察。最小日志也不足以证明实际力矩、精确执行时延或绝对惯量。

## 3. 按需增加的记录

### 电机反馈帧：feedback.csv

用于电流原码、温度、编码器或时序排查时，在完整电机收包回调复制记录：

`record_tick_ms,axis_id,can_bus,can_id,encoder_raw,speed_raw,current_raw,temperature_raw,payload_hex`

原始 8 字节是最可靠的复查依据，原生收包速率保存即可。单温度字段从 byte 6 获取，单位随固件/协议核准；没有 getter 时不能在类外直接读取私有 `temp_`。记录时间是主控收到/处理该帧的时刻。原版 CAN 缓存没有电机采样时戳，也没有历史帧队列。

### 实际发送请求：commands.csv

在唯一发送者 CanTask 内，紧靠 `DjiMotorSend()` 记录当次实参：

`record_tick_ms,can_bus,can_id,slot1_raw,slot2_raw,slot3_raw,slot4_raw,hal_submit_status`

原版 `DjiMotorSend()` 为 void，会丢掉底层返回值，`hal_submit_status` 留空；移植版返回并检查 HAL 状态，可记录 `HAL_OK/HAL_BUSY/...`。**HAL_OK 只说明进入邮箱，不能填写成“发送完成”或“力矩已执行”。** 两版都没有应用层 TX 完成记录路径，因此模板不提供虚构的完成时刻。

主日志的控制字与本表分开：STOP 分支可能发零，移植版 `RudderLqrGetCommands()` 还会检查新鲜度并抑制命令。分析发送请求时记录本表的实际槽位值及停止分支，不能只看较早计算的 `output_speed_`。

分析主控发送请求时，区分 `HAL_OK`、失败和未知状态。失败请求不能作为已经施加的输入；`HAL_OK` 也只能作为命令代理，不能证明电机已执行。原版状态留空时，无法确认该次请求是否受理。同毫秒的不同流不能仅凭时间戳认定为同一周期；需要精确关联时另加记录器关联编号。

### 仅移植版有的诊断：lqr.csv

在 `RudderLqrCompute()` 调用 C 控制器的位置复制当次 `RudderLqrInput input` 和 `RudderLqrOutput result`。它们是局部变量，当前没有自动日志，也不能假设在别的任务可直接访问。

| 数据 | 实际来源 |
| --- | --- |
| 真正控制间隔 | `input.dt_s`，由 CYCCNT 差值 / SystemCoreClock 得到；不填固定 0.001 |
| 实际输入、参考、age/valid | `input.feedback[i] / reference[i]`；电机新鲜度为 `input.motor_age_s[i] / motor_valid[i]` |
| 有效模式、时序和停止状态 | `input.auto_aim / timing_valid / stopped`，比事后读取全局模式更准确 |
| Nm 命令及分项 | `result.axis[i]` 的 torque、unconstrained、feedforward、feedback、integral、compensation、disturbance、flags/status |
| 控制误差 | `result.axis[i].position_error_rad / velocity_error_rad_s`；速度误差已过可选滤波 |
| 等待、故障及自瞄 Yaw 禁止 | `result.waiting / faulted / auto_yaw_inhibited` |
| 当前力矩限额 | 同一控制任务拥有的 `engine.axis[i].config.torque_limit_nm`，含可能的在线降额 |

反馈、参考和电机 age 依据 bridge 缓存中的 **HAL 毫秒接收/发布时间**，不是电机或相机的物理采样时刻。新移植版的速度是由姿态和 body gyro 换算的欧拉角导数，旧版直接使用 gyro 分量；A/B 时必须记录这一差异，不能混在同一个“实测速度”字段中不加说明。

原版没有 ESO、LQR 状态和上述 `result`，不填零冒充已有诊断。此处 `result` 是控制计算结果；随后 bridge/CAN 仍可能抑制输出，最终发送以 commands.csv 为准。私人移植没有使用通用 `Stm32GimbalPeriodic/GimbalPose` 路径，因此不要求填写这两套公共接口的返回值或姿态字段。

启动、停止、等待、故障或自瞄 Yaw 禁止时，部分分项/误差只是结构体初始化的零，未进行正常控制计算；等待/停止时 `status` 甚至仍可能为 0。保留这些原值和分支状态，**不能仅凭 `status==0` 把诊断零算成零误差**。跟踪误差从有效且新鲜的当次输入 `reference.position_rad - feedback.position_rad` 计算，按轴排除/单独报告未控制分支；故障运行和排除时长仍保留。

### 自瞄：vision.csv

在 `Receive_Vision()` 完整包校验成功的位置记录 `mode,yaw,yaw_vel,yaw_acc,pitch,pitch_vel,pitch_acc` 和主控记录时刻；这些是接收的目标，实际采用的参考另见主日志/LQR 日志。原版全局 `recepakge` 会被后续逻辑改写，仅在另一个任务读取它不能保证保存原始收到的包。当前协议没有可供本日志直接使用的相机曝光时间，不要用接收时刻冒充。

## 4. 实验配置、说明与对比条件

复制 [模板目录](../examples/gm6020_rudder_template/README.md)，每次实验至少保留：

- `metadata.json`：唯一运行/上电编号、原版或移植版、代码提交/未提交补丁、实际固件及哈希、编译选项、实际电机/驱动模式、安装负载、测试轨迹和对照分组。未知实物参数填 null，不能用源码假设冒充已确认。
- `config.json`：实际生效的完整参数。旧版保留 PID 增益、增强项、滤波参数、参考限制及自瞄模型/补偿系数；移植版保留完整两轴 YawConfig、原重力曲线、输出映射、bridge 超时和构建开关。模板是空记录，不是配置加载器。
- `gimbal.csv`：上述最小主日志；其他 CSV 只在启用对应记录后填写。
- `notes.md`：初始姿态、动作顺序、是否完成、异常/中断和日志丢失。故障前后和未完成的运行也保留；必要时另存 `events.csv`，手工标记要说明时间只是估计。

尤其保留 `RUDDER_USE_LQR`、`RUDDER_AUTO_YAW_ENABLE` 的实际构建值。原版自瞄分支存在强制 Yaw 命令清零；移植版默认可保留这一禁止行为，不能将“原版没输出、新版输出了”当作算法精度改善。手瞄/自瞄和启用条件要一致。

参数有变动就另存配置版本并记录生效时刻，不能覆盖旧配置；动态限额也要保存。A/B 使用相同安装、轨迹、限额与初始条件，交换顺序并保留每次重复。辨识、调参和最终验证按完整运行划分，保留参考生成方法或实际序列。

CSV 使用 UTF-8；缺失值留空，JSON 用 null，真实零值才写 0。保留原始单位：老工程角度为 deg，gyro 为 rad/s，原码不擅自换成 A/Nm；导出 SI 数据时另存转换脚本。浮点数保留至少 9 位有效数字，整数时间和原码保持整数。NaN/坏包保留原始记录及异常说明，不伪装成零测量。

优先按真实控制调用记录主日志、按原生速率记录反馈。若抽取或丢记录，保存策略、序号缺口和溢出情况；采样外的峰值不能从低频日志恢复。记录器使用有界缓冲区异步导出，测量它是否影响控制周期，避免阻塞打印。读取 DWT 做记录时使用记录器自己的计数状态，不能复用并修改 PID/INS/bridge 的周期计数器。

当前没有现成 VOFA/JustFloat 日志器。USB CDC 已用于视觉，USART3 是遥控接收，USART6 用于裁判系统，不能默认占用它们输出 CSV。INS、控制、CAN 任务的周期也不同，不能统一写成严格 1 kHz。先确定可用导出通道及带宽，再实现缓冲记录；不要为日志额外调用可能触发保护的 `RudderLqrGetCommands()`，直接复制原有调用取得的值。

## 5. 后续交给我分析时

把每次运行目录压缩，包含原始日志、完整配置、元信息和说明；有二进制日志则附格式及解码脚本。截图、视频和分析结果可附在 `analysis/`，不覆盖原始数据。`hardware_data/` 已被 Git 忽略，先本地保存；公开结果时再附分析方法、数据文件或下载位置及校验值，并标明真实测量。

这份模板覆盖已经核对的两轴 GM6020 工程。DM4310 台架使用[专门的辨识方案与数据模板](dm4310_identification.md)，按本仓库 MIT 解码路径核对；三轴云台仍需按实际机构补充映射。不能因通用驱动存在 getter 或协议有某个字段，就认为整车工程已经更新、导出并标定了它。
