# RoboMaster 框架接入核对

核对日期：2026-09-09。主接入方式仍为 [RoboMaster 官方 C 板例程](opensource_integration_notes.md)。其他开源用于检查数据和设备层接口，不引入 ROS、外部任务框架或其实现代码，也不把别队机构参数作为本项目实测值。

## 来源与核对范围

| 仓库与固定版本 | 本次实际核对到的入口 | 范围 |
|---|---|---|
| [`HNUYueLuRM/basic_framework@1a136eb1ed2f101110348d5a5cd182272eada38e`](https://github.com/HNUYueLuRM/basic_framework/tree/1a136eb1ed2f101110348d5a5cd182272eada38e) | [`application/gimbal/gimbal.c`](https://github.com/HNUYueLuRM/basic_framework/blob/1a136eb1ed2f101110348d5a5cd182272eada38e/application/gimbal/gimbal.c)：`GimbalInit()`、`GimbalTask()`、`DJIMotorChangeFeed()`、`DJIMotorSetRef()` | 已读取主分支源码并在同次查询解析 master SHA；固定版本文件重取因网络中断未完成。源码将 yaw 接到 `YawTotalAngle` / `Gyro[2]`，pitch 接到 `Pitch` / `Gyro[0]`，并配置角度、速度 PID。 |
| 同一 HNU 版本 | [`modules/motor/DJImotor/dji_motor.c`](https://github.com/HNUYueLuRM/basic_framework/blob/1a136eb1ed2f101110348d5a5cd182272eada38e/modules/motor/DJImotor/dji_motor.c)、[`motor_def.h`](https://github.com/HNUYueLuRM/basic_framework/blob/1a136eb1ed2f101110348d5a5cd182272eada38e/modules/motor/motor_def.h)、[`ins_task.h`](https://github.com/HNUYueLuRM/basic_framework/blob/1a136eb1ed2f101110348d5a5cd182272eada38e/modules/imu/ins_task.h) | 树和文件路径已核对；正文获取失败，因此未确认该版本开环旁路的枚举值、反馈内部单位或 CAN 组帧实现。不能据本页直接配置其电机模块。 |
| [`rm-controls/rm_controllers@a4dbc9b352e52c9039c938ba0fc1f63cfcbe5aa6`](https://github.com/rm-controls/rm_controllers/tree/a4dbc9b352e52c9039c938ba0fc1f63cfcbe5aa6) | [`rm_gimbal_controllers/src/gimbal_base.cpp`](https://github.com/rm-controls/rm_controllers/blob/a4dbc9b352e52c9039c938ba0fc1f63cfcbe5aa6/rm_gimbal_controllers/src/gimbal_base.cpp) 与[头文件](https://github.com/rm-controls/rm_controllers/blob/a4dbc9b352e52c9039c938ba0fc1f63cfcbe5aa6/rm_gimbal_controllers/include/rm_gimbal_controllers/gimbal_base.h) | 提交和目录树已确认，正文因 TLS 超时未读到。本次不据此声称其重力补偿公式、坐标变换或 effort 接口已核实；需补读源码后才形成该框架的具体映射。 |

HNU 示例中的 `OTHER_FEED` 是外部反馈来源配置；从应用层可看到它仍配置角度、速度 PID。因此“把反馈改为 IMU”不等于“关闭旧闭环”。另外，其 pitch 使用 `Gyro[0]`，而[官方固定例程使用 gyro Y](pitch_sources.md)，显示传感器安装映射不能跨工程照搬。

## 本库要求的移植映射

下表是本库的接口契约。涉及未核实的外部内部单位时，先从该工程传感器解码及姿态计算处确认，再做转换。

| 原框架的数据/职责 | 本库入口 | 转换与接入条件 |
|---|---|---|
| HNU `YawTotalAngle`、`Pitch`，或官方 INS 姿态角 | `Stm32GimbalSnapshot.feedback.position_rad` | 使用选定控制坐标的连续输出轴 rad。若原值为度，乘 `pi/180`；为 rad 时不再次转换。安装零点和正方向须标定。 |
| 对应坐标的角速度；HNU 示例为 `Gyro[2]` / `Gyro[0]` | `feedback.velocity_rad_s` | 根据安装和坐标定义投影到所控轴；若源为 deg/s，乘 `pi/180`；若源为 RPM，乘 `2*pi/60`。对一般 Euler 姿态不能把任意机体系分量视为该角度的导数。 |
| 校准零点后的机械编码器角/速度 | `pose.joint_position_rad` / `joint_velocity_rad_s` | 输出轴 rad 与 rad/s；有外部减速传动时按实际传动关系转换。机械角用于限位，不能直接替代相对水平的重力姿态。 |
| 安装校正后的 pitch 姿态及必要的底盘姿态变换 | `pose.gravity_angle_rad` | 与 `gravity_cos_nm`、`gravity_sin_nm` 的辨识坐标一致。本库的平面模型不代表任意滚转或转轴倾斜情况下的整车变换。 |
| HNU `gimbal_cmd_recv.yaw/pitch`，或官方上层参考 | `reference.position_rad`，以及速度、加速度字段 | 单位依次为 rad、rad/s、rad/s²。上层在机械可达区间内生成相互一致的轨迹；不能将原位置设定直接冒充力矩。 |
| 传感器更新、命令产生、姿态融合的真实时间 | 三路 `*_source_us` | 同一单调时钟域，保留数据产生时间。一个字段组合多源时取最旧时间，同时在宿主检查每个源未超前；最小时间戳无法检测被它掩盖的另一个未来时间戳。 |
| 遥控授权、驱动在线、温度/故障和确认的运行模式 | `drive_ready` 及 `valid` | 宿主聚合实际状态；`read_snapshot` 成功仅表示读到一致快照，不代表电机可运行。 |
| 新控制器的总电机力矩 | `submit_torque_nm` | N·m 已含重力补偿与限幅，设备层只做明确模式下的力矩编码和发送。旧角度/速度 PID 不应再次处理此输出。 |

## 设备模块必须保留的职责

1. **一个闭环输出所有者。** HNU 应在核实电机模块实际旁路路径后关闭对应旧角度/速度控制，或直接在设备层接收本库力矩。不能因为设置 `OTHER_FEED` 就假定旧 PID 被旁路。本页没有核实可直接使用的开环枚举组合，因此不给出未经源码验证的配置代码。
2. **一个 CAN 组帧所有者。** yaw、pitch 各有独立控制状态；同组 GM6020 的命令由一个设备层所有者收集并组帧。原驱动任务与新增回调不能竞争同一 CAN ID，也不能在单轴更新时清空另一轴槽位。
3. **模式与标度由原厂协议确认。** [GM6020 本库适配](gm6020_integration.md)针对新电流模式 `0x1FE/0x2FE`，不能把旧电压模式 `0x1FF`、±30000 的调用原样套用。即使旧变量叫 `current`，也不表示它是 A 或 N·m。
4. **DM4310 MIT 使用纯力矩。** 本库 `dm_mit_encode_torque()` 将 MIT `Kp`、`Kd` 设为零，设备层不得再开启驱动端位置/速度增益形成另一组闭环；PMAX、VMAX、TMAX 与应用力矩上限必须分别确认。GM 组帧接口不能用于 DM MIT。
5. **停机由宿主执行。** 设备层接受命令不等于力矩已经执行；发送监控、队列撤销与驱动停机仍归宿主。故障/暖机零值不能保持不平衡 pitch。

## 当前 API 的边界

对于已定义的固定平面 pitch 与 yaw，现有 `Stm32GimbalSnapshot` 足以表达所需量，不必为了模仿外部框架增加 ROS 或对象依赖。但 `pose_source_us` 只能保留最旧源时间，宿主必须在合并前校验全部源的有效性及时间；若后续提供现成多源融合适配器，应将这一步纳入该适配器及其测试。

`joint_reference = joint_position + reference.position - feedback.position` 是当前机械参考映射的平面约定。若日后支持任意底盘滚转/转轴安装，更具体的扩展需求是宿主显式提供经过完整运动学变换的机械参考与重力投影，并独立携带其有效性/来源时间；不能通过只添加一个姿态角字段就宣称已经支持。详细坐标和停机要求见 [pitch 接入说明](pitch_integration.md)。
