# 按 RoboMaster 官方 C 板例程接入

核对日期：2026-09-08。主要依据为 RoboMaster 官方 `Development-Board-C-Examples`，固定提交 [`59d12b1adcd321dbf1f9e9166aef5eb95ab657bf`](https://github.com/RoboMaster/Development-Board-C-Examples/tree/59d12b1adcd321dbf1f9e9166aef5eb95ab657bf)，具体工程为 `19.gimbal_task`。

本仓库提供独立控制器库与 HAL 无关的薄接入示例，参考官方的任务/设备分层；下述映射是本库的接入建议，不表示官方例程已经实现本库的快照、时间戳或故障契约。官方仓库标注 [GPLv3 或更新版本](https://github.com/RoboMaster/Development-Board-C-Examples/blob/59d12b1adcd321dbf1f9e9166aef5eb95ab657bf/license.txt#L3-L8)，本仓库未复制其任务或 BSP 实现。

## 1. 保留官方 HAL / FreeRTOS 框架

| 官方实际入口与证据 | 映射到本库 |
|---|---|
| [`Src/freertos.c` 创建 `gimbal_task`，高优先级、栈参数 512](https://github.com/RoboMaster/Development-Board-C-Examples/blob/59d12b1adcd321dbf1f9e9166aef5eb95ab657bf/19.gimbal_task/Src/freertos.c#L146) | 在现有任务内初始化一个调用方持有的 `Stm32YawPeriodic`，随后调用 `stm32_yaw_periodic_step()`；本库不创建新调度器、不重新初始化 HAL。栈参数只是原例程配置，不是本库实测需求。 |
| [`gimbal_task()` 的反馈更新→设定→控制→发送流程](https://github.com/RoboMaster/Development-Board-C-Examples/blob/59d12b1adcd321dbf1f9e9166aef5eb95ab657bf/19.gimbal_task/application/gimbal_task.c#L315-L374) | 保留上层模式、校准和参考生成；为 yaw 增加 `YawFeedback` / `YawReference` 适配，再由库输出 N·m。不能让旧 yaw PID 与本库同时写同一电机命令。 |
| [`GIMBAL_CONTROL_TIME=1`](https://github.com/RoboMaster/Development-Board-C-Examples/blob/59d12b1adcd321dbf1f9e9166aef5eb95ab657bf/19.gimbal_task/application/gimbal_task.h#L114)，[`configTICK_RATE_HZ=1000`](https://github.com/RoboMaster/Development-Board-C-Examples/blob/59d12b1adcd321dbf1f9e9166aef5eb95ab657bf/19.gimbal_task/Inc/FreeRTOSConfig.h#L61) | 官方循环末尾是相对 `vTaskDelay(1)`，只能视为名义 1 tick，不能宣称严格 1 kHz。提供同一时钟域的 `monotonic_us` 与源时间戳，薄层以真实间隔计算 dt；调度与截止期管理由宿主负责。 |

## 2. 把中断缓存变成有时间戳的测量快照

官方 [`CAN_receive.c`](https://github.com/RoboMaster/Development-Board-C-Examples/blob/59d12b1adcd321dbf1f9e9166aef5eb95ab657bf/19.gimbal_task/application/CAN_receive.c#L30-L89) 在 `HAL_CAN_RxFifo0MsgPendingCallback()` 中逐字段更新静态 `motor_chassis[7]`，再调用 `detect_hook()`；[`get_yaw_gimbal_motor_measure_point()`](https://github.com/RoboMaster/Development-Board-C-Examples/blob/59d12b1adcd321dbf1f9e9166aef5eb95ab657bf/19.gimbal_task/application/CAN_receive.c#L203-L206) 返回数组元素的只读指针。这个指针本身不是原子快照，也没有本库要求的源时间戳。

在接入层的 `read_snapshot` 中采用短临界区复制或双缓冲发布，得到同一版本的角度、速度、状态与采集时间；随后退出临界区再做控制计算。CAN 接收回调检查总线、标准/扩展帧、ID、DLC和解码结果，更新缓存，不在中断中运行控制器。反馈与参考分别保留各自源时间戳，由薄层填入 age；不能每次读取时重写时间戳而掩盖旧数据。

官方 [`gimbal_feedback_update()`](https://github.com/RoboMaster/Development-Board-C-Examples/blob/59d12b1adcd321dbf1f9e9166aef5eb95ab657bf/19.gimbal_task/application/gimbal_task.c#L706-L739) 将 INS 姿态、编码器相对角和投影后的陀螺仪角速度组合使用。移植时要明确所选 yaw 坐标、正方向及输出轴单位：本库输入为 rad、rad/s，参考和反馈必须对应同一坐标；不能把世界系姿态角与未经变换的机体系速度直接拼接。若使用电机 RPM，则由设备适配层转换为 rad/s。

[`motor_ecd_to_angle_change()`](https://github.com/RoboMaster/Development-Board-C-Examples/blob/59d12b1adcd321dbf1f9e9166aef5eb95ab657bf/19.gimbal_task/application/gimbal_task.c#L752-L766) 计算的是相对零点的单圈折返角，不是连续多圈 yaw。本库不会替宿主做 unwrap：连续角累积、参考跨圈选择及丢帧后的重建由接入层完成，并保持机械限位与所选角度坐标一致。

## 3. 在设备层完成力矩到 CAN 的转换

官方 [`can_filter_init()`](https://github.com/RoboMaster/Development-Board-C-Examples/blob/59d12b1adcd321dbf1f9e9166aef5eb95ab657bf/19.gimbal_task/bsp/boards/bsp_can.c#L8-L30) 配置 CAN1/CAN2 过滤器、启动外设并启用 FIFO0 中断。沿用宿主 BSP 的所有权，在已有接收与发送路径接入本库 codec，避免重复定义 HAL 回调。

[`CAN_cmd_gimbal()`](https://github.com/RoboMaster/Development-Board-C-Examples/blob/59d12b1adcd321dbf1f9e9166aef5eb95ab657bf/19.gimbal_task/application/CAN_receive.c#L110-L125) 将四个 int16 槽位组成完整帧；[`CAN_GIMBAL_ALL_ID=0x1FF`](https://github.com/RoboMaster/Development-Board-C-Examples/blob/59d12b1adcd321dbf1f9e9166aef5eb95ab657bf/19.gimbal_task/application/CAN_receive.h#L32-L41)。其注释中的“电流”、`given_current` 名称和 ±30000 数值不是 SI 单位，也不等于本库的 GM6020 新电流模式。

本库的 GM6020 电流模式按原厂 v1.4 使用 `0x1FE/0x2FE`，不能直接调用旧 `CAN_cmd_gimbal()` 发送 N·m，也不能仅替换旧 ID 后沿用 ±30000 标定。由已确认驱动模式的设备层做 N·m→A→raw 转换，再调用 `gm6020_pack_current_group()`；具体门槛与映射见 [GM6020 接入说明](gm6020_integration.md)。组帧由总线/电机组唯一所有者汇总各轴当前命令，yaw 单轴任务不能用其余槽位清零的便利帧覆盖 pitch 或其他电机。

DM4310 使用 `dm_mit_encode_torque()` 及显式读取确认的映射参数；MIT 与一拖四协议属于不同通道，不能沿用 GM6020 帧。`submit_torque_nm` 只表示设备层接收了请求；CAN 成功入队不等于力矩已经执行，未经测量确认不要设置 `applied_torque_valid=true`。

## 本库的最小接入面

使用 [`examples/stm32/yaw_periodic.h`](../examples/stm32/yaw_periodic.h) 的四个宿主回调：`monotonic_us`、`read_snapshot`、`submit_torque_nm`、`request_disable`。宿主保有校准、使能、CAN 组帧与发送监控；库保有控制状态、输入检查和力矩约束。故障停机回调应撤销待发旧命令并执行宿主实际停机流程；任务里发送一个编码零值不能代替驱动停机。

次要对照：同作者整车固定提交 [`f67ef28a8a6aedb047f1e65837330034313e0be8` 的 `_GimbalTask`](https://github.com/LamdaDay/Infantry_RMUC_2026/blob/f67ef28a8a6aedb047f1e65837330034313e0be8/application/robot_task.h#L103-L123) 也采用云台任务与电机模块分层；其任务周期和整机参数证据已记于 [参数来源](parameter_sources.md)，不据此替换官方接入框架或用户硬件参数。
