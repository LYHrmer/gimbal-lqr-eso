# Pitch 资料核对与坐标约定

核对日期：2026-09-09。官方例程固定为 RoboMaster `Development-Board-C-Examples` 的 [`59d12b1adcd321dbf1f9e9166aef5eb95ab657bf`](https://github.com/RoboMaster/Development-Board-C-Examples/tree/59d12b1adcd321dbf1f9e9166aef5eb95ab657bf)。本页区分官方实际行为、力学推导和本库接入要求；没有把示例工程参数解释为用户电机带载实测值。

## 官方 C 板例程中已经存在的区分

| 核对项 | 固定版本中的实际实现 | 接入本库时的含义 |
|---|---|---|
| 两种 pitch 角度 | [`gimbal_feedback_update()`](https://github.com/RoboMaster/Development-Board-C-Examples/blob/59d12b1adcd321dbf1f9e9166aef5eb95ab657bf/19.gimbal_task/application/gimbal_task.c#L706-L724) 从 INS 读取 `absolute_angle`，由编码器和零点计算 `relative_angle`，速度取 INS 的 Y 轴陀螺仪。 | 保留机械相对角与重力姿态角两路信息。编码器零点不是天然的水平零点；INS 轴也须按实际安装校正。 |
| 电机方向 | 同一文件的 [`PITCH_TURN` 反馈分支](https://github.com/RoboMaster/Development-Board-C-Examples/blob/59d12b1adcd321dbf1f9e9166aef5eb95ab657bf/19.gimbal_task/application/gimbal_task.c#L715-L722) 与[最终命令分支](https://github.com/RoboMaster/Development-Board-C-Examples/blob/59d12b1adcd321dbf1f9e9166aef5eb95ab657bf/19.gimbal_task/application/gimbal_task.c#L346-L350) 都处理方向。 | 角度、角速度、参考和输出力矩必须采用一致的正方向；不能只反转 CAN 力矩而不核对反馈。 |
| 绝对角模式下的限位 | [`gimbal_absolute_angle_limit()`](https://github.com/RoboMaster/Development-Board-C-Examples/blob/59d12b1adcd321dbf1f9e9166aef5eb95ab657bf/19.gimbal_task/application/gimbal_task.c#L879-L911) 根据机械相对角、折返后的绝对角参考误差及新增量限制参考更新。 | 姿态闭环仍需要编码器机械边界。简单地给 INS pitch 设一个固定区间，不能代表底盘倾斜后的机械活动范围。 |
| 相对角模式下的限位 | [`gimbal_relative_angle_limit()`](https://github.com/RoboMaster/Development-Board-C-Examples/blob/59d12b1adcd321dbf1f9e9166aef5eb95ab657bf/19.gimbal_task/application/gimbal_task.c#L922-L939) 直接夹紧相对角参考。 | 可参考这个分层，由上层处理可达参考；控制器另行检查反馈越界。参考限位本身不是避免碰撞的证明。 |
| 校准后的机械范围 | [`cmd_cali_gimbal_hook()`](https://github.com/RoboMaster/Development-Board-C-Examples/blob/59d12b1adcd321dbf1f9e9166aef5eb95ab657bf/19.gimbal_task/application/gimbal_task.c#L449-L460) 将校准极值向内收缩后保存。 | 用户应提供实测机械边界和留量；本库仿真边界不能直接用于实际机构。 |
| 重力处理 | [`gimbal_motor_absolute_angle_control()` 与相对角控制](https://github.com/RoboMaster/Development-Board-C-Examples/blob/59d12b1adcd321dbf1f9e9166aef5eb95ab657bf/19.gimbal_task/application/gimbal_task.c#L996-L1029) 为角度、速度级联 PID；这两条路径未见显式重力前馈。 | 增加重力模型属于本库的独立实现，不能写成官方例程已有的功能。 |
| 零力及掉遥控 | [`GIMBAL_ZERO_FORCE` 选择 RAW 模式](https://github.com/RoboMaster/Development-Board-C-Examples/blob/59d12b1adcd321dbf1f9e9166aef5eb95ab657bf/19.gimbal_task/application/gimbal_behaviour.c#L303-L307)，[`gimbal_zero_force_control()`](https://github.com/RoboMaster/Development-Board-C-Examples/blob/59d12b1adcd321dbf1f9e9166aef5eb95ab657bf/19.gimbal_task/application/gimbal_behaviour.c#L553-L561) 输出零；[任务在 DBUS 错误时发送全零](https://github.com/RoboMaster/Development-Board-C-Examples/blob/59d12b1adcd321dbf1f9e9166aef5eb95ab657bf/19.gimbal_task/application/gimbal_task.c#L352-L361)。 | 软件撤销力矩后，不平衡 pitch 可能下落。宿主负责配重、支撑或制动与实际停机流程；故障锁定不等于机械保持。 |

本库沿用官方任务/BSP 的接入位置，未复制上述 GPL 代码。电机协议注意事项仍见 [官方框架接入说明](opensource_integration_notes.md)：GM6020 新电流模式不能直接使用旧 `CAN_cmd_gimbal()` 的 ID 与数值标度；yaw/pitch 共用组帧时，应由唯一的设备层所有者汇总命令。

## 重力模型的符号与适用条件

MIT 的[单摆课程材料](https://underactuated.mit.edu/pend.html)给出含重力、阻尼和输入力矩的单轴方程；其角度零点对应下垂位置，因此重力项为正弦。零点改成水平后不能照搬该正弦和符号。下面是针对本库平面模型的独立坐标推导。

设 `theta_g` 为相对水平的姿态角，抬头为正，正输出力矩使抬头加速。在 `theta_g=0` 时，合成质心相对转轴的前向坐标为 `x0`、向上坐标为 `z0`。固定基座、固定质心和固定水平转轴下，质心高度为：

```text
z(theta_g) = x0*sin(theta_g) + z0*cos(theta_g)
U(theta_g) = m*g*z(theta_g)
tau_hold(theta_g) = dU/dtheta_g
                 = A*cos(theta_g) + B*sin(theta_g)
A = m*g*x0; B = -m*g*z0
J*qdd = tau_motor - b*qd - tau_hold(theta_g) + disturbance
```

`tau_hold` 是抵消重力所需的电机力矩，植物方程中重力的实际作用为 `-tau_hold`。因此补偿时向电机请求中加入正的 `tau_hold`。`A`、`B` 单位均为 N·m，可描述质心不在枪管中心线上的情况，也可由明确角度坐标下的静态数据辨识。摩擦、线缆弹性和传动滞回不能因为被拟合进这两个系数就称为重力参数。

机械相对角 `q_rel` 与 `theta_g` 作用不同：前者限制机构活动范围，后者确定重力。固定水平台架经零点标定后，两者可仅差常数；底盘有俯仰时就不再相等。在共面、同正方向条件下可写 `theta_g = q_rel + base_pitch + offset`，更一般的安装须使用姿态变换。角速度输入也应是所选控制坐标的导数；不能把任意 Euler 角的导数与机体系单个陀螺仪分量视作普遍等价。

MIT 的[多体动力学课程](https://underactuated.mit.edu/multibody.html)以势能导数建立重力项，并区分位形导数与角速度坐标。这支持上述推导方法，也说明其边界：滚转、转轴倾斜、底盘加速与两轴耦合，通常需要姿态向量、惯性和耦合项。固定系数单轴模型的通过结果不能推广成任意整车姿态下的验证。

## 需要由 pitch 实验单独给出的证据

重力预先占用一部分持续力矩，使抬头和低头时的剩余力矩预算不同；质心偏置、安装角误差、机械边界与带载失能，也使 pitch 不能只靠重命名 yaw 接口适配。至少应分别检查静态多角度保持、双向跟踪、重力参数偏差、边界参考和力矩不足。仿真应保持基准与候选使用相同电机、负载、噪声、时延、限幅和参考，并区分“增加重力前馈的收益”与“相同重力前馈下的核心算法差异”。

硬件尚未搭建，本库的 pitch 惯量、质心力矩和边界只能标记为合成假设；不能称为 RoboMaster 的通用或用户实测参数。已有 yaw 改善百分比也不能自动用于 pitch。静态保持要求的力矩应按连续能力校验；额定或短时峰值不能替代长期保持与热验证。
