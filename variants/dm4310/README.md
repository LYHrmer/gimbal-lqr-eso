# DM4310 适配版本

采用 MIT 模式的纯力矩命令，外部位置/速度环由公共 LQR–ESO 内核计算。编译的静态库是 `build/libgimbal_dm4310.a`，整车源码移植需要以下四个文件：

- `include/yaw_controller.h`、`src/yaw_controller.c`
- `include/dm_mit.h`、`src/dm_mit.c`

本目录的 `simulation_config.h` 由 [仿真 JSON](../../profiles/dm4310_24v.json) 生成。`dm4310_simulation_config()` 只给仿真参数示例。上机必须用实际负载辨识值和允许限值替换；原型项目不会自动打开设备。

接入顺序：

1. 完成 DM4310 CAN 驱动、ID/模式/反馈范围确认，以及上层使能/失能状态管理。
2. 初始化公共控制器；把连续展开的输出轴角度、速度、反馈年龄和连续参考传入周期函数。
3. 仅在上层允许驱动且周期状态为 `YAW_OK` 或 `YAW_WARMUP` 时，调用 `dm_mit_encode_torque()`。
4. 编码返回成功且 `frame.valid=true` 后，才将该标准数据帧提交 CAN。失败帧 `DLC=0`，不得把旧 payload 再发出去。
5. 故障走上层失能/停止流程；确认原因消除后显式复位，不能凭下一帧反馈正常自动恢复。

协议映射 `TMAX` 与实验力矩上限分别配置；纯力矩通道令电机内的 `Kp=Kd=0`。详细语义及量化零偏见 [DM4310 接入说明](../../docs/dm4310_integration.md)。

本版参考电机是手册中的 24 V DM-J4310-2EC V1.1；48 V 或其他固件/版本需要核对，不能直接共用编码范围或电机能力。仿真只约束已注明的规格/近似模型，没有温升、真实电流环或齿隙验证。
