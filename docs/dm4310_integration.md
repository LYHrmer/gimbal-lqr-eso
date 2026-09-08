# DM4310 接入说明：先验证协议，再连接实物

本项目当前交付通用 C 编解码和离线仿真。`dm_mit.c` 不打开 CAN/串口，不发送报文，不修改电机参数，也不执行使能、失能或设置零点。硬件主控尚未确定，因此总线收发、实时调度和驱动状态机需要在选定主控后实现。这里的模块按公开协议独立编写，没有移植厂商或参考项目源码。

## 核对依据

- 达妙《DM-J4310-2EC V1.1 Gear Motor Instruction Manual》，文档 V1.0，发行日期 2023-11-16；2026-09-08 核对。MIT 模式见印刷页 7，反馈和控制帧见印刷页 9–10，通信失联保护见印刷页 14。PDF 查看器的零起始页索引分别为 6、8、9、13。[手册 PDF](https://files.seeedstudio.com/products/Damiao/DM-J4310-en.pdf)
- 达妙官方 `dmBots/motor-sdk` 的 `Python例程/u2can/DM_CAN.py`，固定提交 `0b2ede457bdbf0882e29ab9958ab8fda047b7f4a`，2026-09-08 核对；交叉确认型号映射范围、反馈半字节拆分、`TIMEOUT` 参数。部分参数旁的英文/中文注释并不准确，物理量以协议手册为准。[固定版本例程](https://github.com/dmBots/motor-sdk/blob/0b2ede457bdbf0882e29ab9958ab8fda047b7f4a/Python%E4%BE%8B%E7%A8%8B/u2can/DM_CAN.py)

这份较早手册针对特定版本电机。用户实物的型号后缀、驱动固件和配置尚未读取；本模块的桌面测试不构成实物兼容性验证。

## 三种力矩范围必须分开

| 名称 | 本项目中的含义 | 来源 |
| --- | --- | --- |
| `t_max_nm` / 驱动 `TMAX` | CAN 力矩整数与 Nm 之间的协议映射端点 | 读取实际驱动配置 |
| 额定、峰值力矩 | 电机在规定工况下的能力 | 对应实物型号的规格和热条件 |
| `torque_limit_nm` | 本次实验允许请求的绝对力矩上限 | 实验方案独立设定 |

`P_MAX`、`V_MAX`、`T_MAX` 全部显式传入 `dm_mit_config_t`，模块没有型号默认值。官方例程中 DM4310 的映射示例为 `(12.5 rad, 30 rad/s, 10 Nm)`，DM4310_48V 为 `(12.5 rad, 50 rad/s, 10 Nm)`；它们用于交叉核对，不应直接当成你的电机配置。[官方型号映射表](https://github.com/dmBots/motor-sdk/blob/0b2ede457bdbf0882e29ab9958ab8fda047b7f4a/Python%E4%BE%8B%E7%A8%8B/u2can/DM_CAN.py)

测试里的 `0.5 Nm` 是编解码测试限额，不是适用于所有机械装配的实验推荐值。即使限额远小于协议 `TMAX`，也必须保持正确的 `TMAX` 映射；通过修改本地 `TMAX` 来“降低输出”会造成主控与驱动对同一整数的解释不一致。

## 纯力矩输出与零点量化

MIT 下 `kp=kd=0` 时使用力矩前馈实现力矩指令；本模块始终将两个增益的整数字段写成零。[手册 MIT 模式](https://files.seeedstudio.com/products/Damiao/DM-J4310-en.pdf#page=7)

本实现采用最近值量化，正中间的两个候选码选择较大的码；如果量化后的名义力矩略微超过实验限额，则向内调整一个码。输入本身超过限额直接返回错误，控制器应在调用前按自身策略限幅。

12 位映射有 4096 个编码、4095 个间隔。由映射公式可得：

```text
tau_wire = (2 * raw - 4095) / 4095 * TMAX
一格量化步长 = 2 * TMAX / 4095
离零最近的两个值 = ±TMAX / 4095
```

因此无法用单个 MIT 指令编码数学上精确的零力矩。本实现对 `torque_nm=0` 输出 `raw=2048`，名义力矩为 `+TMAX/4095`；当 `TMAX=10 Nm`，约为 `+0.002442 Nm`。`command.torque_wire_nm` 返回这个名义值，便于日志和观察器记录。它不是力矩传感器读数，不能保证实际电磁或轴端力矩等于该值。若实验限额小于最小可表示力矩，配置检查直接拒绝。

最近值量化误差通常不超过半格；在应用限额边界向内调整后，误差可接近一格。测试覆盖这两种情况。位置、速度的“零”也采用较大的中心码，但它们在本模块零增益条件下不参与 MIT 阻抗项。

**8 个全零字节不是零力矩命令**：力矩字段 `raw=0` 对应映射负端点。编码失败时模块将 `valid=false`、`dlc=0`、`can_id=UINT32_MAX`、`torque_wire_nm=NAN`，保留原 payload；这些字节禁止发送，不能以补零报文或旧报文作为失败替代。

## 主控调用约定

以下片段只展示数据流，`verified_*` 和 `experiment_*` 均应来自上层配置；代码本身不包含发送或自动使能。

```c
dm_mit_config_t protocol = {
    .p_max_rad = verified_pmax,
    .v_max_rad_s = verified_vmax,
    .t_max_nm = verified_tmax,
    .torque_limit_nm = experiment_torque_limit,
    .motor_can_id = verified_motor_can_id,
    .master_can_id = verified_master_can_id,
};
dm_mit_command_t command = {0};
dm_mit_result_t result = dm_mit_encode_torque(&protocol, torque_nm, &command);

if (result == DM_MIT_OK && command.valid && command.dlc == 8) {
    /* 交给上层驱动状态机：确认允许运行后，以标准数据帧发送。 */
    /* command.torque_wire_nm 用作本次编码指令日志。 */
} else {
    /* 本次禁止发送；交由上层进入故障/失能流程。 */
}
```

上层需要把“成功编码”“硬件发送完成”和“电机已执行”分开记录。总线发送失败时，不能把 `torque_wire_nm` 当作已实际施加的输入；观察器应使用有明确时序约定的执行输入估计。缓冲区不能相互重叠，收发线程之间的同步由主控实现。编译器不能启用会假定 NaN/Inf 不存在的 `-ffast-math`。

## 反馈检查

调用 `dm_mit_decode_feedback` 时传入原始 CAN ID、扩展帧/远程帧标记、8 字节 payload 和实际长度。USB-CAN 适配器的外层串口封装需要先由对应驱动拆除。模块仅接受标准数据帧，检查期望 `MST_ID`，再检查 `data[0]` 低 4 位是否等于 `motor_can_id & 0x0f`；高 4 位单独作为 `status` 输出。[反馈定义及官方实现](https://github.com/dmBots/motor-sdk/blob/0b2ede457bdbf0882e29ab9958ab8fda047b7f4a/Python%E4%BE%8B%E7%A8%8B/u2can/DM_CAN.py)

手册某处将反馈 ID 描述为 CAN ID 低 8 位，但同一字节还包含状态高 4 位；本实现按字节布局和官方例程只提取低 4 位。因此两个电机若共用 `MST_ID` 且 CAN ID 低 4 位相同，本解析器无法区分它们，配置总线时应避免这种组合。参数读写回包也不应送入这条反馈通道；上层必须按当前事务类型路由。

解析器输出原始位置/速度/力矩整数、换算物理量、MOS 温度、转子/绕组温度字节以及状态原码。未知状态保留原值；故障帧和超过实验限额的反馈力矩仍可成功解析，以便上层观察真实异常。`feedback.valid=true` 仅表示格式和身份检查通过，不表示电机正常、更不表示可以使能。

上层应针对实物固件建立允许状态表；手册列出的 `8..E` 涵盖过压、欠压、过流、驱动过温、线圈过温、通信丢失和过载。[状态说明](https://files.seeedstudio.com/products/Damiao/DM-J4310-en.pdf#page=6) 不要采用“只要不是已知故障就允许驱动”的规则。角度跨界展开、方向和零位校准、反馈时间戳、重复/过期帧处理、温度阈值判断均由上层完成。

## 硬件接入时补齐的行为

1. 在上电但未使能时记录实物型号、固件、模式、总线速率、ID 和三个映射范围；逐项核对后才能使用该配置。
2. 建立显式使能状态机，并实现失能报文、失能确认和超时兜底。模块不把发送零力矩当作停止使能。
3. 主控按单调时钟检查反馈新鲜度和控制周期；反馈丢失、计算错误或发送失败进入上层停止流程。
4. 配置并读回驱动侧 `TIMEOUT`，按对应固件确认单位和触发行为；用断开命令流的测试确认失联退出使能，不能只依赖主控进程继续存活。驱动的失联保护行为见对应手册。[通信保护](https://files.seeedstudio.com/products/Damiao/DM-J4310-en.pdf#page=14)
5. 接入限位、急停和电源处理后再安排低能量实物测试；仿真参数和测试限额需在辨识之后调整。

## 离线验证

从仓库根目录编译，不依赖任何硬件 SDK：

```sh
cc -std=c11 -Wall -Wextra -Werror -Wconversion -Wshadow -pedantic -O2 \
  -Iinclude src/dm_mit.c tests/test_dm_mit.c -lm -o /tmp/test_dm_mit
/tmp/test_dm_mit
```

测试验证固定十六进制报文、正负端点、20001 个全范围输入的量化误差与单调性、应用限额内的输出、最小可编码力矩、非有限值、非法配置、错误 ID/帧类型/长度，以及全部 16 种状态原码的保留。测试使用 `assert`，编译测试时不要定义 `NDEBUG`。这些检查验证软件协议行为，尚未验证实际驱动的波形、参数或响应。
