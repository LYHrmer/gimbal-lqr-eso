# STM32 接入：官方任务框架与纯 C 控制内核

部署对象是 STM32 上运行的 C 算法。Python 仅负责离线设计、仿真和验证，不进入电机实时控制链。本仓库提供 [gimbal_periodic.c/.h](../examples/stm32/gimbal_periodic.c) 的 Yaw/Pitch 薄层示例，由实际工程提供回调；原 [yaw_periodic.c/.h](../examples/stm32/yaw_periodic.c) 接口继续兼容。公共库不绑定某一整车的 HAL 初始化、链接脚本和 CAN 引脚，不提供可直接烧录的整车固件。

## 接在官方例程的什么位置

主参考为 RoboMaster 官方 [Development-Board-C-Examples，固定提交 `59d12b1adcd321dbf1f9e9166aef5eb95ab657bf`](https://github.com/RoboMaster/Development-Board-C-Examples/tree/59d12b1adcd321dbf1f9e9166aef5eb95ab657bf/19.gimbal_task)。它已有 HAL、FreeRTOS、CAN 接收和云台任务分层。本示例独立实现调用适配，未复制官方 GPL 源码；把现有板级工程作为外部集成对象。

官方 [`gimbal_task()`](https://github.com/RoboMaster/Development-Board-C-Examples/blob/59d12b1adcd321dbf1f9e9166aef5eb95ab657bf/19.gimbal_task/application/gimbal_task.c#L315) 中，反馈更新、参考处理、控制计算、CAN 提交依次执行。保留板级外设、反馈来源、上层参考和任务管理，在各轴控制计算位置调用 `stm32_gimbal_periodic_step()`；该轴原有控制计算与发送路径应随替换同步停用，避免同一电机收到两套控制命令。新算法仍在这个任务中运行，不另建一套调度框架。

手瞄和自瞄都可使用新核心。摇杆/鼠标与视觉继续由原应用生成参考，接入层统一连续 rad、rad/s、rad/s²，并明确模式切换的参考同步与状态重置。保留原控制器作回退时，应使用明确的选择开关，不能让两份控制运算同时更新同一电机。多于两轴时先完成宿主关节参考分配，见 [多轴接入](multiaxis_integration.md)。

该版本的 [`GIMBAL_CONTROL_TIME=1`](https://github.com/RoboMaster/Development-Board-C-Examples/blob/59d12b1adcd321dbf1f9e9166aef5eb95ab657bf/19.gimbal_task/application/gimbal_task.h#L114)，[`configTICK_RATE_HZ=1000`](https://github.com/RoboMaster/Development-Board-C-Examples/blob/59d12b1adcd321dbf1f9e9166aef5eb95ab657bf/19.gimbal_task/Inc/FreeRTOSConfig.h#L61)，循环尾使用 `vTaskDelay()`。这是名义 1 tick，计算时间和调度会影响实际间隔。接入时以 1 kHz 为目标，用实际单调计时得到 `dt`；不能固定填写 `0.001f`。任务周期安排可由工程自己的定时器/FreeRTOS 绝对周期机制负责，超期仍交给控制器检查。

## 薄层回调与状态

每轴静态分配一个 `Stm32GimbalPeriodic`，在控制任务中调用 `stm32_gimbal_periodic_init()`，传入经过实机辨识与限值核对的 `GimbalConfig` 和 `Stm32GimbalHooks`。Yaw 配置 `pitch_enabled=false` 且重力系数为零；Pitch 提供独立重力系数和关节行程。初始化后等待下一次真实周期再调用 `step()`，不要立即人为补一帧名义时间。`init/step/acknowledge` 必须串行调用。

只使用历史 Yaw 接口的工程可继续分配 `Stm32YawPeriodic`、传入 `YawConfig/Stm32YawHooks`，调用对应 `stm32_yaw_periodic_*` 函数；两套薄层都调用同一基础 C 核。

| 回调 | 由官方任务 / BSP 接入层实现的责任 |
| --- | --- |
| `monotonic_us` | 提供同一时钟域、微秒单位、不倒退的 `uint64_t` 时间；处理底层计数器溢出与一致读取。 |
| `read_snapshot` | 一次取得相互一致的反馈、参考、各自源时间戳和 `drive_ready`；Pitch 另含关节角/速度、重力角与姿态源戳。角度和速度已经转换为连续输出轴 rad / rad·s⁻¹。 |
| `submit_torque_nm` | 非阻塞接收力矩与源时间，执行本轴协议校验或更新组帧所有者的槽位；复制数据，不能保留临时栈对象指针。返回成功只表示接收，不表示电机已经执行。 |
| `request_disable` | 立即禁止后续普通力矩提交、作废排队命令，并通知上层开始真实停机/失能流程；电机驱动看门狗和 CAN 故障恢复仍由工程实现。 |

`drive_ready` 包含上层明确授权、模式确认和当前驱动健康状态，不等于“收到一帧可解码数据”。通常只在允许控制的阶段运行本周期薄层；运行过程中撤销授权会锁存停止。初始化和复位均不会使能设备或切换模式。

`STM32_GIMBAL_OK` / `STM32_GIMBAL_WARMUP` 才表示正常周期；其他返回值均停止普通提交并锁存。控制器故障细节保留在 `last_output.control.status`；回调/计时故障要看 `Stm32GimbalResult`，不能仅凭旧的 `YAW_OK` 诊断值继续发送。历史 Yaw 薄层分别使用 `STM32_YAW_*` 和 `last_output.status`。首次有效周期和显式复位后的首次周期输出软件零。这个零仍需经过电机协议，DM MIT 的量化零偏不能视为物理停机；有重力负载的 Pitch 也可能下坠。

`request_disable` 是停止请求，可靠停机必须由上层状态机完成并确认；异步 CAN 提交后再发生的 bus-off、发送超时也要进入该状态机。只有确认故障原因消除、原排队命令作废且重启条件满足后，任务才显式调用 `stm32_gimbal_periodic_acknowledge()`（历史接口为 `stm32_yaw_periodic_acknowledge()`）；不能按“下一帧正常”自动复位。Pitch 还需由整车落实支撑、制动或其他物理保持措施。

## CAN 接收、时间戳和角度

官方 [`HAL_CAN_RxFifo0MsgPendingCallback()`](https://github.com/RoboMaster/Development-Board-C-Examples/blob/59d12b1adcd321dbf1f9e9166aef5eb95ab657bf/19.gimbal_task/application/CAN_receive.c#L60) 写入共享电机结构，getter 返回数组元素指针；该指针本身不保证一致快照。CAN ISR 只做收帧、必要的定长检查/发布和接收时间标记，把 ESO、`expm1f`、`tanhf` 和完整控制计算放在任务上下文。

`read_snapshot` 应用很短的临界区或工程已经正确实现的双缓冲机制，成组复制数值、有效位和时间戳。临界区须覆盖实际写入者的中断优先级；参考写入任务也要完整发布，不能只给读端加锁，而任由低优先级写端在写一半时被控制任务抢占。`volatile` 不能代替多字段一致性。复制后退出临界区再做数学运算。32 位 MCU 读取软件扩展的 64 位时间也需要一致性处理，不能读到撕裂的高低字。

时间戳必须记录在反馈到达/参考生成时；任务读取时不能刷新旧数据的时间戳。示例在取得快照后读取当前时间，分别计算反馈、参考和 Pitch pose 的 age，并拒绝未来时间戳、时钟不前进和回退。`pose_source_us` 使用构造姿态涉及的最老源时刻，包括关节编码器、重力倾角和必要的底座姿态；新电机帧不能替过期姿态刷新年龄。使用同一单调时钟域；板级层负责将 32 位计数器扩展为 64 位，转换表达式也必须在 64 位中计算，不能只把返回类型写为 uint64。本例没有把瞬时电流或力矩反馈当作上个周期平均执行力矩，始终设置 `applied_torque_valid=false`。

GM6020 的反馈是单圈 `0..8191` 计数，接入层应按输出轴方向、零位和跨圈计数形成连续角度。以最短跨圈差判断时，必须保证相邻有效反馈之间不会真实跨越半圈；丢帧后无法消除多圈歧义就应使反馈失效。DM4310 按实际驱动的反馈绕回规则处理，不能把 `[-PMAX,+PMAX]` 的协议数值跨度自动当成编码器一圈。电机角度、IMU 角度与参考必须统一到同一个轴和 rad 单位；有外部传动时还须换算输出轴参数。

世界姿态跟踪量、机械关节角和重力倾角分别标定，不能在倾斜底座上把电机编码器零位直接当作水平。Pitch 便利适配假定单轴平面关系；复杂重力曲线或串联轴耦合由宿主计算后调用 `yaw_controller_step_with_load()`，详见 [Pitch 接入](pitch_integration.md)。Yaw 方向目标可选用 `gimbal_angle_near()` 提升到最近连续圈；明确多圈运动参考不要经过该转换。三实例支持的边界见 [多轴说明](multiaxis_integration.md)。

## 两种电机的提交路径

**DM4310 MIT**：用实际设备读出的 `PMAX/VMAX/TMAX` 与独立应用力矩限值初始化 `dm_mit_config_t`。上层确认 MIT 模式后，`submit_torque_nm` 调用 `dm_mit_encode_torque()`；该模块固定 `Kp=Kd=0`，输入为 N·m。只有返回 `DM_MIT_OK`、`valid=true`、`dlc=8` 才能提交标准 CAN 数据帧，提交失败返回 false。编码失败不发送任何遗留 payload，不自动切换固件或模式。详见 [DM4310 协议](dm4310_integration.md)。

**GM6020 电流模式**：固件和 Assistant 电流环开关均已确认后，用 `gm6020_torque_to_word()` 产生本轴 current word。一个总线任务拥有该组的全部 4 个槽位，按源时间收集各轴的新鲜命令，一次调用 `gm6020_pack_current_group()`，再提交整组帧；ID `0x2FE` 的第 4 槽必须显式为 0。不能为了更新 yaw 就把同组 pitch/其他轴的槽位清零；缺少或过期槽位需按该组的停机策略处理，不能复用无期限的旧命令。

官方老版 [`CAN_cmd_gimbal()`](https://github.com/RoboMaster/Development-Board-C-Examples/blob/59d12b1adcd321dbf1f9e9166aef5eb95ab657bf/19.gimbal_task/application/CAN_receive.c#L110) 使用 `0x1FF`，变量名 `given_current` 不能证明实际电流模式。这里采用 v1.4 电流协议的 `0x1FE/0x2FE`，必须替换对应映射和组调度；不能直接把 `τ/Kt` 塞入旧电压帧。反馈 `current_raw` 不擅自换算安培。详见 [GM6020 协议与版本要求](gm6020_integration.md)。

## C11、FPU 和实际耗时

将 `src/yaw_controller.c`、`src/gimbal_controller.c`、所选协议 `.c` 和 `examples/stm32/gimbal_periodic.c` 加入工程，包含 `include/` 与 `examples/stm32/`。使用最近圈助手时再加入 `src/gimbal_coordinates.c`；历史 Yaw 薄层可继续只带基础核心和 `yaw_periodic.c`。算法与薄层无动态分配、无底层 I/O；应用回调的实现仍需满足任务的时间预算。使用 C11，链接与工程 ABI 一致的数学库 `libm`，确保 `expm1f/tanhf/sinf/cosf` 等符号真正解析。无需在 MCU 中运行 LQR 求解器。

M4/M7 的具体 FPU 能力取决于芯片；启动文件、FreeRTOS 端口和编译选项要匹配。`-mcpu`、`-mfpu` 和 `-mfloat-abi` 应在算法、协议、应用及所链接库中一致，不能混链 hard-float 与 soft-float ABI。单精度 FPU 不会自动让所有 `double` 运算成为硬件指令，本协议转换和部分保护运算含 `double`，应纳入测时。参考 [ST AN4044，Rev 2](https://www.st.com/resource/en/application_note/an4044-floating-point-unit-demonstration-on-stm32-microcontrollers-stmicroelectronics.pdf) 与 [GCC ARM 选项](https://gcc.gnu.org/onlinedocs/gcc/ARM-Options.html)。

禁止 `-ffast-math`、`-Ofast` 以及允许假定所有输入有限的优化。`isfinite()` 与 NaN/Inf 故障检测是控制契约的一部分，源码在 `__FAST_MATH__` 下会拒绝编译。普通 `-O2` 可作为验证起点；是否满足 1 ms 必须在最终板卡、数学库、优化级别与任务负载下测量。

在器件确实提供并允许使用 DWT cycle counter 时，按对应芯片/CMSIS 文档完成初始化后，围住实际调用测量：

```c
/* Only where DWT CYCCNT is supported, enabled and advancing. */
uint32_t start_cycles = DWT->CYCCNT;
Stm32GimbalResult result = stm32_gimbal_periodic_step(&axis_app);
uint32_t elapsed_cycles = (uint32_t)(DWT->CYCCNT - start_cycles);
/* Use the actual core clock; elapsed interval must be shorter than one wrap. */
```

DWT/CYCCNT 不是所有 STM32 的通用能力，不能向不支持的器件直接写这段寄存器代码；[Arm CMSIS 的 DWT 定义](https://github.com/ARM-software/CMSIS_5/blob/5.9.0/CMSIS/Core/Include/core_cm4.h) 也提供 cycle-counter 能力位。没有可用 CYCCNT 时使用实际配置的硬件定时器。记录冷/热缓存、正常/限幅/故障路径和实际中断负载下的最大观测耗时；同时测完整周期与队列延迟。DWT 耗时不能代替控制 dt 或反馈 age，平均耗时也不能代表最坏调度延迟。当前没有板上耗时实测值。

## ARM 目标编译检查

`tools/check_arm_build.py` 使用 GNU Arm C 编译器，把控制核心、两种协议、周期薄层和参数头文件探针编译为 Cortex-M4F 的 ELF32 ARM 目标文件，并生成两份静态库。这里选择 `cortex-m4 / fpv4-sp-d16 / hard-float` 作为明确的编译检查目标；具体芯片不匹配时需要修改选项。

```bash
# Debian/Ubuntu 示例；只需要编译器、binutils 与 newlib 头文件。
sudo apt-get install --no-install-recommends gcc-arm-none-eabi libnewlib-dev
python3 tools/check_arm_build.py --newlib-include /usr/include/newlib
```

编译检查同时使用严格警告和 `-fno-fast-math`。它检查 C 接口和目标代码生成，尚未把 startup、链接脚本、HAL、RTOS 与数学库链接为整板固件，也不测 MCU 耗时。最终链接仍需使用工程对应的 C/数学库。GitHub CI 单独执行这一检查，与宿主机测试和仿真并行。

本次 GNU Arm 13.2.1 检查已通过，生成 8 个目标文件（基础核心、Pitch/坐标适配、两协议、两周期薄层和参数头文件探针）及两份静态库；实际编译器、选项与源文件哈希见 [ARM 编译结果](../results/arm_compile.json)。使用自带 newlib 的完整 GNU Arm 工具链时，可省略额外头文件参数。

## 本次能验证到的边界

`variants/*/simulation_config.h` 与 `pitch_simulation_config.h` 明确是 **simulation_only**。示例不自动引用它们；测试文件引用仿真配置仅作为宿主机 fixture，不能把测试参数视为上机默认值。

宿主机可验证薄层逻辑：真实周期、slew、超周期、NaN、过期/未来时间戳、独立 Pitch 姿态年龄、快照失败、驱动未就绪、提交失败、停止锁存和显式复位后的 warmup。7 项 CTest 覆盖两个薄层以及三实例/近圈助手。手工复跑：

```bash
cc -std=c11 -Wall -Wextra -Wpedantic -Wconversion -Wshadow -Werror \
  -I include -I examples/stm32 tests/test_stm32_gimbal_periodic.c \
  examples/stm32/gimbal_periodic.c src/yaw_controller.c src/gimbal_controller.c -lm \
  -o /tmp/test_stm32_gimbal_periodic
/tmp/test_stm32_gimbal_periodic
```

这些测试和 ARM 交叉编译可检查软件行为及目标代码生成，不能替代所选 STM32 的启动、CAN 时序、电机方向、执行失能和实时性验收。
