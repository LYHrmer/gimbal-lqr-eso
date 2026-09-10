# 实机前的软件修复与验证

[项目首页](../README.md) · [实机实验步骤](experiment_plan.md) · [STM32 接入](stm32_integration.md)

本轮以提交 `2901a83` 为修改前基线，目标是修复可复现的软件问题。控制增益、仿真模型、实验种子和性能门槛保持原值；软件可靠性改善与跟踪精度提升分别报告。

## 发现的问题与收益

| 项目 | 修改前可复现行为 | 修改后行为及验证 |
| --- | --- | --- |
| 编译选项绕过有限数检查 | GCC 11.4 的 `-O2 -ffinite-math-only` 不触发现有 `__FAST_MATH__` 保护；`torque_limit_nm=NAN` 被 `yaw_config_valid()` 接受 | 同时检查 `__FINITE_MATH_ONLY__`；8 个 C 文件分别验证正常编译及拒绝 `-ffinite-math-only`、`-ffast-math`、`-Ofast` |
| RLS 激励统计被大样本长期污染 | seed=0 的第 250 个窗口替换为有限 `[1e12,1e12]`；异常退出 100 窗口后，增减累计 Gram 仍有消减残留，后续正常数据长期被判为激励不足 | 每步由当前窗口重建 Gram，外积使用 float64 并拒绝非有限贡献；异常退出后与干净数据的激励判断一致，恢复参数更新 |
| Pitch 内缩边界与 float 接口不一致 | 使用 GM6020 Pitch 原仿真配置，在数学内缩边界的最内侧可表示端点给出外向速度，旧版仍返回 warmup | 公共边界函数与控制检查使用相同、向内舍入的 float 端点；拒绝端点外向速度，保留端点静止和向内运动 |

编译修复使不受支持的编译选项在构建阶段暴露；RLS 修复消除异常样本离开窗口后的数值残留；Pitch 修复统一宿主参考生成与控制器的端点定义。**这些是有失败复现支持的工程改进，不是新的 RMSE 提升结论。**

RLS 的新测试还覆盖 float32/float64 大值和有限输入的外积溢出，检查拒绝时 C 参数及 P 不变。重建窗口增加的是主机实验计算量，尚未移植到 STM32；不把这段 Python 前端称为板端在线辨识。Pitch 端点只向安全区内舍入，不放宽机械边界或指定留量。

具体恢复对照：`[1e12,1e12]` 异常退出后的 2650 个正常窗口，旧版接受 148 次，修复后接受 2650 次；`[1e200,1e200]` 的有限输入外积溢出反例由 0 次恢复到 2650 次。正常数据无需重置估计器。原 44 条实验流的非来源报告字段、固定协议及七项门槛保持一致。

在 GCC 11.4、普通 `-O2` 下，仅增加编译保护的 7 个模块（单轴核心、坐标助手、两协议、两薄层和 RLS）与基线生成的目标文件逐字节相同。Pitch 适配器另外改变了限位判断，因此单独验证；不能把这项目标文件对照解释为整个库没有行为变化。

本轮完整重跑两种电机的 Pitch 仿真，包括开发组、留出组、载荷/噪声/延迟变化及失败边界工况，全部 profile 数据与此前发布值一致。改动没有使本组仿真退化，也没有带来新的跟踪精度提升。边界函数另以精确有理数核查 10,000 组有限 float 几何，端点及接受/拒绝判断无差异；该独立回归保存在 `tests/test_pitch_simulation.py`。

原始核对记录：[编译绕过复现](../results/prehardware/finite_guard_before.json)、[正常目标文件对照](../results/prehardware/code_generation.json)、[RLS 异常恢复前后对照](../results/prehardware/rls_recovery.json)、[Pitch 端点前后对照](../results/prehardware/pitch_boundaries.json)、[完整 Pitch 仿真一致性](../results/prehardware/pitch_simulation.json)。这些是本次构建归档，复跑时应以自己的源码、编译器和生成结果为准。

[本轮软件检查记录](../results/prehardware/verification.json) 绑定相关源码和测试的 SHA-256：默认 7 项 CTest、含可选 RLS 的 8 项 ASan/UBSan 检查、47 项主工程与 11 项 RLS Python 测试通过；Cortex-M4F 的原 8 个目标及独立 RLS 目标编译通过。板上执行时间和硬件效果尚未验证。

## 故障恢复覆盖

[STM32 薄层测试](../tests/test_stm32_gimbal_periodic.c) 从已经输出非零力矩的状态注入 8 类故障：三种来源的未来时间戳、时钟停顿、时钟倒退、快照失败、运行授权撤销和提交失败。每例检查停止提交、仅请求一次停机、输入恢复仍锁存，以及显式 ACK 后首帧零力矩。另覆盖配置自别名初始化和四个必需回调缺失。

这部分旧运行逻辑本来就正确，本轮只加强回归覆盖。测试中的软件停机请求不代表真实 CAN 已失能或 Pitch 已保持。

## 复跑

在仓库根目录执行；Python 环境按 [快速开始](quickstart.md) 安装：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DGIMBAL_BUILD_RLS_EXPERIMENT=ON
cmake --build build --parallel
ctest --test-dir build --output-on-failure
.venv/bin/python -m unittest discover -s tests -p 'test_*.py'
.venv/bin/python -m unittest discover -s experimental/online_rls/tests -p 'test_*.py'
.venv/bin/python experimental/online_rls/run_experiments.py --output build/prehardware-rls
.venv/bin/python tools/check_online_rls_results.py --report build/prehardware-rls/report.json
.venv/bin/python sim/run_pitch_profiles.py --phase full --output build/prehardware-pitch
```

单独复跑编译选项回归不需要 NumPy/SciPy：`python3 -m unittest discover -s tests -p test_compiler_contract.py`。核心数值测试需要先构建共享 C 库；完整 Pitch 仿真还需要按快速开始获取固定版本的上游对照源码。

## 仍需实机回答的问题

实际力矩标度、命令与反馈时序、各轴惯量/摩擦/重力参数、最终 STM32 耗时，以及 CAN 故障后的执行停机，都需要所选硬件的数据。完整采集前端、候选参数采用流程和大 Yaw—小 Yaw—Pitch 耦合闭环也尚未完成。

既有性能结论继续保留：DM4310 已声明工况有仿真改善，GM6020 Yaw 需区分积分配置收益与同参数内核对照，GM6020 Pitch 开发组仍未通过，完整 Pitch 验收仍为 `false`。不通过修改门槛或删除失败工况来声称优化成功。
