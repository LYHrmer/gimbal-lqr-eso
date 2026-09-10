# 离散 LQR 调参与 C 内核仿真

仿真直接通过 `ctypes` 调用 `build/libyaw_controller.so`，Python 只负责生成参考轨迹、演化被控对象、注入测量误差和绘图。没有单独的 Python 控制器。绑定加载前会用本机 C 编译器核验所有结构大小与字段偏移，包括 `bool` 和枚举所在结构的布局；不一致时立即退出。

**本仓库正弦基准不等于论坛实验轨迹。** 公开的 [3Hz20deg 原图](https://hz-rm-bbs-web-prod.oss-cn-hangzhou.aliyuncs.com/b4ab732bb5fc4c32b7b9281a2794799d1783157227130/3Hz20deg.jpg) 中，`/plan_yaw` 显示的是慢爬升后快速换向的非正弦周期波形。原始 CSV 缺失，帖子“3 Hz 20 deg”不能直接解释成云台 ±20°、3 Hz 正弦。本项目选择导数一致的正弦，是为了构造可核验的软件比较。本文 5 Hz / 13.83 N.m 的可达性计算只针对本项目的假设，不是对作者真实实验的反证。

先按根目录 README 构建共享库，再运行：

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
.venv/bin/python tools/tune_lqr.py
.venv/bin/python sim/run_benchmarks.py
```

需要 C 编译器、Python、NumPy、SciPy 和 Matplotlib；无需电机、CAN 设备或 GPU。默认共享库可用 `--library /path/to/libyaw_controller.so` 替换。若本机已有一致的依赖，也可以直接用对应 Python 执行；本次本机验证使用 `python3 -s`，避免用户目录 NumPy 2 与系统旧版 SciPy/Matplotlib 的二进制混用。准确依赖版本写入结果 JSON。

`tools/tune_lqr.py` 使用模型

```text
x = [theta, omega]
xdot = [[0, 1], [0, -B/J]] x + [[0], [1/J]] torque
u_feedback = Kd (reference_state - measured_state)
```

SciPy `cont2discrete(method="zoh")` 给出精确零阶保持模型，`solve_discrete_are` 求离散 Riccati 方程，再算 `Kd=(R+Bd.T P Bd)^-1 Bd.T P Ad`。所有输入都要求有限且为正数；结果记录离散闭环极点、谱半径和 Riccati 残差。默认值 `J=.039, B=.30, dt=.001, Q=diag(1600,1), R=.2` 得到 `Kd≈[85.876964, 3.078769]`、谱半径约 `0.956448`。这是理想未限幅模型的性质，不能推导为带噪声、延迟、饱和后的硬件稳定性保证。

可以独立重新设计：

```bash
.venv/bin/python tools/tune_lqr.py --inertia 0.02 --damping 0.1 --dt 0.001 \
  --q-position 1600 --q-velocity 1 --r-torque 0.2 --output results/custom_lqr.json
```

自定义调参文件不会自动覆盖基准场景。基准脚本在 `make_config()` 中明确指定全部参数，并保存实际经 `float` 转换的 C 配置。如果要模拟自己的负载，需要同步修改 `design_lqr()` 调用、控制器名义参数和 `Plant` 参数；保留名义模型与真实模型之间的差异，才能评价鲁棒性。

基准共六种场景：名义模型、综合扰动模型各运行 1 Hz ±5°、3 Hz ±20°、5 Hz ±20°。每种场景比较同一 C 内核的 `eso_gain=0` 与 `0.8`，其余参数不变，积分关闭，随机种子默认为 `4310`。完整假设见 [assumptions.md](../assumptions.md)。不要把 ±7 N.m 的挑战仿真配置复制为上机默认。

在每个 `t_k`，脚本使用同时刻参考值与量化/加噪后的真实状态调用控制器，`valid=true`、`age_s=0`。首步只初始化观察器并输出零；后续输出经过配置的一周期命令延迟和一阶执行器动态，再用 RK4 演化到 `t_(k+1)`。CSV 的位置、速度和执行器力矩均为 `t_k` 时刻值；`command_nm` 是在此刻新发出的命令，因此执行器力矩与新命令不应强行移到同一执行区间比较。观察器只看到前一最终命令，`applied_torque_valid=false`。

量化反馈采用假设已校零的零中心均匀间隔模型，不是逐 bit 的 CAN 反馈回放。DM MIT 原始映射的半间隔零点偏移没有在此基准中模拟；实际 C 命令编解码的双电机实验另见 [robustness.md](robustness.md)。

正弦乘以 2 s 的五次启动包络 `e(s)=10s³−15s⁴+6s⁵`，参考速度、加速度使用乘积法则求出全部包络导数。2–8 s 为指标窗口，RMSE 等误差相对真实位置计算，避免把传感器噪声当成被控对象位移。

输出包括：

- `results/lqr_design.json`：离散模型、权重、增益、极点和设计范围。
- `results/metrics.json`：全部场景、C 配置、指标、种子、依赖版本、ABI 大小、C 源码/头文件和共享库 SHA256。
- 每个场景一张 PNG 和一张可编辑 SVG，上面板跟踪、下面板真实误差，明确标注 synthetic。
- `results/stressed_3hz_20deg.csv.gz`：综合扰动 3 Hz 场景的两组完整对齐轨迹，gzip 时间戳固定。加 `--all-csv` 保存全部轨迹。
- `results/README.md`：自动生成的结果对照表。

```bash
.venv/bin/python sim/run_benchmarks.py --output results/recheck --seed 4310 --all-csv
```

可以增加 RK4 子步数检查积分精度：`--substeps 10 --output results/refined`；该操作保持 C 控制周期 1 ms 不变。共享库应在 C 核心有更改后重新构建，结果文件中的源码哈希用于追溯，不替代构建系统的依赖更新。

完成共享库构建后，可复跑快速检查：

```bash
.venv/bin/python -m unittest discover -s tests -p 'test_*.py'
```

`tests/test_simulation.py` 使用标准 `unittest`，独立解析有阻尼模型与零阻尼双积分器的 ZOH 解并检查离散闭环极点，拒绝非有限及越界调参输入，以有限差分检查启动包络内外的参考导数，并通过真实 C 库检查固定种子复现、输出限幅/变化率、在线降额优先、反馈超时故障锁存和显式复位。复现性测试只运行 0.3 s 的局部场景，不重跑全部基准，也不修改已发布结果。

原版与新版的直接对比由另一入口执行，结果单独放在 `results/upstream_comparison/`：

```bash
.venv/bin/python tools/fetch_upstream.py
.venv/bin/python sim/compare_upstream.py
```

下载器固定原版提交 `665c5b4ab1067d6cb63122c120822f27502953e5` 并验证源码 SHA256；仅缓存到被 Git 忽略的 `build/upstream/`。适配器编译未修改的原版 C 文件，不用 Python 重写原控制器。已有对应版本的本地仓库时，可离线执行 `sim/compare_upstream.py --upstream-source /path/to/YAW_Auto_Controller`；版本内容不匹配会拒绝运行。

两版本使用相同离散 LQR 增益、ESO 带宽 80 rad/s / 补偿系数 0.8、名义 J/B、硬力矩限幅、命令变化率、被控对象、轨迹和种子；积分及旧版额外 bias 环关闭。这是统一配置下的直接软件比较，不是各自最优调参结果，也不是作者原始硬件配置。旧版没有的故障锁存、数据年龄检查和补偿变化率限制不会由适配器补入；旧版无状态码，因此其 fault/warmup 指标为 `null`。

固定种子结果中，综合扰动 3 Hz / ±20° 正弦的 RMSE 从原版 `0.48102°` 降到新版 `0.42247°`，降低约 **12.17%**；综合扰动 1 Hz 仅降低约 **1.06%**。5 Hz 饱和工况基本没有改善，名义 5 Hz 还略有退化，全部结果均保留。这里的 12.17% 才是本次直接对原版的比较；前面新版内部 `eso_gain=0` 与 `0.8` 的消融结果不能被表述为相对原版的提升。JSON 记录原版源码/桥接编译信息及两版本参数，代表 3 Hz 场景同时保存完整 CSV、PNG 和 SVG。

5 Hz / 20° 的名义稳态线性需求约 13.83 N.m，超过 7 N.m 限幅；图表与 JSON 将其显式标记为力矩不可达边界。应同时观察 RMSE、力矩限幅比例与输出峰值。即使 ESO 降低某些失配工况的误差，也不能声称它超越原作者硬件效果：本次没有在原装置或用户尚未搭好的装置上测量。
