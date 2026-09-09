# 快速开始：先在电脑上跑通

[项目首页](../README.md) · [文档导航](README.md) · [STM32 接入](stm32_integration.md)

按下面顺序，先编译 C 控制器，再运行实际 C 参与的仿真，最后按需复现原版对照。**不需要电机、CAN 设备、MATLAB、ROS 或 GPU。** Python 只用于电脑端验证，最终部署的是 C 源码。

## 1. 编译并测试 C 控制器

以下命令面向 Linux / Bash；已验证 GCC 11、CMake 3.22、Python 3.10。C 构建需要支持 C11 的编译器和 CMake ≥ 3.16；此步骤不需要 Python。

```bash
git clone https://github.com/LYHrmer/robomaster-gimbal-lqr-eso.git
cd robomaster-gimbal-lqr-eso
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j 2
ctest --test-dir build --output-on-failure
```

预期看到 `100% tests passed, 0 tests failed out of 7`。这 7 个测试程序覆盖控制内核、Pitch 适配、两种电机协议、STM32 周期接口及多实例/角度处理。

| 构建产物 | 用途 |
| --- | --- |
| `build/libyaw_controller.so` | Python 仿真加载的实际 C 控制器 |
| `build/libgimbal_dm4310.a` | 当前电脑架构的 DM4310 静态库 |
| `build/libgimbal_gm6020.a` | 当前电脑架构的 GM6020 静态库 |

这一步生成的是电脑端产物。STM32 需要用 ARM 工具链重新编译。保留 `build` 目录名，后面的 Python 测试默认从这里加载共享库；C 源码变化后先重新构建。

## 2. 准备 Python 仿真环境

使用 Python ≥ 3.10 和独立虚拟环境安装 NumPy、SciPy、Matplotlib：

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
```

后续命令都在仓库根目录执行，直接使用 `.venv/bin/python`，不需要激活环境。**本页新生成的结果统一写入忽略目录 `build/quickstart-results/`**，保留仓库中已发布的 `results/`。

## 3. 运行第一组闭环仿真

```bash
.venv/bin/python sim/run_benchmarks.py --output build/quickstart-results/eso-ablation
```

这个脚本通过 `ctypes` 调用刚编译的 C 共享库，运行 6 个合成工况，比较**同一新版控制器中 ESO 补偿关闭与开启**的表现。无需下载原版源码。

完成后查看 `build/quickstart-results/eso-ablation/`：

| 文件 | 先看什么 |
| --- | --- |
| `README.md` | 各工况的误差和力矩指标汇总 |
| `stressed_3hz_20deg.png` | 综合扰动工况的跟踪曲线；同目录也有 SVG |
| `metrics.json` | 完整指标、假设参数、种子与源文件哈希 |
| `stressed_3hz_20deg.csv.gz` | 压缩后的逐采样轨迹；增加 `--all-csv` 可保留其他工况 |

这里使用声明的 **7 N·m 合成挑战条件**，不是两种电机的通用配置。输出中的 `TORQUE-INFEASIBLE` 表示该参考在模型的力矩约束下不可实现；脚本会保留它作为边界结果。此次运行用于检查仿真流程和 ESO 作用，不代表相对原版或实车的性能提升。

需要查看默认 `J/B/dt/Q/R` 如何生成 LQR 增益时，可单独运行：

```bash
.venv/bin/python tools/tune_lqr.py --output build/quickstart-results/lqr_design.json
```

## 4. 跑完整 Python 测试与固定原版对照

完整 Python 测试包含原版回归测试。先获取固定提交的两个原版源码文件，再运行测试，让联网准备与测试分开：

```bash
.venv/bin/python tools/fetch_upstream.py
.venv/bin/python -m unittest discover -s tests -p 'test_*.py'
```

预期为 `Ran 34 tests` 和 `OK`。获取脚本固定提交 `665c5b4ab1067d6cb63122c120822f27502953e5`，验证 SHA256 后缓存到 `build/upstream/`；后续命中完整缓存时不需要联网。完整测试本身不会重写已发布报告。

无法访问下载地址时，用包含该固定版本源码的本地目录替代获取命令，然后继续运行测试：

```bash
.venv/bin/python tools/fetch_upstream.py --source-dir /path/to/YAW_Auto_Controller
```

本地文件仍须通过固定哈希校验，任意版本或修改后的源码不能作为此对照。获取脚本不会修改原版目录。

准备完成后，生成缺陷回归报告和原/新版实际 C 对照：

```bash
.venv/bin/python tests/test_upstream_regressions.py --report build/quickstart-results/upstream-regressions.json
.venv/bin/python sim/compare_upstream.py --output build/quickstart-results/upstream-comparison
```

先看 `upstream-comparison/README.md` 和曲线，再按需查 `metrics.json`。这里仍是 7 N·m 合成挑战条件；电机约束下的验收需要下一步的专项仿真。

## 5. 按需复现电机与 Pitch 专项验证

完成前四步后，可以继续运行下列专项实验。它们使用电机协议量化、已声明的力矩/速度约束和多个种子，运行量大于入门示例。

```bash
.venv/bin/python tools/generate_profile_headers.py --check
.venv/bin/python sim/run_motor_profiles.py --output build/quickstart-results/motor-profiles
.venv/bin/python tools/check_performance.py --report build/quickstart-results/motor-profiles/metrics.json --output build/quickstart-results/yaw-acceptance.json
.venv/bin/python sim/run_pitch_profiles.py --phase full --output build/quickstart-results/pitch
```

`check_performance.py` 检查 Yaw 预定主工况，未通过会返回非零退出码。Pitch 默认保存成功和退步结果，并检查实验有效性与独立积分步长细化；**脚本正常结束不等于 Pitch 总性能验收通过**。当前 GM6020 Pitch 开发组未达预定门槛，报告的 `all_primary_acceptance_passed` 仍为 `false`。增加 `--require-primary-improvement` 会强制检查完整性能门槛，当前预期返回非零。

判断结论时阅读 [Yaw 验证报告](validation_report.md) 与 [Pitch 验证报告](pitch_validation.md)，同时核对对照参数、退步工况和适用范围。这里的参数都标为 `simulation_only`，需要替换为自身装置的参数才能进入实机实验。

## 6. 下一步：接入 STM32

| 你要做的事 | 阅读入口 |
| --- | --- |
| 选择 DM4310 MIT 或 GM6020 电流通道 | [DM4310 版本](../variants/dm4310/README.md) · [GM6020 版本](../variants/gm6020/README.md) |
| 理清控制器输入、任务周期、反馈快照和 CAN 回调 | [STM32 接入说明](stm32_integration.md) · [C 周期示例](../examples/stm32/gimbal_periodic.c) |
| 按 RoboMaster 官方例程定位接入位置 | [官方源码接入位置](opensource_integration_notes.md) · [RM 框架移植说明](rm_framework_porting.md) |
| 接入 Pitch，区分重力角和机械关节角 | [Pitch 接入](pitch_integration.md) |
| 处理连续大 yaw 或多个轴实例 | [多轴接入](multiaxis_integration.md) |
| 设计自己的参数并记录实机实验 | [参数来源](parameter_sources.md) · [实验步骤](experiment_plan.md) |

手瞄与自瞄可以共用控制内核，由上层生成连续的位置、速度、加速度参考。大 yaw—小 yaw—pitch 还需要参考分配和耦合处理；当前三实例测试验证状态隔离，不代表完成三轴整机闭环验证。

## 常见启动问题

| 现象 | 处理方法 |
| --- | --- |
| 缺少 `build/libyaw_controller.so` | 先完成第 1 步；Python 需要电脑端共享库，不能加载 ARM 静态库 |
| `venv` / `ensurepip` 不可用 | 安装当前系统对应的 Python venv 组件，再创建环境 |
| NumPy / SciPy 导入失败或出现二进制版本冲突 | 使用本页独立 `.venv`，避免混用系统包与用户目录中的包 |
| 固定原版源码下载失败 | 使用第 4 步的 `--source-dir` 离线获取方式 |
| `Upstream checksum mismatch` | 检查是否为指定提交的完整原始文件；保留哈希校验，不用修改后的文件替代基准 |
| 构建拒绝 `-ffast-math` / `-Ofast` | 移除这些选项；控制器依赖有效的 NaN/Inf 检查 |
