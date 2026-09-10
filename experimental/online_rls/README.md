# 在线 RLS 参数估计 · 实验组件

[项目首页](../../README.md) · [辨识与整定](../../docs/system_identification.md) · [发布结果与曲线](../../results/online_rls/README.md)

本组件用纯 C11 递推最小二乘估计候选参数，通过逐窗口合成数据检查数值与边界行为。**默认关闭，不链接到 DM4310/GM6020 控制库，不修改手瞄、自瞄或电机命令。** 板级日志、C 激励前端、闭环偏差修正和自动增益切换尚未实现。

## 目录与接口

| 文件 | 用途 |
| --- | --- |
| [include/rls_shadow.h](include/rls_shadow.h) | 固定维数状态、配置、状态码与输入契约 |
| [src/rls_shadow.c](src/rls_shadow.c) | float32 RLS、Joseph 更新、参数与矩阵检查 |
| [tests/test_rls_shadow.c](tests/test_rls_shadow.c) | 参数恢复、拒绝冻结、批量等价与长期更新测试 |
| [run_experiments.py](run_experiments.py) | 主机窗口、激励判断、真实 C 调用及合成实验 |
| [CMakeLists.txt](CMakeLists.txt) | 独立或顶层可选构建 |

最多 6 个参数、无动态分配、无 HAL/RTOS 依赖，每实例独立状态。源文件归本仓库 MIT 许可；外部参考工程未复制到此目录。

## 先跑 C 测试

在仓库根目录执行，Linux 主机需要 C11 编译器和 CMake ≥ 3.16；此步骤不需要 Python：

```bash
cmake -S experimental/online_rls -B build/rls-standalone -DCMAKE_BUILD_TYPE=Release
cmake --build build/rls-standalone --parallel
ctest --test-dir build/rls-standalone --output-on-failure
```

这只运行一个 `rls_shadow` CTest，其中包含多组检查与 20,000 次六维更新。产物为主机 `libshadow_rls.a` 和 `libshadow_rls_host.so`，不能直接作为 STM32 固件链接产物。

需要与原控制器一起构建时：

```bash
cmake -S . -B build-rls -DCMAKE_BUILD_TYPE=Release -DGIMBAL_BUILD_RLS_EXPERIMENT=ON
cmake --build build-rls --parallel
ctest --test-dir build-rls --output-on-failure
```

此时共 8 个 CTest。默认关闭实验组件的主项目仍为 7 个；生产电机静态库不包含 RLS。`-DYAW_SANITIZERS=ON` 可启用地址和未定义行为检查。

## 运行合成实验

先按 [快速开始](../../docs/quickstart.md) 创建 `.venv` 并安装根目录 `requirements.txt`，然后执行：

```bash
.venv/bin/python experimental/online_rls/run_experiments.py --output build/online-rls-results
.venv/bin/python tools/check_online_rls_results.py --report build/online-rls-results/report.json
```

脚本默认在 `build/online-rls-host/` 编译实际 C 共享库；支持 `--cc` 或 `CC` 选择主机编译器，`--build-dir` 指定构建目录，`--no-plots` 只生成 JSON/CSV。所有默认生成路径位于 `build/`，脚本拒绝把源码目录和已发布的 `results/` 用作输出位置。

已有本项目 CMake 构建的库可复用：

```bash
.venv/bin/python experimental/online_rls/run_experiments.py --library build/rls-standalone/libshadow_rls_host.so --output build/online-rls-reuse
```

宿主库包含 C 源码/头文件哈希与 ABI 信息，脚本拒绝旧源码或不匹配的接口布局；运行前后还检查源码是否变化。这用于防止误用旧构建，不是对不可信共享库的安全沙箱。

| 输出 | 阅读方式 |
| --- | --- |
| `report.json` / `protocol.json` | 按种子的参数误差、门槛、异常结果与构建来源 |
| `online_rls_checks.png` / `.svg` | 参数恢复、慢漂、激励不足及失配反例 |
| `*.csv` | 逐窗口参数、状态码、激励判断和信息矩阵指标 |

独立校验器重新检查报告中的实际指标和声明条件，不只读取 `all_declared_algorithm_gates_passed`。实验脚本每次生成协议记录，不能把它称为完成了独立盲测或预注册。

## 更新契约

宿主提供归一化 `phi`、`y`、样本有效标志和激励标志，对应 `y = phiᵀ theta + residual`。原型检查有限数值、创新幅值、候选参数边界，以及 P 的对称/正定/元素范围；拒绝时参数和 P 都不改变，诊断计数正常更新。

- 遗忘只在接受更新时发生；P 是逆加权信息矩阵，不是已校准的真实参数置信度。
- Python 示例基于近期有效数据检查激励；C 不会自行识别陈旧时间戳、乱序、重复帧或机械限位。
- 主机激励统计每步从当前 100 窗口重建 Gram，避免大样本退出后因累计消减而长期冻结；非有限外积不加入统计。异常退出窗口后恢复判断的回归见 [实机前软件检查](../../docs/prehardware_review.md)，它不解决测量噪声偏差或错误物理标度。
- 硬拒绝可能在真实大幅变化后持续停更，需要记录和重新标定，不能保证自动恢复。
- 同一实例由一个任务拥有，状态与配置初始化后按只读契约使用；不能在多个中断/任务中同时更新。
- 参数拟合成功不会唯一决定 LQR 的 Q/R、ESO 带宽或积分，后续仍需独立模型与控制验证。

## STM32 资源与验证边界

原型状态为 276 字节、配置为 92 字节；**这不是任务栈需求**。当前 Joseph 矩阵更新与 Cholesky 检查约 O(p³)，维数上限为 6。一次已有 M4F 编译的静态分析中，update 自身栈帧为 680 字节，其矩阵检查为 200 字节；还要计入编译器差异、库函数、调用者和中断。

可用已有 ARM 工具链重新检查：

```bash
python3 tools/check_arm_build.py --include-experimental-rls --report build/arm-with-rls.json
```

此步骤检查目标编译及静态栈记录，尚未完成板上运行时间、完整固件链接或硬件闭环测试。[发布结果](../../results/online_rls/README.md) 明确保留测量噪声、标度错误和时序错位反例；它们不能被解释为实机控制性能改善。
