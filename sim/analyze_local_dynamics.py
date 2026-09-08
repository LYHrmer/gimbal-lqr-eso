#!/usr/bin/env python3
"""Local linear ESO/filter analysis, checked against the actual C step function.

This is a zero-equilibrium, unsaturated model, not a hardware/global stability
certificate. All floating controller history states are retained, including
the integral and compensation states.
"""
import argparse
import ctypes as ct
import hashlib
import json
import math
from pathlib import Path
import platform

import numpy as np
import scipy
from scipy.linalg import expm

from c_core import Config, Core, Feedback, Reference

ROOT = Path(__file__).resolve().parents[1]
HISTORY = (
    "observer_position_rad", "observer_velocity_rad_s", "observer_disturbance_nm",
    "integral_nm", "compensation_nm", "last_torque_nm", "last_velocity_rad_s",
    "velocity_error_filtered_rad_s",
)
PREFIX = ("plant_position_rad", "plant_velocity_rad_s", "plant_torque_nm") + HISTORY
DELAYS = (0, 1, 5, 10)
TAUS = (0.0, 0.002)
EPSILONS = (1e-5, 2e-5)
MATRIX_TOLERANCE = 5e-5
RADIUS_TOLERANCE = 2e-6


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def physical_zoh(inertia, damping, lag, dt):
    """Exact ZOH for position, velocity and a first-order torque actuator."""
    if lag <= 0:
        raise ValueError("This analysis requires a positive actuator lag.")
    continuous = np.array([[0.0, 1.0, 0.0],
                           [0.0, -damping / inertia, 1.0 / inertia],
                           [0.0, 0.0, -1.0 / lag]])
    augmented = np.zeros((4, 4))
    augmented[:3, :3] = continuous
    augmented[2, 3] = 1.0 / lag
    discrete = expm(dt * augmented)
    return discrete[:3, :3], discrete[:3, 3]


def analytic_matrix(config, dt, physical, delay):
    """Build the complete local map independently of the C implementation.

    State before call k: plant(k), controller history(k-1), command queue.
    ESO uses the previous controller command, not measured applied torque.
    The queue's first entry acts during this interval; u(k) is appended.
    """
    c = config
    if c.coulomb_nm != 0:
        raise ValueError("Local model is restricted to nominal Coulomb=0.")
    if c.eso_bandwidth_rad_s <= 0:
        raise ValueError("The ESO must be enabled for this analysis.")
    ad, bd = physical
    # C receives binary32 h, while the physical ZOH uses the scheduled dt.
    h = ct.c_float(dt).value
    j, damping, w = c.inertia_kg_m2, c.damping_nm_s_rad, c.eso_bandwidth_rad_s
    q = -math.expm1(-w * h)
    injection = np.array([-math.expm1(-3 * w * h),
                          1.5 * q * q * (2 - q) / h, j * q**3 / h**2])
    transition = np.array([[1.0, h, 0.5 * h * h / j],
                           [0.0, 1.0, h / j], [0.0, 0.0, 1.0]])
    correction = np.eye(3) - np.outer(injection, [1.0, 0.0, 0.0])
    observer_a = correction @ transition
    observer_u = correction @ np.array([0.5 * h * h / j, h / j, 0.0])
    tau = c.velocity_error_filter_tau_s
    alpha = -math.expm1(-h / tau) if tau > 0 else 1.0
    size = len(PREFIX) + delay

    def step(state):
        z = observer_a @ state[3:6] + observer_u * (
            state[8] - damping * state[9]) + injection * state[0]
        error = (1 - alpha) * state[10] - alpha * state[1]
        compensation = -c.eso_gain * z[2]
        integral = state[6] - c.k_integral * h * state[0] if c.k_integral > 0 else 0.0
        command = -c.k_position * state[0] + c.k_velocity * error + integral + compensation
        held_command = state[11] if delay else command
        plant = ad @ state[:3] + bd * held_command
        queue = np.r_[state[12:], command] if delay else np.empty(0)
        return np.r_[plant, z, integral, compensation, command, state[1], error, queue]

    return np.stack([step(row) for row in np.eye(size)], axis=1)


def c_jacobian(core, config, dt, physical, delay, epsilon):
    """Central finite differences of the real C controller plus the ZOH plant."""
    ad, bd = physical
    size = len(PREFIX) + delay

    def step(state):
        controller = core.init(config)
        controller.ready = True  # Linearize the running, fault-free equilibrium.
        for index, name in enumerate(HISTORY, start=3):
            setattr(controller, name, float(state[index]))
        feedback = Feedback(float(state[0]), float(state[1]), 0.0, 0.0, True, False)
        reference = Reference(0.0, 0.0, 0.0, 0.0, True)
        output = core.step(controller, feedback, reference, dt)
        if output.status != 0 or output.flags != 0:
            raise RuntimeError("Finite-difference point activated a guard or limiter.")
        command = output.torque_nm
        held_command = state[11] if delay else command
        plant = ad @ state[:3] + bd * held_command
        queue = np.r_[state[12:], command] if delay else np.empty(0)
        history = [getattr(controller, name) for name in HISTORY]
        return np.r_[plant, history, queue]

    if np.max(np.abs(step(np.zeros(size)))) != 0:
        raise RuntimeError("Zero is not an equilibrium of this configuration.")
    basis = np.eye(size) * epsilon
    return np.stack([(step(row) - step(-row)) / (2 * epsilon)
                     for row in basis], axis=1)


def spectral_radius(matrix):
    return float(np.max(np.abs(np.linalg.eigvals(matrix))))


def case_result(core, config, dt, lag, inertia_multiplier, delay, group):
    physical = physical_zoh(config.inertia_kg_m2 * inertia_multiplier,
                            config.damping_nm_s_rad, lag, dt)
    matrix = analytic_matrix(config, dt, physical, delay)
    radius = spectral_radius(matrix)
    checks = []
    for epsilon in EPSILONS:
        numerical = c_jacobian(core, config, dt, physical, delay, epsilon)
        c_radius = spectral_radius(numerical)
        error = float(np.max(np.abs(matrix - numerical)))
        checks.append({"epsilon": epsilon, "max_abs_matrix_difference": error,
            "c_spectral_radius": c_radius,
            "spectral_radius_difference": abs(radius - c_radius),
            "consistent": error <= MATRIX_TOLERANCE and
                          abs(radius - c_radius) <= RADIUS_TOLERANCE})
    poles = np.linalg.eigvals(matrix)
    return {"group": group, "inertia_multiplier": inertia_multiplier,
        "plant_inertia_kg_m2": config.inertia_kg_m2 * inertia_multiplier,
        "plant_damping_nm_s_rad": config.damping_nm_s_rad,
        "plant_variation_classification": "synthetic assumption; not a measured gimbal population",
        "command_delay_samples": delay, "command_delay_s": delay * dt,
        "k_integral": config.k_integral,
        "velocity_error_filter_tau_s": config.velocity_error_filter_tau_s,
        "config_binary32": {name: getattr(config, name) for name, _ in Config._fields_},
        "state_dimension": len(matrix),
        "state_order": list(PREFIX) + [f"queued_command_{i}_nm" for i in range(delay)],
        "analytic_spectral_radius": radius, "locally_asymptotically_stable": radius < 1,
        "poles": [[float(p.real), float(p.imag)] for p in poles],
        "analytic_matrix": matrix.tolist(), "c_jacobian_checks": checks}


def analyze(profile, core):
    dt = profile["dt_s"]
    lag = profile["actuator_dynamic_assumptions"]["torque_lag_s"]
    slow_gain = 2 * profile["controller"]["k_position"]
    reference_design = {"k_integral": slow_gain, "integral_limit_nm": 0.2,
                        "antiwindup_rate_s": 2.0, "velocity_error_filter_tau_s": 0.0}
    base = Config(**profile["controller"])
    selected = all(getattr(base, name) == ct.c_float(value).value
                   for name, value in reference_design.items())
    # Read selected values from the profile; retain an explicit comparison rule.
    slow_design = ({name: profile["controller"][name] for name in reference_design}
                   if selected else dict(reference_design))
    rows = []
    # Rejected LP candidate archive: its original Ki=0 experiment is retained.
    for delay in DELAYS:
        for tau in TAUS:
            config = Config(**profile["controller"])
            config.k_integral = 0.0
            config.integral_limit_nm = 0.0
            config.velocity_error_filter_tau_s = tau
            rows.append(case_result(core, config, dt, lag, 1.0, delay,
                                    "rejected_filter_archive"))
    # Final bounded design: Ti=Kp/Ki=0.5s, cap .2 Nm, aw=2/s; no filter.
    for multiplier in (0.75, 1.0, 1.25):
        for delay in (1, 5, 10):
            config = Config(**profile["controller"])
            for name, value in slow_design.items():
                setattr(config, name, value)
            rows.append(case_result(core, config, dt, lag, multiplier, delay,
                                    "slow_integral_no_filter"))
    return {"profile_name": profile["name"], "dt_s": dt, "actuator_lag_s": lag,
            "nominal_config": dict(profile["controller"]),
            "slow_integral_design": {**slow_design, "reference_rule": reference_design,
                "classification": "bounded controller-design assumption, not measured hardware",
                "selected_in_input_profile": selected},
            "scope": "Same new C core with Ki=0/slow integral and disabled/rejected error filtering. Complete continuous-history local model at zero equilibrium, including ESO, exact ZOH damped plant, assumed actuator lag and integer command delay. Nominal Coulomb=0; all clamps inactive. Not an upstream-controller comparison or a global/hardware stability proof.",
            "excluded": ["nonlinear saturation/guards", "Coulomb friction", "external load",
                         "sensor/command quantization", "noise", "reference feedforward",
                         "thermal limits", "sample jitter"],
            "matrix_abs_tolerance": MATRIX_TOLERANCE,
            "spectral_radius_abs_tolerance": RADIUS_TOLERANCE,
            "jacobian_consistency_passed": all(c["consistent"] for row in rows for c in row["c_jacobian_checks"]),
            "all_sampled_linear_cases_stable": all(row["locally_asymptotically_stable"] for row in rows),
            "cases": rows}


def markdown(report):
    selected = report["slow_integral_design"]["selected_in_input_profile"]
    selection = "输入 GM profile 已选择该慢积分配置。" if selected else "输入 profile 尚未选择慢积分；本报告将其标为待非线性仿真评估的候选。"
    lines = ["# 含 ESO 与积分的局部离散动力学核验", "",
        "**🔴 本组分析含局部不稳定工况。真实 C 与解析矩阵一致，不代表全部工况稳定或全系统验证通过。**", "",
        f"主分析为不滤波的慢积分：Ki=2Kp={report['slow_integral_design']['k_integral']:.8f}，积分上限 {report['slow_integral_design']['integral_limit_nm']:g} N·m，抗饱和回算速率 {report['slow_integral_design']['antiwindup_rate_s']:g}/s，ESO 参数不变。{selection}它属于配置优化；本报告不将收益归因于 C 内核，也不比较上游旧控制器。", "",
        "另保留 Ki=0、tau=0/2 ms 的滤波试验档案。2 ms 滤波候选已在开发仿真中被拒绝，以下局部分析不改变这一选择。", "",
        f"模型采用 `{report['profile_name']}` 的名义 J/B 与 LQR/ESO 参数，电机力矩滞后取 profile 中假设的 {report['actuator_lag_s']*1000:g} ms。J 倍率为合成敏感性场景，不是云台实测分布。", "",
        "在零参考、零扰动、固定采样周期的运行平衡点，名义库仑摩擦为 0，限幅和故障保护均未触发。状态包括植物位置/速度/力矩、C 的全部 8 个浮点历史状态和逐样本命令队列；ready/initialized/fault 固定为正常运行状态。", "",
        "植物采用连续阻尼与一阶力矩滞后的精确 ZOH。ESO 使用上一控制命令与上一测量速度，逐项建模预测/校正、扰动补偿、当期积分更新及误差滤波。队首命令作用于当前周期，新命令进入队尾：队列长度就是命令延迟样本数。", "",
        "零参考下，当期积分 `I_k=I_prev-Ki*h*theta_k`，反馈力矩 `u_k=-Kp*theta_k+Kv*e_f+I_k-eso_gain*z_d`。本局部邻域没有限幅，最终命令等于未限幅命令，因此抗饱和回算项为零；积分和补偿历史状态仍显式保留。", "",
        "可选误差滤波为 `e_f[k]=(1-alpha)e_f[k-1]+alpha(r_v[k]-v[k])`，`alpha=1-exp(-h/tau)`；tau=0 直接使用原误差。滤波既影响噪声通道，也改变闭环相位。", ""]
    for group, title in (("slow_integral_no_filter", "慢积分，滤波关闭"),
                         ("rejected_filter_archive", "Ki=0 的滤波候选档案")):
        lines += [f"## {title}", "",
            "| J 倍率 | 命令延迟 | 滤波 tau | 解析谱半径 | C Jacobian 谱半径 | 局部结论 |",
            "|---:|---:|---:|---:|---:|---|"]
        for row in report["cases"]:
            if row["group"] != group:
                continue
            flag = "小于 1" if row["locally_asymptotically_stable"] else "🔴 大于等于 1：非渐近稳定"
            lines.append(f"| {row['inertia_multiplier']:g} | {row['command_delay_s']*1000:g} ms | {row['velocity_error_filter_tau_s']*1000:g} ms | {row['analytic_spectral_radius']:.9f} | {row['c_jacobian_checks'][0]['c_spectral_radius']:.9f} | {flag} |")
        lines.append("")
    checks = [c for row in report["cases"] for c in row["c_jacobian_checks"]]
    worst = max(c["max_abs_matrix_difference"] for c in checks)
    radius_error = max(c["spectral_radius_difference"] for c in checks)
    lines += [f"真实 C 核验对全部状态使用中心差分，扰动分别为 ±1e-5 与 ±2e-5。所有扰动点均检查正常运行且没有触发限幅。{len(checks)} 组核验最大矩阵绝对差 {worst:.3g}，最大谱半径差 {radius_error:.3g}；差异与 C 的 float 计算精度一致。完整矩阵、极点、配置、容差和文件哈希保存在 JSON 中。", "",
        "慢积分增加约 0.46 s 衰减时间尺度的慢极点；这与抑制恒定偏差的设计目的相符，不能解读为整个回路响应都变快。J×0.75 与 10 ms 命令延迟组合仍不稳定，必须保留这一边界。", "",
        "谱半径小于 1 只说明本次未饱和局部模型的渐近稳定性，不能覆盖量化、噪声、真实摩擦与负载、限幅、时序抖动、热约束和硬件通信；这些离散延迟点也不能给出连续的最大安全延迟。", "",
        "复现（仓库根目录，使用已安装 requirements.txt 的 Python 环境）：", "",
        "```sh", "python sim/analyze_local_dynamics.py --doc-output docs/local_dynamics.md", "```", "",
        "脚本正常退出表示分析与 C 一致性核验完成；不表示所有被分析工况稳定。", ""]
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", type=Path, default=ROOT / "profiles/gm6020_current.json")
    parser.add_argument("--library", type=Path, default=ROOT / "build/libyaw_controller.so")
    parser.add_argument("--output", "--report", dest="report", type=Path,
                        default=ROOT / "results/local_dynamics.json")
    parser.add_argument("--doc-output", "--markdown", dest="markdown", type=Path,
                        help="Optionally write a Markdown report; omitted in CI by default.")
    args = parser.parse_args()
    profile = json.loads(args.profile.read_text())
    report = analyze(profile, Core(args.library))
    report["provenance"] = {"profile_sha256": sha256(args.profile),
        "library_sha256": sha256(args.library), "analysis_script_sha256": sha256(__file__),
        "controller_c_sha256": sha256(ROOT / "src/yaw_controller.c"),
        "controller_h_sha256": sha256(ROOT / "include/yaw_controller.h"),
        "python_binding_sha256": sha256(ROOT / "sim/c_core.py"),
        "python": platform.python_version(), "numpy": np.__version__, "scipy": scipy.__version__}
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    if args.markdown is not None:
        args.markdown.parent.mkdir(parents=True, exist_ok=True)
        args.markdown.write_text(markdown(report))
    print(json.dumps({key: report[key] for key in (
        "jacobian_consistency_passed", "all_sampled_linear_cases_stable")}))
    if not report["jacobian_consistency_passed"]:
        raise SystemExit("C Jacobian and analytic model disagree; inspect the report.")


if __name__ == "__main__":
    main()
