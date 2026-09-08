#!/usr/bin/env python3
"""Synthetic, fixed-gain sensitivity matrix using both actual C controllers."""
import argparse
import csv
import ctypes as ct
from dataclasses import asdict, dataclass, replace
import hashlib
import json
import math
from pathlib import Path
import platform
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import scipy
from scipy.signal import cont2discrete

from sim.c_core import Config, Core, Feedback, Output, Reference
from sim.motor_envelope import DM4310_24V_REFERENCE, actuator_limit
from sim.run_benchmarks import COLUMNS, DT, IX, RAMP, SEED, Plant, config_dict, make_config, reference_at
from sim.upstream_reference import UpstreamCore
from tools.tune_lqr import design_lqr

DURATION = 6.
EXTRA_COLUMNS = ["transmitted_command_nm", "following_interval_envelope_limited",
                 "following_interval_envelope_limit_eval_fraction", "integral_nm"]
TEAM_COMMIT = "f67ef28a8a6aedb047f1e65837330034313e0be8"


@dataclass(frozen=True)
class Scenario:
    name: str
    group: str
    description: str
    plant: Plant
    frequency_hz: float = 3.
    amplitude_deg: float = 10.
    torque_cap_nm: float = 3.
    speed_envelope: bool = True
    parameter_basis: str = "illustrative assumptions and relative multipliers, not a typical-population distribution"


def scenarios():
    base = Plant(external_torque_nm=.35, coulomb_nm=.12, torque_lag_s=.002,
                 command_delay_samples=1, position_quantum_rad=25./65535.,
                 velocity_quantum_rad_s=60./4095., position_noise_std_rad=.00007,
                 velocity_noise_std_rad_s=.01)
    first = Scenario("rated3_base_3hz10", "reference", "assumed base load; 3 N.m cap", base)
    cases = [first,
        replace(first, name="rated3_1hz5", group="trajectory", description="1 Hz / 5 deg", frequency_hz=1., amplitude_deg=5.),
        replace(first, name="rated3_2hz10", group="trajectory", description="2 Hz / 10 deg", frequency_hz=2.),
        replace(first, name="rated3_3hz20", group="trajectory", description="3 Hz / 20 deg, 3 N.m cap", amplitude_deg=20.),
        replace(first, name="peak7_3hz20", group="trajectory", description="3 Hz / 20 deg, 7 N.m peak scenario", amplitude_deg=20., torque_cap_nm=7.),
    ]
    for factor in (.5, .75, 1.25, 1.5):
        cases.append(replace(first, name=f"inertia_x{factor:g}", group="inertia",
            description=f"true J x{factor:g}; controller J fixed", plant=replace(base, inertia_kg_m2=.039*factor)))
    for factor in (.5, 1.5, 2.):
        cases.append(replace(first, name=f"damping_x{factor:g}", group="damping",
            description=f"true B x{factor:g}; controller B fixed", plant=replace(base, damping_nm_s_rad=.30*factor)))
    cases.append(replace(first, name="ideal_measurement", group="measurement",
        description="no quantization or measurement noise", plant=replace(base, position_quantum_rad=0.,
        velocity_quantum_rad_s=0., position_noise_std_rad=0., velocity_noise_std_rad_s=0.)))
    cases.append(replace(first, name="noise_std_x3", group="measurement",
        description="Gaussian noise std x3; quantization unchanged", plant=replace(base,
        position_noise_std_rad=.00021, velocity_noise_std_rad_s=.03)))
    for delay in (0, 3, 5, 10):
        cases.append(replace(first, name=f"command_delay_{delay}ms", group="delay",
            description=f"command delay {delay} samples, 2 ms actuator lag retained",
            plant=replace(base, command_delay_samples=delay)))
    cases.append(replace(first, name="actuator_lag_5ms", group="lag", description="assumed actuator lag 5 ms",
                         plant=replace(base, torque_lag_s=.005)))
    cases.append(replace(first, name="peak7_3hz20_no_speed_envelope", group="envelope",
        description="peak reference without speed derating, as an explicit model ablation",
        amplitude_deg=20., torque_cap_nm=7., speed_envelope=False))
    cases.append(replace(first, name="compound_2hz10", group="compound", frequency_hz=2.,
        description="J/B x1.5, noise std x3, command delay 5 samples", plant=replace(base,
        inertia_kg_m2=.039*1.5, damping_nm_s_rad=.30*1.5, position_noise_std_rad=.00021,
        velocity_noise_std_rad_s=.03, command_delay_samples=5)))
    team = replace(base, inertia_kg_m2=.0610521813, damping_nm_s_rad=.513734025, coulomb_nm=.3)
    basis = (f"same-author vehicle configuration macros at {TEAM_COMMIT}; "
             "comments refer to identification but raw data unavailable; not universal RoboMaster parameters")
    cases += [
        replace(first, name="author_runtime_2hz10", group="source_reference", frequency_hz=2., plant=team,
                description="author vehicle J/B/Coulomb macros, controller unchanged", parameter_basis=basis),
        replace(first, name="author_runtime_3hz10", group="source_reference", plant=team,
                description="author vehicle macros at 3 Hz, controller unchanged", parameter_basis=basis),
        replace(first, name="author_candidate_3hz10", group="source_reference",
                plant=replace(base, inertia_kg_m2=.0422563489, damping_nm_s_rad=.286217284),
                description="author tuning-script J/B candidate; friction remains assumed",
                parameter_basis=f"tuning-script candidate at {TEAM_COMMIT}, not confirmed active hardware configuration"),
    ]
    # Added after the initial 24 showed no speed-envelope activity. Retain all
    # original cases (including regressions) and add a paired physical boundary.
    cases += [replace(first, name="peak7_4hz20_speed_envelope", group="envelope_boundary",
                      description="4 Hz / 20 deg speed-envelope boundary", frequency_hz=4.,
                      amplitude_deg=20., torque_cap_nm=7.),
              replace(first, name="peak7_4hz20_no_speed_envelope", group="envelope_boundary",
                      description="same 4 Hz boundary without speed derating", frequency_hz=4.,
                      amplitude_deg=20., torque_cap_nm=7., speed_envelope=False)]
    assert len(cases) == 26
    return cases


def linear_feedback_diagnostics(plant, gain):
    """Exact ZOH plant/lag + integer command delay; excludes ESO and nonlinearities."""
    j, b, lag = plant.inertia_kg_m2, plant.damping_nm_s_rad, plant.torque_lag_s
    if lag > 0:
        a = np.array([[0., 1., 0.], [0., -b/j, 1/j], [0., 0., -1/lag]])
        u = np.array([[0.], [0.], [1/lag]])
        k = np.array([[gain["k_position"], gain["k_velocity"], 0.]])
    else:
        a = np.array([[0., 1.], [0., -b/j]])
        u = np.array([[0.], [1/j]])
        k = np.array([[gain["k_position"], gain["k_velocity"]]])
    n = a.shape[0]
    ad, bd, _, _, _ = cont2discrete((a, u, np.eye(n), np.zeros((n, 1))), DT)
    delay = plant.command_delay_samples
    if delay:
        closed = np.zeros((n+delay, n+delay))
        closed[:n, :n] = ad
        closed[:n, n:n+1] = bd
        for index in range(delay-1):
            closed[n+index, n+index+1] = 1.
        closed[-1, :n] = -k
    else:
        closed = ad - bd @ k
    frequencies = np.geomspace(.001, math.pi / DT * (1-1e-6), 6000)
    z = np.exp(1j*frequencies*DT)
    responses = np.linalg.solve(z[:, None, None]*np.eye(n)-ad, np.broadcast_to(bd, (len(z), n, 1)))
    loop = (k @ responses).reshape(-1) * z**(-delay)
    logmag = np.log(np.abs(loop))
    phase = np.unwrap(np.angle(loop))

    def crossing(values, target):
        hits = np.flatnonzero((values[:-1] > target) & (values[1:] <= target))
        if not len(hits):
            return None
        i = int(hits[0])
        fraction = (values[i]-target)/(values[i]-values[i+1])
        return i, fraction

    unity = crossing(logmag, 0.)
    phase_cross = crossing(phase, -math.pi)
    result = {
        "scope": "ideal PD/state-feedback reference model with ZOH actuator lag and command delay; excludes integral, velocity-error filtering, ESO, friction, limits, noise and reference feedforward; not the full configured controller margin",
        "sample_rate_hz": 1/DT, "eso_bandwidth_rad_s": 80.,
        "feedback_only_closed_loop_spectral_radius": float(np.max(np.abs(np.linalg.eigvals(closed)))),
        "gain_crossover_hz": None, "phase_margin_deg": None,
        "gain_margin_db": None, "additional_delay_margin_ms": None,
    }
    if unity:
        i, fraction = unity
        omega = math.exp(np.log(frequencies[i])*(1-fraction)+np.log(frequencies[i+1])*fraction)
        angle = phase[i]*(1-fraction)+phase[i+1]*fraction
        margin = math.pi+angle
        result.update(gain_crossover_hz=omega/(2*math.pi), phase_margin_deg=math.degrees(margin),
                      additional_delay_margin_ms=1000*margin/omega)
    if phase_cross:
        i, fraction = phase_cross
        result["gain_margin_db"] = float(-20/math.log(10)*(logmag[i]*(1-fraction)+logmag[i+1]*fraction))
    return result


def step_allow_latched_fault(core, controller, feedback, reference):
    if isinstance(core, Core):
        output = Output()
        core.lib.yaw_controller_step(ct.byref(controller), ct.byref(feedback),
                                    ct.byref(reference), DT, ct.byref(output))
        return output
    return core.step(controller, feedback, reference, DT)


def run_trial(core, scenario, gain, seed=SEED, duration=DURATION, substeps=4,
              limiter=None, quantizer=None, controller_config=None):
    """One common plant loop. Optional limiter/quantizer support motor-specific profiles."""
    plant = scenario.plant
    config = Config(**controller_config) if controller_config is not None else make_config(gain, .8)
    config.torque_limit_nm = scenario.torque_cap_nm
    controller = core.init(config)
    envelope = replace(DM4310_24V_REFERENCE, current_torque_limit_nm=scenario.torque_cap_nm)
    if limiter is None:
        limiter = (lambda u, v: actuator_limit(u, v, envelope)) if scenario.speed_envelope else (
            lambda u, v: min(max(u, -scenario.torque_cap_nm), scenario.torque_cap_nm))
    rng = np.random.default_rng(seed)
    data = np.zeros((round(duration/DT)+1, len(COLUMNS)+len(EXTRA_COLUMNS)))
    p = v = torque = 0.
    pending = [0.]*plant.command_delay_samples
    completed, failure, first_fault = True, None, None
    actual_count = 0
    for index in range(len(data)):
        time_s = index*DT
        rp, rv, ra = reference_at(time_s, scenario.frequency_hz, math.radians(scenario.amplitude_deg))
        noise = rng.normal(size=2)
        mp = p + noise[0]*plant.position_noise_std_rad
        mv = v + noise[1]*plant.velocity_noise_std_rad_s
        if plant.position_quantum_rad:
            mp = float(np.round(mp/plant.position_quantum_rad)*plant.position_quantum_rad)
        if plant.velocity_quantum_rad_s:
            mv = float(np.round(mv/plant.velocity_quantum_rad_s)*plant.velocity_quantum_rad_s)
        out = step_allow_latched_fault(core, controller, Feedback(mp, mv, 0., 0., True, False),
                                       Reference(rp, rv, ra, 0., True))
        if not all(math.isfinite(value) for value in (out.torque_nm, out.unconstrained_torque_nm,
                out.compensation_nm, out.disturbance_nm)):
            completed, failure = False, f"nonfinite controller output at {time_s:.6f} s"
            break
        if out.status > 1 and first_fault is None:
            first_fault = {"time_s": time_s, "status": int(out.status)}
        sent = quantizer(out.torque_nm) if quantizer else out.torque_nm
        data[index, :len(COLUMNS)] = [time_s, rp, rv, ra, p, v, mp, mv, out.torque_nm,
            torque, rp-p, out.compensation_nm, out.disturbance_nm, out.unconstrained_torque_nm,
            out.flags, out.status]
        data[index, len(COLUMNS)] = sent
        data[index, len(COLUMNS)+3] = out.integral_nm
        actual_count = index+1
        if index+1 == len(data):
            break
        pending.append(sent)
        command = pending.pop(0)
        evaluations = clipped = 0

        def derivative(px, vx, tx):
            nonlocal evaluations, clipped
            target = limiter(command, vx)
            evaluations += 1
            clipped += abs(target-command) > 1e-9
            used_torque = tx if plant.torque_lag_s > 0 else target
            acceleration = (used_torque + plant.external_torque_nm - plant.damping_nm_s_rad*vx
                            - plant.coulomb_nm*math.tanh(vx/plant.coulomb_velocity_rad_s))/plant.inertia_kg_m2
            return (vx, acceleration, (target-tx)/plant.torque_lag_s if plant.torque_lag_s > 0 else 0.)

        h = DT/substeps
        for _ in range(substeps):
            k1 = derivative(p, v, torque)
            k2 = derivative(p+h*k1[0]/2, v+h*k1[1]/2, torque+h*k1[2]/2)
            k3 = derivative(p+h*k2[0]/2, v+h*k2[1]/2, torque+h*k2[2]/2)
            k4 = derivative(p+h*k3[0], v+h*k3[1], torque+h*k3[2])
            p += h*(k1[0]+2*k2[0]+2*k3[0]+k4[0])/6
            v += h*(k1[1]+2*k2[1]+2*k3[1]+k4[1])/6
            torque += h*(k1[2]+2*k2[2]+2*k3[2]+k4[2])/6
            if plant.torque_lag_s == 0:
                torque = limiter(command, v)
        data[index, len(COLUMNS)+1] = clipped > 0
        data[index, len(COLUMNS)+2] = clipped/evaluations
        if not all(math.isfinite(value) for value in (p, v, torque)):
            completed, failure = False, f"nonfinite plant state after {time_s:.6f} s"
            break
    data = data[:actual_count]
    window = data[data[:, IX["time_s"]] >= RAMP]
    intervals = data[(data[:, IX["time_s"]] >= RAMP) & (data[:, IX["time_s"]] < duration)]
    metrics = {"completed": completed, "failure": failure, "first_latched_fault": first_fault,
               "evaluation_start_s": RAMP, "evaluation_end_s": duration,
               "actual_last_sample_s": float(data[-1, 0]) if len(data) else None,
               "actual_c_config": config_dict(config)}
    if len(window):
        error = np.rad2deg(window[:, IX["error_rad"]])
        metrics.update(position_rmse_deg=float(np.sqrt(np.mean(error**2))),
            position_max_abs_error_deg=float(np.max(np.abs(error))),
            command_rms_nm=float(np.sqrt(np.mean(window[:, IX["command_nm"]]**2))),
            actual_torque_rms_nm=float(np.sqrt(np.mean(window[:, IX["actuator_torque_nm"]]**2))),
            command_peak_abs_nm=float(np.max(np.abs(window[:, IX["command_nm"]]))),
            actual_torque_peak_abs_nm=float(np.max(np.abs(window[:, IX["actuator_torque_nm"]]))),
            actual_velocity_peak_abs_rad_s=float(np.max(np.abs(window[:, IX["velocity_rad_s"]]))),
            command_at_limit_fraction=float(np.mean(np.abs(window[:, IX["command_nm"]]) >= scenario.torque_cap_nm-1e-6)),
            controller_hard_limit_fraction=float(np.mean((window[:, IX["flags"]].astype(np.uint32)&1) != 0)),
            controller_slew_limit_fraction=float(np.mean((window[:, IX["flags"]].astype(np.uint32)&2) != 0)),
            integral_mean_nm=float(np.mean(window[:, len(COLUMNS)+3])),
            integral_rms_nm=float(np.sqrt(np.mean(window[:, len(COLUMNS)+3]**2))),
            integral_peak_abs_nm=float(np.max(np.abs(window[:, len(COLUMNS)+3]))),
            actuation_residual_mean_nm=float(np.mean(window[:, IX["command_nm"]]-window[:, IX["unconstrained_command_nm"]])),
            actuation_residual_rms_nm=float(np.sqrt(np.mean((window[:, IX["command_nm"]]-window[:, IX["unconstrained_command_nm"]])**2))),
            envelope_limited_interval_fraction=float(np.mean(intervals[:, len(COLUMNS)+1])) if len(intervals) else None,
            envelope_limited_rk4_evaluation_fraction=float(np.mean(intervals[:, len(COLUMNS)+2])) if len(intervals) else None,
            command_quantization_max_abs_nm=float(np.max(np.abs(window[:, len(COLUMNS)]-window[:, IX["command_nm"]]))),
            fault_sample_count=int(np.count_nonzero(data[:, IX["status"]] > 1)) if isinstance(core, Core) else None)
    return data, metrics


def write_trace(path, traces):
    import gzip
    import io
    with path.open("wb") as raw:
        with gzip.GzipFile(filename="", fileobj=raw, mode="wb", mtime=0) as compressed:
            with io.TextIOWrapper(compressed, encoding="utf-8", newline="") as stream:
                writer = csv.writer(stream)
                writer.writerow(["variant"]+COLUMNS+EXTRA_COLUMNS)
                for variant, trace in traces.items():
                    for row in trace:
                        writer.writerow([variant]+[format(float(value), ".9g") for value in row])


def plot_tracking(path, scenario, traces):
    plt.rcParams.update({"font.size": 10, "svg.fonttype": "none", "svg.hashsalt": "yaw-sensitivity-v1"})
    fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True, constrained_layout=True)
    first = next(iter(traces.values()))
    axes[0].plot(first[:, 0], np.rad2deg(first[:, 1]), "--", color="#222b38", label="Reference")
    colors = {"Original core": "#d1792d", "Original core (same Ki)": "#8956a2", "New core": "#1268ad"}
    for index, (name, trace) in enumerate(traces.items()):
        color = colors.get(name, ["#d1792d", "#1268ad", "#8956a2"][index % 3])
        axes[0].plot(trace[:, 0], np.rad2deg(trace[:, IX["position_rad"]]), color=color, label=name, lw=1.)
        axes[1].plot(trace[:, 0], np.rad2deg(trace[:, IX["error_rad"]]), color=color, lw=1.)
    for ax in axes:
        ax.axvspan(0, RAMP, color="#cbd2d9", alpha=.2)
        ax.grid(alpha=.2)
    axes[0].legend(ncol=2 if len(traces) > 2 else 3, fontsize=9)
    axes[0].set_ylabel("Position (deg)")
    axes[1].set_ylabel("Reference - actual (deg)")
    axes[1].set_xlabel("Time (s)")
    axes[0].set_title(f"SYNTHETIC | {scenario.name}\n{scenario.description}", loc="left")
    for extension in ("png", "svg"):
        fig.savefig(Path(str(path)+"."+extension), dpi=160,
                    metadata={"Date": None} if extension == "svg" else None)
    plt.close(fig)


def plot_matrix(output, cases):
    plt.rcParams.update({"font.size": 9, "svg.fonttype": "none", "svg.hashsalt": "yaw-sensitivity-v1"})
    labels, old, new, delta = [], [], [], []
    for name, case in cases.items():
        labels.append(name)
        old.append(case["variants"]["Original core"].get("position_rmse_deg", np.nan))
        new.append(case["variants"]["New core"].get("position_rmse_deg", np.nan))
        delta.append(case["new_rmse_change_percent"] if case["new_rmse_change_percent"] is not None else np.nan)
    y = np.arange(len(labels))
    fig, axes = plt.subplots(1, 2, figsize=(13, 10), sharey=True, constrained_layout=True,
                             gridspec_kw={"width_ratios": [1.2, 1.]})
    axes[0].barh(y-.17, old, height=.32, color="#d1792d", label="Original core")
    axes[0].barh(y+.17, new, height=.32, color="#1268ad", label="New core")
    axes[0].set_xscale("log")
    axes[0].set_yticks(y, labels)
    axes[0].invert_yaxis()
    axes[0].set_xlabel("True-position RMSE (deg), log scale")
    axes[0].legend(loc="lower right")
    axes[1].barh(y, delta, height=.6, color=["#b84b42" if value > 0 else "#398a63" for value in delta])
    axes[1].axvline(0, color="#222b38", lw=.8)
    axes[1].set_xlabel("New / original RMSE change (%)\nnegative = lower error")
    for ax in axes:
        ax.grid(axis="x", alpha=.2)
    fig.suptitle("SYNTHETIC | Fixed-gain DM4310 sensitivity matrix\nScenario choices are engineering tests, not a probability distribution", fontsize=13)
    for extension in ("png", "svg"):
        fig.savefig(output / f"matrix.{extension}", dpi=160,
                    metadata={"Date": None} if extension == "svg" else None)
    plt.close(fig)


def file_hash(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "results/robustness")
    parser.add_argument("--upstream-source", type=Path)
    parser.add_argument("--seed", type=int, default=SEED)
    parser.add_argument("--substeps", type=int, default=4)
    args = parser.parse_args()
    if args.seed < 0 or args.substeps < 1:
        parser.error("seed must be nonnegative and substeps positive")
    args.output.mkdir(parents=True, exist_ok=True)
    original, new = UpstreamCore(args.upstream_source), Core()
    design = design_lqr()
    gain = design["gain"]
    report = {"data_type": "SYNTHETIC", "seed": args.seed, "duration_s": DURATION,
        "dt_s": DT, "startup_s": RAMP, "substeps": args.substeps,
        "comparison": "unchanged pinned original C versus new C, same gains/ESO 0.8/plant/seed",
        "tuning_policy": "nominal J=.039/B=.30 and DLQR gains fixed for all cases; never retuned against true plant",
        "parameter_scope": "rated motor specifications and named author configurations plus explicit assumptions, not typical RoboMaster population statistics",
        "envelope": {**asdict(DM4310_24V_REFERENCE), "scope": "piecewise motoring envelope through assumed (0,7 N.m), rated (120 rpm,3 N.m), no-load (200 rpm,0); braking current cap only; no thermal or regeneration model"},
        "lag_scope": "2/5 ms torque lag is assumed, not a measured current-loop time constant",
        "measurement_scope": "feedback valid/age0, same calibrated zero-centered quantization/noise samples; not bit-exact CAN feedback replay; ESO receives last limited command, no oracle torque",
        "fault_policy": "latched C faults continue with returned zero torque; cases retained; nonfinite trajectories explicitly fail",
        "upstream": original.metadata, "new_core_source_sha256": file_hash(ROOT / "src/yaw_controller.c"),
        "new_library_sha256": file_hash(new.path), "sensitivity_source_sha256": file_hash(Path(__file__)),
        "motor_envelope_source_sha256": file_hash(ROOT / "sim/motor_envelope.py"),
        "source_sha256": {name: file_hash(ROOT/name) for name in (
            "include/yaw_controller.h", "sim/c_core.py", "sim/upstream_reference.py",
            "tools/fetch_upstream.py", "tools/tune_lqr.py", "sim/run_benchmarks.py")},
        "lqr_design": design,
        "dependencies": {"python": platform.python_version(), "numpy": np.__version__, "scipy": scipy.__version__, "matplotlib": matplotlib.__version__},
        "cases": {}}
    source_catalog = ROOT / "sim/parameter_sets.json"
    if source_catalog.is_file():
        report["parameter_catalog_sha256"] = file_hash(source_catalog)
    all_traces = {}
    for scenario in scenarios():
        variants, traces = {}, {}
        for label, core in [("Original core", original), ("New core", new)]:
            traces[label], variants[label] = run_trial(core, scenario, gain, args.seed, substeps=args.substeps)
        old, current = variants["Original core"], variants["New core"]
        change = ((current["position_rmse_deg"]/old["position_rmse_deg"]-1)*100
                  if old["completed"] and current["completed"] and old.get("position_rmse_deg", 0) else None)
        omega = math.tau*scenario.frequency_hz
        demand = math.radians(scenario.amplitude_deg)*math.hypot(scenario.plant.inertia_kg_m2*omega**2,
                                                                scenario.plant.damping_nm_s_rad*omega)
        report["cases"][scenario.name] = {"scenario": asdict(scenario), "variants": variants,
            "new_rmse_change_percent": change, "linear_required_torque_amplitude_nm": demand,
            "linear_demand_exceeds_current_cap": demand > scenario.torque_cap_nm,
            "linear_feedback_diagnostics": linear_feedback_diagnostics(scenario.plant, gain)}
        all_traces[scenario.name] = traces
        change_text = f"{change:+.2f}%" if change is not None else "incomplete"
        print(f"{scenario.name}: old={old.get('position_rmse_deg', float('nan')):.4f}, "
              f"new={current.get('position_rmse_deg', float('nan')):.4f} deg; {change_text}; "
              f"new command-limit={current.get('command_at_limit_fraction', 0):.1%}, "
              f"envelope={current.get('envelope_limited_interval_fraction', 0):.1%}, "
              f"fault={current['first_latched_fault']}", flush=True)
        (args.output / "metrics.json").write_text(json.dumps(report, indent=2, allow_nan=False)+"\n")
    plot_matrix(args.output, report["cases"])
    complete = [(name, case["new_rmse_change_percent"]) for name, case in report["cases"].items()
                if case["new_rmse_change_percent"] is not None]
    selected = ["rated3_base_3hz10", max(complete, key=lambda item: item[1])[0]] if complete else ["rated3_base_3hz10"]
    indexed = {scenario.name: scenario for scenario in scenarios()}
    for name in dict.fromkeys(selected):
        plot_tracking(args.output / name, indexed[name], all_traces[name])
        write_trace(args.output / f"{name}.csv.gz", all_traces[name])
    lines = ["# Synthetic DM4310 sensitivity matrix", "",
        "Fixed controller gains and nominal J/B. All original 24 scenarios are retained, plus two paired 4 Hz boundary cases added when the initial set showed no speed-envelope activity. Both actual C implementations receive the same plant and seed. This is not a sample of typical RoboMaster hardware.", "",
        "| Scenario | Original / new RMSE (deg) | Change | New command cap | New envelope clipping | New fault |",
        "| --- | ---: | ---: | ---: | ---: | --- |"]
    with (args.output / "matrix.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["scenario", "original_rmse_deg", "new_rmse_deg", "change_percent", "new_command_cap_fraction", "new_envelope_fraction", "new_first_fault"])
        for name, case in report["cases"].items():
            old, new_metrics = case["variants"]["Original core"], case["variants"]["New core"]
            change = case["new_rmse_change_percent"]
            change_text = f"{change:+.2f}%" if change is not None else "incomplete"
            lines.append(f"| {name} | {old.get('position_rmse_deg', float('nan')):.4f} / {new_metrics.get('position_rmse_deg', float('nan')):.4f} | "
                f"{change_text} | {new_metrics.get('command_at_limit_fraction', 0):.1%} | "
                f"{new_metrics.get('envelope_limited_interval_fraction', 0):.1%} | {new_metrics['first_latched_fault']} |")
            writer.writerow([name, old.get("position_rmse_deg"), new_metrics.get("position_rmse_deg"), change,
                new_metrics.get("command_at_limit_fraction"), new_metrics.get("envelope_limited_interval_fraction"), new_metrics["first_latched_fault"]])
    lines += ["", "Negative change means reduced RMSE. Regressions, saturations and faults remain in the matrix.",
        "The 3 N.m cap uses a rated-specification reference, not an all-speed continuous or thermal guarantee. 7 N.m cases are explicit peak assumptions without a permissible duration model.",
        "Actuator clipping is applied during every RK4 evaluation, before assumed torque lag. The clipping fraction counts control intervals with any such limiting; command-cap saturation is reported separately.",
        "The recorded crossover/margins are for the linear feedback-only ZOH model with lag/delay. They exclude ESO, friction and saturation and are not a stability certificate for the full controller.",
        "Representative traces include the base case and the largest relative regression, selected after retaining every case in this table.",
        "Read ../../docs/robustness.md and ../../docs/parameter_sources.md for parameter meanings and limits."]
    (args.output / "README.md").write_text("\n".join(lines)+"\n")


if __name__ == "__main__":
    main()
