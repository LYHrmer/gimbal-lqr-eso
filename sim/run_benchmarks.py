#!/usr/bin/env python3
"""SYNTHETIC experiments; the controller is build/libyaw_controller.so."""
import argparse
import csv
import gzip
import hashlib
import json
import math
import platform
from dataclasses import asdict, dataclass
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import scipy

from sim.c_core import Config, Core, Feedback, Reference
from tools.tune_lqr import design_lqr

DT = .001
DURATION = 8.
RAMP = 2.
SEED = 4310
ESO_GAIN = .8


@dataclass(frozen=True)
class Plant:
    inertia_kg_m2: float = .039
    damping_nm_s_rad: float = .30
    external_torque_nm: float = 0.
    coulomb_nm: float = 0.
    coulomb_velocity_rad_s: float = .04
    torque_lag_s: float = 0.
    command_delay_samples: int = 0
    position_quantum_rad: float = 0.
    velocity_quantum_rad_s: float = 0.
    position_noise_std_rad: float = 0.
    velocity_noise_std_rad_s: float = 0.


def reference_at(time_s, frequency_hz, amplitude_rad):
    """C2 quintic startup envelope, including its exact first two derivatives."""
    s = min(1., max(0., time_s / RAMP))
    e = 10*s**3 - 15*s**4 + 6*s**5
    de = (30*s**2 - 60*s**3 + 30*s**4) / RAMP if 0 < s < 1 else 0.
    dde = (60*s - 180*s**2 + 120*s**3) / RAMP**2 if 0 < s < 1 else 0.
    w = 2 * math.pi * frequency_hz
    sn, cs = math.sin(w*time_s), math.cos(w*time_s)
    return (amplitude_rad*e*sn,
            amplitude_rad*(de*sn + e*w*cs),
            amplitude_rad*(dde*sn + 2*de*w*cs - e*w*w*sn))


def make_config(gain, eso_gain):
    return Config(
        inertia_kg_m2=.039, damping_nm_s_rad=.30,
        k_position=gain["k_position"], k_velocity=gain["k_velocity"],
        k_integral=0., integral_limit_nm=0., antiwindup_rate_s=20.,
        coulomb_nm=0., coulomb_velocity_rad_s=.04,
        eso_bandwidth_rad_s=80., eso_gain=eso_gain,
        disturbance_limit_nm=3., compensation_limit_nm=2.,
        compensation_slew_nm_s=100., torque_limit_nm=7., torque_slew_nm_s=1000.,
        dt_min_s=.0005, dt_max_s=.002,
        feedback_timeout_s=.005, reference_timeout_s=.005,
        position_min_rad=-math.pi, position_max_rad=math.pi,
        velocity_limit_rad_s=60., tracking_error_limit_rad=math.pi)


def config_dict(config):
    return {name: getattr(config, name) for name, _ in Config._fields_}


def plant_step(state, command_nm, plant, dt, substeps):
    """RK4 mechanical/actuator plant, independent of the C controller."""
    h = dt / substeps

    def derivative(x):
        p, v, tau = x
        used_torque = tau if plant.torque_lag_s > 0 else command_nm
        acceleration = (used_torque + plant.external_torque_nm
                        - plant.damping_nm_s_rad * v
                        - plant.coulomb_nm * math.tanh(v / plant.coulomb_velocity_rad_s)) / plant.inertia_kg_m2
        torque_derivative = ((command_nm - tau) / plant.torque_lag_s
                             if plant.torque_lag_s > 0 else 0.)
        return np.array([v, acceleration, torque_derivative])

    x = state.copy()
    for _ in range(substeps):
        k1 = derivative(x)
        k2 = derivative(x + .5*h*k1)
        k3 = derivative(x + .5*h*k2)
        k4 = derivative(x + h*k3)
        x += h*(k1 + 2*k2 + 2*k3 + k4)/6
    if plant.torque_lag_s == 0:
        x[2] = command_nm
    if not np.all(np.isfinite(x)):
        raise RuntimeError("nonfinite plant state")
    return x


COLUMNS = ["time_s", "reference_position_rad", "reference_velocity_rad_s",
           "reference_acceleration_rad_s2", "position_rad", "velocity_rad_s",
           "measured_position_rad", "measured_velocity_rad_s", "command_nm",
           "actuator_torque_nm", "error_rad", "compensation_nm", "disturbance_nm",
           "unconstrained_command_nm", "flags", "status"]
IX = {name: i for i, name in enumerate(COLUMNS)}


def run_case(core, plant, frequency, amplitude_deg, gain, eso_gain, seed, substeps=5):
    config = make_config(gain, eso_gain)
    controller = core.init(config)
    rng = np.random.default_rng(seed)
    state = np.zeros(3)
    delay = [0.] * plant.command_delay_samples
    count = round(DURATION / DT) + 1
    samples = np.zeros((count, len(COLUMNS)))
    for i in range(count):
        now = i*DT
        p, v, a = reference_at(now, frequency, math.radians(amplitude_deg))
        measured = state[:2] + rng.normal(size=2)*np.array([
            plant.position_noise_std_rad, plant.velocity_noise_std_rad_s])
        for axis, quantum in enumerate([plant.position_quantum_rad, plant.velocity_quantum_rad_s]):
            if quantum > 0:
                measured[axis] = np.round(measured[axis]/quantum)*quantum
        feedback = Feedback(measured[0], measured[1], 0., 0., True, False)
        reference = Reference(p, v, a, 0., True)
        out = core.step(controller, feedback, reference, DT)
        samples[i] = [now, p, v, a, state[0], state[1], measured[0], measured[1],
                      out.torque_nm, state[2], p-state[0], out.compensation_nm,
                      out.disturbance_nm, out.unconstrained_torque_nm, out.flags, out.status]
        if i + 1 < count:
            delay.append(out.torque_nm)
            delayed_command = delay.pop(0)
            state = plant_step(state, delayed_command, plant, DT, substeps)
    window = samples[samples[:, IX["time_s"]] >= RAMP]
    error_deg = np.rad2deg(window[:, IX["error_rad"]])
    flags = window[:, IX["flags"]].astype(np.uint32)
    metrics = {
        "evaluation_start_s": RAMP, "evaluation_end_s": DURATION,
        "position_rmse_deg": float(np.sqrt(np.mean(error_deg**2))),
        "position_mae_deg": float(np.mean(np.abs(error_deg))),
        "position_p95_abs_error_deg": float(np.percentile(np.abs(error_deg), 95)),
        "position_max_abs_error_deg": float(np.max(np.abs(error_deg))),
        "command_rms_nm": float(np.sqrt(np.mean(window[:, IX["command_nm"]]**2))),
        "command_peak_abs_nm": float(np.max(np.abs(window[:, IX["command_nm"]]))),
        "actuator_peak_abs_nm": float(np.max(np.abs(window[:, IX["actuator_torque_nm"]]))),
        "torque_limit_fraction": float(np.mean((flags & 1) != 0)),
        "slew_limit_fraction": float(np.mean((flags & 2) != 0)),
        "disturbance_limit_fraction": float(np.mean((flags & 8) != 0)),
        "compensation_limit_fraction": float(np.mean((flags & 16) != 0)),
        "fault_count": int(np.count_nonzero(samples[:, IX["status"]] > 1)),
        "warmup_count": int(np.count_nonzero(samples[:, IX["status"]] == 1)),
    }
    return samples, metrics


def plot_case(output_dir, name, frequency, amplitude_deg, traces, boundary):
    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 10,
                         "axes.spines.top": False, "axes.spines.right": False,
                         "svg.fonttype": "none", "svg.hashsalt": "dm4310-synthetic-v1"})
    fig, axes = plt.subplots(2, 1, figsize=(10.2, 6.4), sharex=True,
                             gridspec_kw={"height_ratios": [1.55, 1.]}, constrained_layout=True)
    first = next(iter(traces.values()))
    t = first[:, IX["time_s"]]
    axes[0].plot(t, np.rad2deg(first[:, IX["reference_position_rad"]]),
                 color="#202b38", linestyle="--", linewidth=1.2, label="Reference")
    for (label, data), color in zip(traces.items(), ["#d1792d", "#1268ad"]):
        axes[0].plot(t, np.rad2deg(data[:, IX["position_rad"]]),
                     color=color, linewidth=1.05, label=label)
        axes[1].plot(t, np.rad2deg(data[:, IX["error_rad"]]), color=color, linewidth=1.)
    for ax in axes:
        ax.axvspan(0, RAMP, color="#cbd2d9", alpha=.22)
        ax.axvline(RAMP, color="#758394", linewidth=.7, linestyle=":")
        ax.grid(alpha=.18)
        ax.set_xlim(0, DURATION)
    axes[0].set_ylabel("Position (deg)")
    axes[1].set_ylabel("Reference - actual (deg)")
    axes[1].set_xlabel("Time (s); shaded interval = smooth startup")
    axes[0].legend(ncol=3, loc="upper right", fontsize=9)
    kind = "Combined mismatch, load, friction, lag, delay, noise" if name.startswith("stressed") else "Nominal plant"
    suffix = " | torque-infeasible boundary" if boundary else ""
    axes[0].set_title(f"SYNTHETIC | {kind}\n{frequency:g} Hz, {amplitude_deg:g} deg amplitude{suffix}",
                      loc="left", fontsize=11, pad=12)
    for extension in ("png", "svg"):
        metadata = {"Creator": "dm4310-lqr-eso-controller synthetic simulation"}
        if extension == "svg":
            metadata["Date"] = None
        fig.savefig(output_dir / f"{name}.{extension}", dpi=180,
                    metadata=metadata)
    plt.close(fig)


def write_csv(path, traces):
    # Reproducible gzip header: fixed mtime and no original filename.
    import io
    with path.open("wb") as raw:
        with gzip.GzipFile(fileobj=raw, mode="wb", filename="", mtime=0) as zipped:
            with io.TextIOWrapper(zipped, encoding="utf-8", newline="") as stream:
                writer = csv.writer(stream)
                writer.writerow(["variant"] + COLUMNS)
                for variant, trace in traces.items():
                    for row in trace:
                        writer.writerow([variant] + [format(float(value), ".9g") for value in row])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "results")
    parser.add_argument("--library", type=Path, default=ROOT / "build/libyaw_controller.so")
    parser.add_argument("--seed", type=int, default=SEED)
    parser.add_argument("--substeps", type=int, default=5, help="RK4 plant substeps per control interval")
    parser.add_argument("--all-csv", action="store_true", help="also retain other five trajectory CSVs")
    args = parser.parse_args()
    if args.seed < 0 or args.substeps < 1:
        parser.error("seed must be nonnegative and substeps must be positive")
    args.output.mkdir(parents=True, exist_ok=True)
    core = Core(args.library)
    design = design_lqr()
    gain = design["gain"]
    (args.output / "lqr_design.json").write_text(json.dumps(design, indent=2, allow_nan=False) + "\n")
    nominal = Plant()
    stressed = Plant(inertia_kg_m2=.039*1.25, damping_nm_s_rad=.30*.8,
                     external_torque_nm=.35, coulomb_nm=.12, torque_lag_s=.002,
                     command_delay_samples=1, position_quantum_rad=25./65535.,
                     velocity_quantum_rad_s=60./4095., position_noise_std_rad=.00007,
                     velocity_noise_std_rad_s=.01)
    result = {
        "data_type": "SYNTHETIC: no hardware measurements",
        "comparison": "same new C core: ESO compensation gain 0 versus 0.8; integral disabled",
        "scope": "not a reproduction of, or measured improvement over, original-author hardware",
        "clock": {"dt_s": DT, "duration_s": DURATION, "startup_s": RAMP,
                  "sample_semantics": "reference and plant measurement at t; command issued at t; evolve to t+dt",
                  "feedback_age_s": 0, "reference_age_s": 0,
                  "applied_torque_valid": False},
        "integration": {"method": "RK4", "substeps": args.substeps},
        "seed": args.seed,
        "abi_struct_sizes_bytes": core.layout,
        "core_source_sha256": hashlib.sha256((ROOT / "src/yaw_controller.c").read_bytes()).hexdigest(),
        "core_header_sha256": hashlib.sha256((ROOT / "include/yaw_controller.h").read_bytes()).hexdigest(),
        "shared_library_sha256": hashlib.sha256(core.path.read_bytes()).hexdigest(),
        "dependencies": {"python": platform.python_version(), "numpy": np.__version__,
                         "scipy": scipy.__version__, "matplotlib": matplotlib.__version__,
                         "platform": platform.platform()},
        "controller_variants": {"ESO gain 0": config_dict(make_config(gain, 0.)),
                                "ESO gain 0.8": config_dict(make_config(gain, ESO_GAIN))},
        "cases": {},
    }
    for kind, plant in [("nominal", nominal), ("stressed", stressed)]:
        for frequency, amplitude in [(1., 5.), (3., 20.), (5., 20.)]:
            name = f"{kind}_{frequency:g}hz_{amplitude:g}deg"
            omega, amplitude_rad = 2*math.pi*frequency, math.radians(amplitude)
            linear_torque_amplitude = amplitude_rad*math.hypot(
                plant.inertia_kg_m2*omega**2, plant.damping_nm_s_rad*omega)
            boundary = linear_torque_amplitude > 7.
            traces, variants = {}, {}
            for label, eso_gain in [("ESO gain 0", 0.), ("ESO gain 0.8", ESO_GAIN)]:
                traces[label], variants[label] = run_case(core, plant, frequency, amplitude,
                                                         gain, eso_gain, args.seed, args.substeps)
            before = variants["ESO gain 0"]["position_rmse_deg"]
            after = variants["ESO gain 0.8"]["position_rmse_deg"]
            result["cases"][name] = {
                "frequency_hz": frequency, "amplitude_deg": amplitude, "plant": asdict(plant),
                "steady_linear_required_torque_amplitude_nm": linear_torque_amplitude,
                "torque_infeasible_linear_reference": boundary,
                "variants": variants,
                "eso_rmse_change_percent": (after/before - 1)*100 if before else None,
            }
            plot_case(args.output, name, frequency, amplitude, traces, boundary)
            if args.all_csv or name == "stressed_3hz_20deg":
                write_csv(args.output / f"{name}.csv.gz", traces)
            print(f"{name}: RMSE {before:.4f} -> {after:.4f} deg; "
                  f"ESO saturation {variants['ESO gain 0.8']['torque_limit_fraction']:.1%}"
                  + (" [TORQUE-INFEASIBLE]" if boundary else ""), flush=True)
    (args.output / "metrics.json").write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    lines = ["# Synthetic benchmark results", "",
             "These are generated plant simulations, not DM4310 measurements. Both variants call the same C core.", "",
             "| Scenario | RMSE: ESO 0 (deg) | RMSE: ESO 0.8 (deg) | ESO torque saturation | Torque-infeasible reference |",
             "| --- | ---: | ---: | ---: | --- |"]
    for name, case in result["cases"].items():
        variants = case["variants"]
        lines.append(f"| {name} | {variants['ESO gain 0']['position_rmse_deg']:.4f} | "
                     f"{variants['ESO gain 0.8']['position_rmse_deg']:.4f} | "
                     f"{variants['ESO gain 0.8']['torque_limit_fraction']:.1%} | "
                     f"{'yes' if case['torque_infeasible_linear_reference'] else 'no'} |")
    lines += ["", "Metrics evaluate 2–8 s after a C2 smooth startup. Negative `eso_rmse_change_percent` means lower RMSE.",
              "At 5 Hz / 20 deg, the assumed load requires more than the 7 N.m simulation torque cap; controller tuning cannot remove this physical constraint.",
              "7 N.m is a synthetic challenge condition, not an approved hardware setting.", "",
              "`metrics.json` records all configurations, seed, dependency versions, source hashes, and ABI sizes.",
              "`stressed_3hz_20deg.csv.gz` retains aligned full trajectories for both variants; rerun with `--all-csv` for every case.",
              "Each scenario has an editable SVG and PNG containing tracking and actual-position error panels."]
    (args.output / "README.md").write_text("\n".join(lines) + "\n")


if __name__ == "__main__":
    main()
