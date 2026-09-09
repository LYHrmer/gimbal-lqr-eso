#!/usr/bin/env python3
"""Fixed, synthetic pitch experiments: unchanged upstream C and two new C variants.

Run development first, freeze implementation, then run --phase full once for the
preselected held-out seeds. Never alter gains or select seeds inside this script.
"""
import argparse
import csv
from dataclasses import asdict, dataclass, replace
import gzip
import io
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

from sim.c_core import Config, Feedback, Reference
from sim.motor_envelope import MotorEnvelope, actuator_limit
from sim.pitch_core import GimbalConfig, PitchCore, Pose
from sim.run_benchmarks import COLUMNS, DT, IX, RAMP, Plant, config_dict, reference_at
from sim.run_motor_profiles import WireCodec
from sim.run_sensitivity import file_hash
from sim.upstream_reference import UpstreamCore

DEVELOPMENT_SEEDS = (5300, 5301, 5302)
HOLDOUT_SEEDS = (5303, 5304, 5305, 5306, 5307)
LABELS = ("Original C (same Ki)", "New C, gravity off", "New C, gravity on")
DURATION = 6.
PITCH_COLUMNS = COLUMNS + ["wire_torque_nm", "gravity_load_nm", "gravity_feedforward_nm",
    "joint_position_rad", "joint_reference_rad", "integral_nm", "envelope_limited_interval"]
PIX = {name: i for i, name in enumerate(PITCH_COLUMNS)}


@dataclass(frozen=True)
class PitchCase:
    name: str = "primary"
    center_deg: float = 20.
    frequency_hz: float = 1.
    amplitude_deg: float = 5.
    base_tilt_deg: float = 0.
    gravity_factor: float = 1.
    gravity_phase_deg: float = 0.
    model_gravity_factor: float = 1.
    inertia_factor: float = 1.
    noise_factor: float = 1.
    delay_samples: int = 1
    expected_boundary: bool = False


def cases():
    """Predeclared cases; no case is selected or removed based on its result."""
    base = PitchCase()
    return [base,
        replace(base, name="gravity_x0.75", gravity_factor=.75),
        replace(base, name="gravity_x1.25", gravity_factor=1.25),
        replace(base, name="gravity_phase_15deg", gravity_phase_deg=15.),
        replace(base, name="inertia_x0.75", inertia_factor=.75),
        replace(base, name="inertia_x1.25", inertia_factor=1.25),
        replace(base, name="noise_x3", noise_factor=3.),
        replace(base, name="delay_10ms", delay_samples=10),
        replace(base, name="tracking_1.5hz", frequency_hz=1.5),
        replace(base, name="negative_pitch", center_deg=-15.),
        replace(base, name="near_upper_limit", center_deg=43., amplitude_deg=.5),
        replace(base, name="base_tilt_15deg", base_tilt_deg=15.),
        replace(base, name="wrong_gravity_sign", model_gravity_factor=-1.),
        replace(base, name="balanced_load_wrong_model", gravity_factor=0.),
        replace(base, name="high_gravity_boundary", gravity_factor=4., expected_boundary=True),
        replace(base, name="reference_outside_joint_margin", center_deg=43.5,
                amplitude_deg=3., expected_boundary=True)]


def load_profiles():
    result = []
    for name in ("dm4310_24v_pitch.json", "gm6020_current_pitch.json"):
        profile = json.loads((ROOT/"profiles"/name).read_text())
        profile["profile_file"] = name
        profile["motor_base"] = json.loads((ROOT/"profiles"/profile["motor_profile"]).read_text())
        base = profile["motor_base"]
        profile["controller"] = dict(base["controller"], **profile["controller_overrides"])
        plan = profile["preselected_primary"]
        if (not profile["simulation_only"] or not base["simulation_only"] or base["dt_s"] != DT
                or plan["development_seeds"] != list(DEVELOPMENT_SEEDS)
                or plan["holdout_seeds"] != list(HOLDOUT_SEEDS)
                or plan["dt_s"] != DT or plan["duration_s"] != DURATION
                or plan["evaluation_start_s"] != RAMP
                or (plan["center_deg"], plan["frequency_hz"], plan["amplitude_deg"]) != (20., 1., 5.)):
            raise ValueError("pitch experiment no longer matches the preselected contract")
        if set(profile["controller"]) != {key for key, _ in Config._fields_}:
            raise ValueError("pitch controller fields do not match C Config")
        result.append(profile)
    return result


def gravity_load(angle, cos_nm, sin_nm):
    return cos_nm*math.cos(angle)+sin_nm*math.sin(angle)


def pitch_reference(time_s, case):
    p, v, a = reference_at(time_s, case.frequency_hz, math.radians(case.amplitude_deg))
    return p+math.radians(case.center_deg+case.base_tilt_deg), v, a


def physical_parameters(profile, case):
    base, cfg, assumption = profile["motor_base"], profile["controller"], profile["plant_assumptions"]
    plant = Plant(inertia_kg_m2=cfg["inertia_kg_m2"]*case.inertia_factor,
        damping_nm_s_rad=cfg["damping_nm_s_rad"], external_torque_nm=assumption["external_torque_nm"],
        coulomb_nm=assumption["coulomb_nm"], coulomb_velocity_rad_s=assumption["coulomb_velocity_rad_s"],
        torque_lag_s=base["actuator_dynamic_assumptions"]["torque_lag_s"],
        command_delay_samples=case.delay_samples, **base["sensor"])
    plant = replace(plant, position_noise_std_rad=plant.position_noise_std_rad*case.noise_factor,
        velocity_noise_std_rad_s=plant.velocity_noise_std_rad_s*case.noise_factor)
    phase = math.radians(case.gravity_phase_deg)
    ga, gb = assumption["gravity_cos_nm"], assumption["gravity_sin_nm"]
    ga, gb = ((ga*math.cos(phase)+gb*math.sin(phase))*case.gravity_factor,
              (-ga*math.sin(phase)+gb*math.cos(phase))*case.gravity_factor)
    motor = base["motor"]
    rated = ({"rated_velocity_rad_s": motor["rated_speed_rad_s"], "rated_torque_nm": motor["rated_torque_nm"]}
             if base["protocol"]["kind"] == "dm_mit" else {})
    envelope = MotorEnvelope(cfg["torque_limit_nm"], motor["zero_speed_anchor_nm"],
        motor["no_load_speed_rad_s"], **rated)
    return plant, ga, gb, envelope


def run_trial(profile, case, label, seed, original, new, duration=DURATION, substeps=4):
    if label not in LABELS or substeps < 1:
        raise ValueError("unknown implementation or invalid RK4 substeps")
    cfg = Config(**profile["controller"])
    plant, ga, gb, envelope = physical_parameters(profile, case)
    boundaries = profile["plant_assumptions"]
    minimum, maximum = boundaries["physical_min_rad"], boundaries["physical_max_rad"]
    gravity = profile["gravity_model"]
    factor = case.model_gravity_factor if label == LABELS[2] else 0.
    gcfg = GimbalConfig(cfg, gravity["cos_nm"]*factor, gravity["sin_nm"]*factor,
        minimum, maximum, profile["joint_margin_rad"], True)
    controller = original.init(cfg) if label == LABELS[0] else new.init(gcfg)
    codec = WireCodec(profile["motor_base"], new.lib)
    rng = np.random.default_rng(seed)
    data = np.zeros((round(duration/DT)+1, len(PITCH_COLUMNS)))
    base_angle = math.radians(case.base_tilt_deg)
    p, v, torque = math.radians(case.center_deg)+base_angle, 0., 0.
    pending = [0.]*plant.command_delay_samples
    first_fault = first_crossing = failure = None
    actual_count = 0
    for index in range(len(data)):
        time_s = index*DT
        rp, rv, ra = pitch_reference(time_s, case)
        noise = rng.normal(size=2)
        # Assumed calibrated joint encoder + exactly known constant chassis tilt.
        # This is neither a raw feedback CAN replay nor an IMU estimation model.
        mp = p-base_angle+noise[0]*plant.position_noise_std_rad
        mv = v+noise[1]*plant.velocity_noise_std_rad_s
        if plant.position_quantum_rad:
            mp = float(np.round(mp/plant.position_quantum_rad)*plant.position_quantum_rad)
        if plant.velocity_quantum_rad_s:
            mv = float(np.round(mv/plant.velocity_quantum_rad_s)*plant.velocity_quantum_rad_s)
        measured_joint = mp
        mp += base_angle
        feedback, reference = Feedback(mp, mv, 0., 0., True, False), Reference(rp, rv, ra, 0., True)
        if label == LABELS[0]:
            out = original.step(controller, feedback, reference, DT)
            gravity_ff, joint_ref = 0., rp-base_angle
        else:
            gout = new.step(controller, feedback, reference, Pose(measured_joint, mv, mp, 0., True), DT)
            out, gravity_ff, joint_ref = gout.control, gout.gravity_feedforward_nm, gout.joint_reference_rad
        if not all(math.isfinite(value) for value in (out.torque_nm, out.unconstrained_torque_nm,
                out.compensation_nm, out.disturbance_nm, gravity_ff)):
            failure = f"nonfinite C controller output at {time_s:.6f} s"
            break
        if out.status > 1 and first_fault is None:
            first_fault = {"time_s": time_s, "status": int(out.status)}
        sent = codec(out.torque_nm)
        data[index] = [time_s, rp, rv, ra, p, v, mp, mv, out.torque_nm, torque, rp-p,
            out.compensation_nm, out.disturbance_nm, out.unconstrained_torque_nm,
            out.flags, out.status, sent, gravity_load(p, ga, gb), gravity_ff,
            p-base_angle, joint_ref, out.integral_nm, 0.]
        actual_count = index+1
        if index+1 == len(data):
            break
        pending.append(sent)
        command = pending.pop(0)
        clipped = False

        def derivative(px, vx, tx):
            nonlocal clipped
            target = actuator_limit(command, vx, envelope)
            clipped |= abs(target-command) > 1e-9
            used = tx if plant.torque_lag_s > 0 else target
            acceleration = (used+plant.external_torque_nm-gravity_load(px, ga, gb)
                -plant.damping_nm_s_rad*vx-plant.coulomb_nm*math.tanh(vx/plant.coulomb_velocity_rad_s)) / plant.inertia_kg_m2
            return vx, acceleration, (target-tx)/plant.torque_lag_s if plant.torque_lag_s > 0 else 0.

        h = DT/substeps
        for step in range(substeps):
            k1 = derivative(p, v, torque)
            k2 = derivative(p+h*k1[0]/2, v+h*k1[1]/2, torque+h*k1[2]/2)
            k3 = derivative(p+h*k2[0]/2, v+h*k2[1]/2, torque+h*k2[2]/2)
            k4 = derivative(p+h*k3[0], v+h*k3[1], torque+h*k3[2])
            p += h*(k1[0]+2*k2[0]+2*k3[0]+k4[0])/6
            v += h*(k1[1]+2*k2[1]+2*k3[1]+k4[1])/6
            torque += h*(k1[2]+2*k2[2]+2*k3[2]+k4[2])/6
            if plant.torque_lag_s == 0:
                torque = actuator_limit(command, v, envelope)
            if not all(math.isfinite(value) for value in (p, v, torque)):
                failure = f"nonfinite plant state after {time_s:.6f} s"
                break
            if p-base_angle < minimum or p-base_angle > maximum:
                first_crossing = {"time_s": time_s+(step+1)*h,
                    "joint_position_rad": p-base_angle, "velocity_rad_s": v}
                failure = "physical joint boundary crossed; no position clamping/contact model"
                break
        data[index, PIX["envelope_limited_interval"]] = clipped
        if failure:
            break
    data = data[:actual_count]
    window = data[data[:, IX["time_s"]] >= RAMP]
    intervals = window[window[:, IX["time_s"]] < duration]
    metrics = {"completed": failure is None and actual_count == round(duration/DT)+1,
        "failure": failure, "first_latched_fault": first_fault,
        "first_physical_limit_crossing": first_crossing,
        "evaluation_start_s": RAMP, "evaluation_end_s": duration,
        "actual_last_sample_s": float(data[-1, 0]) if len(data) else None,
        "actual_c_config": config_dict(cfg),
        "actual_c_gravity_model": {"cos_nm": float(gcfg.gravity_cos_nm), "sin_nm": float(gcfg.gravity_sin_nm)}
            if label != LABELS[0] else None,
        "actual_plant": dict(asdict(plant), gravity_cos_nm=ga, gravity_sin_nm=gb),
        "wire_codec": codec.metrics(),
        "fault_sample_count": int(np.count_nonzero(data[:, IX["status"]] > 1)) if label != LABELS[0] else None,
        "startup_peak_error_deg": float(np.max(np.abs(np.rad2deg(data[data[:, 0] < RAMP, IX["error_rad"]]))))
            if len(data) else None}
    if len(window):
        error = np.rad2deg(window[:, IX["error_rad"]])
        command = window[:, IX["command_nm"]]
        metrics.update(position_rmse_deg=float(np.sqrt(np.mean(error**2))),
            position_max_abs_error_deg=float(np.max(np.abs(error))),
            command_rms_nm=float(np.sqrt(np.mean(command**2))),
            actual_torque_rms_nm=float(np.sqrt(np.mean(window[:, IX["actuator_torque_nm"]]**2))),
            command_peak_abs_nm=float(np.max(np.abs(command))),
            command_at_limit_fraction=float(np.mean(np.abs(command) >= cfg.torque_limit_nm-1e-6)),
            controller_slew_limit_fraction=float(np.mean((window[:, IX["flags"]].astype(np.uint32)&2) != 0)),
            envelope_limited_interval_fraction=float(np.mean(intervals[:, PIX["envelope_limited_interval"]])) if len(intervals) else None,
            integral_mean_nm=float(np.mean(window[:, PIX["integral_nm"]])),
            disturbance_mean_nm=float(np.mean(window[:, IX["disturbance_nm"]])),
            minimum_joint_clearance_deg=float(np.rad2deg(np.min(np.minimum(
                window[:, PIX["joint_position_rad"]]-minimum, maximum-window[:, PIX["joint_position_rad"]])))))
    return data, metrics


def compare(profile, case, seed, original, new, substeps=4):
    traces, metrics = {}, {}
    for label in LABELS:
        traces[label], metrics[label] = run_trial(profile, case, label, seed, original, new, substeps=substeps)
    result = {"seed": seed, "case": asdict(case), "variants": metrics}
    for label, field in ((LABELS[0], "change_vs_upstream_percent"), (LABELS[1], "change_vs_gravity_off_percent")):
        old, current = metrics[label], metrics[LABELS[2]]
        result[field] = (100*(current["position_rmse_deg"]/old["position_rmse_deg"]-1)
            if old["completed"] and current["completed"] and old.get("position_rmse_deg", 0) > 0
            and not current["first_latched_fault"] and not old["first_latched_fault"] else None)
    return traces, result


def summarize(trials):
    summary = {}
    for field in ("change_vs_upstream_percent", "change_vs_gravity_off_percent"):
        valid = [trial[field] for trial in trials if trial[field] is not None]
        summary[field] = {"completed_pair_count": len(valid), "planned_pair_count": len(trials),
            "mean_paired_percent": float(np.mean(valid)) if len(valid) == len(trials) and valid else None,
            "improved_pair_count": sum(value < 0 for value in valid),
            "worst_paired_percent": max(valid) if valid else None}
    for label in LABELS:
        values = [trial["variants"][label] for trial in trials]
        summary[label] = {key: float(np.mean([entry[key] for entry in values]))
            if values and all(entry.get(key) is not None for entry in values) else None
            for key in ("position_rmse_deg", "position_max_abs_error_deg", "command_rms_nm",
                        "startup_peak_error_deg", "command_at_limit_fraction")}
    return summary


def assess_group(profile, trials, seeds):
    """A fixed performance contract, separate from retaining negative stress cases."""
    thresholds = profile["preselected_success_criteria"]
    failures = []
    if [row["seed"] for row in trials] != list(seeds):
        failures.append("planned seeds incomplete or reordered")
    for row in trials:
        if row["case"] != asdict(PitchCase()):
            failures.append("primary case was changed")
        for label in LABELS:
            metric = row["variants"][label]
            if not metric["completed"] or metric["first_latched_fault"] or metric["first_physical_limit_crossing"]:
                failures.append(f"seed {row['seed']} {label}: incomplete, faulted, or out of physical range")
            if metric["actual_c_config"] != config_dict(Config(**profile["controller"])):
                failures.append("actual C gains/configuration differ across comparison")
            for key in ("position_rmse_deg", "position_max_abs_error_deg", "command_rms_nm"):
                value = metric.get(key)
                if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0:
                    failures.append(f"invalid {key}")
    if failures:
        return {"passed": False, "failures": failures, "thresholds": thresholds}
    # Recompute percentages from actual metrics rather than trusting cached
    # presentation fields. A claimed improvement must not override worse data.
    checked = []
    for row in trials:
        current_rmse = row["variants"][LABELS[2]]["position_rmse_deg"]
        verified = dict(row)
        for label, field in ((LABELS[0], "change_vs_upstream_percent"), (LABELS[1], "change_vs_gravity_off_percent")):
            baseline_rmse = row["variants"][label]["position_rmse_deg"]
            verified[field] = 100*(current_rmse/baseline_rmse-1) if baseline_rmse > 0 else None
        checked.append(verified)
    summary = summarize(checked)
    current = summary[LABELS[2]]
    for label, field in ((LABELS[0], "change_vs_upstream_percent"), (LABELS[1], "change_vs_gravity_off_percent")):
        pair = summary[field]
        change = pair["mean_paired_percent"]
        if change is None or change > -thresholds["minimum_mean_paired_rmse_improvement_percent"]:
            failures.append(f"{label}: mean paired RMSE improvement below fixed threshold")
        required_wins = thresholds["minimum_improved_holdout_seeds"] if len(seeds) == 5 else len(seeds)
        if pair["improved_pair_count"] < required_wins:
            failures.append(f"{label}: insufficient improved seeds")
        old = summary[label]
        for key, ratio in (("position_max_abs_error_deg", 1.),
                ("command_rms_nm", thresholds["maximum_mean_command_rms_ratio"])):
            if current[key] is None or old[key] is None or current[key] > ratio*old[key]:
                failures.append(f"{label}: mean {key} exceeds comparison limit")
    return {"passed": not failures, "failures": failures, "thresholds": thresholds}


def integration_convergence(profile, original, new):
    """Independent extra seed, halved RK4 step; never used for tuning."""
    rows = {}
    for label in LABELS:
        coarse, cm = run_trial(profile, PitchCase(), label, 5308, original, new, substeps=4)
        fine, fm = run_trial(profile, PitchCase(), label, 5308, original, new, substeps=8)
        changes = {key: 100*abs(cm[key]-fm[key])/max(abs(fm[key]), 1e-12)
            for key in ("position_rmse_deg", "position_max_abs_error_deg", "command_rms_nm", "actual_torque_rms_nm")}
        trajectory_difference = float(np.max(np.abs(np.rad2deg(coarse[:, IX["position_rad"]]-fine[:, IX["position_rad"]]))))
        rows[label] = {"four_substeps": cm, "eight_substeps": fm,
            "absolute_relative_metric_difference_percent": changes,
            "maximum_position_trace_difference_deg": trajectory_difference,
            "passed": cm["completed"] and fm["completed"] and not cm["first_latched_fault"]
                and not fm["first_latched_fault"] and max(changes.values()) <= 1. and trajectory_difference <= .01}
    return {"seed": 5308, "selected_before_run": True,
        "thresholds": {"maximum_metric_difference_percent": 1., "maximum_position_difference_deg": .01},
        "variants": rows, "passed": all(row["passed"] for row in rows.values()),
        "scope": "Numerical step refinement, not hardware validation or formal convergence order for a quantized nonlinear loop."}


def write_trace(path, traces):
    with path.open("wb") as raw:
        with gzip.GzipFile(fileobj=raw, mode="wb", filename="", mtime=0) as zipped:
            with io.TextIOWrapper(zipped, encoding="utf-8", newline="") as stream:
                writer = csv.writer(stream)
                writer.writerow(["variant"]+PITCH_COLUMNS)
                for label, data in traces.items():
                    writer.writerows([label]+[format(float(x), ".9g") for x in row] for row in data)


def plot_trace(path, traces, case):
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True, constrained_layout=True)
    first = traces[LABELS[0]]
    axes[0].plot(first[:, 0], np.rad2deg(first[:, IX["reference_position_rad"]]), "k--", label="Reference")
    for label, color in zip(LABELS, ("#b66e28", "#777d85", "#176baf")):
        data = traces[label]
        axes[0].plot(data[:, 0], np.rad2deg(data[:, IX["position_rad"]]), color=color, label=label)
        axes[1].plot(data[:, 0], np.rad2deg(data[:, IX["error_rad"]]), color=color)
        axes[2].plot(data[:, 0], data[:, IX["command_nm"]], color=color)
    for ax in axes:
        ax.axvspan(0, RAMP, alpha=.15, color="gray")
        ax.grid(alpha=.18)
    axes[0].set_ylabel("World pitch (deg)")
    axes[1].set_ylabel("Tracking error (deg)")
    axes[2].set_ylabel("Total command (N m)")
    axes[2].set_xlabel("Time (s); shaded interval = startup")
    axes[0].legend(fontsize=8, ncol=2)
    axes[0].set_title(f"SYNTHETIC pitch | {case.name} | same gains including Ki\nGravity, current/torque quantization, actuator lag and command delay", loc="left")
    fig.savefig(path, dpi=160, metadata={"Software": "robomaster-gimbal synthetic pitch simulation"})
    plt.close(fig)


def provenance(new):
    names = ["sim/run_pitch_profiles.py", "sim/pitch_core.py", "sim/run_benchmarks.py",
        "sim/run_motor_profiles.py", "sim/motor_envelope.py", "sim/c_core.py", "sim/upstream_reference.py",
        "src/yaw_controller.c", "include/yaw_controller.h", "src/gimbal_controller.c", "include/gimbal_controller.h",
        "src/dm_mit.c", "src/gm6020.c", "include/dm_mit.h", "include/gm6020.h",
        "tools/fetch_upstream.py", "profiles/dm4310_24v.json", "profiles/gm6020_current.json",
        "profiles/dm4310_24v_pitch.json", "profiles/gm6020_current_pitch.json"]
    internal = ROOT/"src/yaw_controller_internal.h"
    if internal.exists():
        names.append("src/yaw_controller_internal.h")
    return {"source_sha256": {name: file_hash(ROOT/name) for name in names},
            "shared_library_sha256": file_hash(new.path)}


def report_markdown(report):
    lines = ["# Synthetic pitch validation", "",
        "Actual C controllers and actual C motor command codecs. All three variants have identical gains, including Ki; GM6020 is not compared against a substituted Ki=0 baseline.",
        "The fixed primary is +20 deg centre, 1 Hz / 5 deg amplitude, gravity 0.45*cos(theta)+0.10*sin(theta) N.m, 2 ms actuator lag + 1 ms command delay. J/B and gravity are illustrative assumptions, not identified RoboMaster parameters.",
        "Development seeds 5300-5302; held-out seeds 5303-5307. Startup 0-2 s is reported separately; primary error uses 2-6 s. Physical boundary crossings are failures, never clipped trajectories.", "",
        "| Motor / split | Original same-Ki RMSE (deg) | New gravity off | New gravity on | Mean paired change vs original | Vs gravity off | Fixed acceptance |",
        "| --- | ---: | ---: | ---: | ---: | ---: | --- |"]
    def number(value):
        return f"{value:.6f}" if value is not None else "incomplete"
    for name, entry in report["profiles"].items():
        for split in ("development", "holdout"):
            if split not in entry:
                continue
            summary = entry[split+"_summary"]
            lines.append(f"| {name} / {split} | "+" | ".join(number(summary[label]["position_rmse_deg"]) for label in LABELS)+
                " | "+" | ".join(number(summary[field]["mean_paired_percent"])+"%" for field in
                ("change_vs_upstream_percent", "change_vs_gravity_off_percent"))+f" | {entry[split+'_acceptance']['passed']} |")
        lines += ["", f"## {name}: every preselected stress case (seed 5300)", "",
            "| Case | Original / off / on RMSE (deg) | Change vs original | Fault or boundary failure |",
            "| --- | ---: | ---: | --- |"]
        for row in entry["cases"]:
            issues = [label+": "+str(value["failure"] or value["first_latched_fault"])
                      for label, value in row["variants"].items() if value["failure"] or value["first_latched_fault"]]
            lines.append("| "+row["case"]["name"]+" | "+" / ".join(number(row["variants"][label].get("position_rmse_deg")) for label in LABELS)+
                " | "+number(row["change_vs_upstream_percent"])+"% | "+("; ".join(issues) or "none")+" |")
        for split in ("development", "holdout"):
            if split in entry and not entry[split+"_acceptance"]["passed"]:
                lines += ["", split+" acceptance failures: "+"; ".join(entry[split+"_acceptance"]["failures"])]
        if "integration_convergence" in entry:
            lines += ["", "Independent seed 5308, RK4 4 vs 8 substeps: "+str(entry["integration_convergence"]["passed"])+
                " (all four scalar metrics within 1%, complete position traces within 0.01 deg)."]
    lines += ["", "Negative percentage means lower RMSE. A passed primary gate does not establish improvement in every stress case or on hardware.",
        "The gravity-off/new comparison isolates the gravity feature; the upstream comparison also includes core implementation differences. Gravity must use a calibrated physical angle. Wrong sign, load mismatch, long delay and inadequate torque remain in the results.",
        "An assumed fixed chassis tilt is known exactly; joint encoder noise/quantization is transformed into world angle. No dynamic IMU fusion, coupled yaw-pitch motion, flexible transmission, hard-stop contact, thermal or regeneration model is included.",
        "No-feedback or out-of-range software zero is not a brake: unsupported pitch can fall. Boundary traces stop when the physical range is first crossed."]
    return "\n".join(lines)+"\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--phase", choices=("development", "full"), default="full")
    parser.add_argument("--output", type=Path, default=ROOT/"results/pitch")
    parser.add_argument("--upstream-source", type=Path)
    parser.add_argument("--require-primary-improvement", action="store_true")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    profiles, new, original = load_profiles(), PitchCore(), UpstreamCore(args.upstream_source)
    initial = provenance(new)
    report = {"data_type": "SYNTHETIC; actual C and command codecs; no hardware measurements",
        "phase": args.phase, "clock": {"dt_s": DT, "duration_s": DURATION, "startup_s": RAMP, "rk4_substeps": 4},
        "development_seeds": list(DEVELOPMENT_SEEDS), "holdout_seeds": list(HOLDOUT_SEEDS),
        "preselected_cases": [asdict(case) for case in cases()], "upstream": original.metadata,
        "abi_sizes_bytes": new.layout, "provenance": initial,
        "dependencies": {"python": platform.python_version(), "numpy": np.__version__, "matplotlib": matplotlib.__version__},
        "profiles": {}}
    def save():
        (args.output/"metrics.json").write_text(json.dumps(report, indent=2, allow_nan=False)+"\n")
    for profile in profiles:
        name = profile["name"]
        entry = {"profile": profile, "cases": [], "development": []}
        report["profiles"][name] = entry
        for case in cases():
            traces, row = compare(profile, case, DEVELOPMENT_SEEDS[0], original, new)
            entry["cases"].append(row)
            print(f"{name}/{case.name}: "+", ".join(f"{label}={row['variants'][label].get('position_rmse_deg')}" for label in LABELS), flush=True)
            if case.name == "primary":
                entry["development"].append(row)
                write_trace(args.output/f"{name}_primary.csv.gz", traces)
                plot_trace(args.output/f"{name}_primary.png", traces, case)
            elif case.expected_boundary:
                write_trace(args.output/f"{name}_{case.name}.csv.gz", traces)
            save()
        for seed in DEVELOPMENT_SEEDS[1:]:
            _, row = compare(profile, PitchCase(), seed, original, new)
            entry["development"].append(row)
        entry["development_summary"] = summarize(entry["development"])
        entry["development_acceptance"] = assess_group(profile, entry["development"], DEVELOPMENT_SEEDS)
        if args.phase == "full":
            entry["holdout"] = []
            for seed in HOLDOUT_SEEDS:
                _, row = compare(profile, PitchCase(), seed, original, new)
                entry["holdout"].append(row)
                print(f"{name}/held-out {seed}: vs upstream {row['change_vs_upstream_percent']}; vs off {row['change_vs_gravity_off_percent']}", flush=True)
            entry["holdout_summary"] = summarize(entry["holdout"])
            entry["holdout_acceptance"] = assess_group(profile, entry["holdout"], HOLDOUT_SEEDS)
            entry["integration_convergence"] = integration_convergence(profile, original, new)
        save()
    if provenance(new) != initial:
        raise RuntimeError("controller/simulation sources changed during the experiment")
    report["source_unchanged_during_run"] = True
    report["all_primary_acceptance_passed"] = all(entry[key]["passed"] for entry in report["profiles"].values()
        for key in ("development_acceptance", "holdout_acceptance") if key in entry)
    save()
    (args.output/"README.md").write_text(report_markdown(report))
    if args.phase == "full" and not all(entry["integration_convergence"]["passed"] for entry in report["profiles"].values()):
        raise SystemExit("Pitch integration refinement check failed; complete results retained")
    if args.require_primary_improvement and (args.phase != "full" or not report["all_primary_acceptance_passed"]):
        raise SystemExit("Fixed primary improvement gate failed; complete results retained")


if __name__ == "__main__":
    main()
