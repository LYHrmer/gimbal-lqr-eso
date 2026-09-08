#!/usr/bin/env python3
"""Bounded RK4 refinement check for the two final motor profiles (synthetic only).

Fixed verification seed 4320 is separate from development/holdout seeds. This
checks numerical sensitivity of the reported 2 Hz +/-5 deg primary scenario;
it is neither a stability proof nor a motor experiment.
"""
import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import platform
import sys


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    inferred = Path(__file__).resolve().parents[1]
    if not (inferred / "sim").is_dir():
        inferred = Path.cwd()
    parser.add_argument("--repo", type=Path, default=inferred)
    parser.add_argument("--library", type=Path)
    parser.add_argument("--upstream-source", type=Path)
    parser.add_argument("--output", type=Path,
                        help="Default: REPO/results/integration_convergence.json")
    return parser.parse_args()


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    args = arguments()
    root = args.repo.resolve()
    sys.path.insert(0, str(root))
    import numpy as np
    from sim.c_core import Core
    from sim.motor_envelope import MotorEnvelope, actuator_limit
    from sim.run_benchmarks import DT, IX, RAMP
    from sim.run_motor_profiles import WireCodec, make_scenario
    from sim.run_sensitivity import DURATION, run_trial
    from sim.upstream_reference import UpstreamCore

    # Chosen before evaluating this independent seed, not inferred from results.
    seed = 4320
    step_counts = (4, 8)
    scalar_limit_percent = 1.0
    trajectory_limit_deg = 0.01
    scalar_keys = ("position_rmse_deg", "position_max_abs_error_deg",
                   "command_rms_nm", "actual_torque_rms_nm")
    profile_names = ("dm4310_24v.json", "gm6020_current.json")
    source_names = (
        "src/yaw_controller.c", "include/yaw_controller.h", "sim/c_core.py",
        "sim/run_benchmarks.py", "sim/run_sensitivity.py", "sim/run_motor_profiles.py",
        "sim/upstream_reference.py", "sim/motor_envelope.py", "src/dm_mit.c",
        "include/dm_mit.h", "src/gm6020.c", "include/gm6020.h",
    ) + tuple("profiles/" + name for name in profile_names)
    source_hashes = {name: sha256(root / name) for name in source_names}
    original = UpstreamCore(args.upstream_source)
    updated = Core(args.library)
    report = {
        "data_type": "SYNTHETIC; actual C controllers and C command codecs; no motor traffic",
        "scope": "RK4 4-vs-8 substep sensitivity only; quantized/noisy nonlinear trajectories need not exhibit formal fourth-order convergence",
        "fixed_verification_case": {
            "seed": seed, "frequency_hz": 2.0, "amplitude_deg": 5.0,
            "control_dt_s": DT, "duration_s": DURATION, "evaluation_start_s": RAMP,
            "rk4_substeps": list(step_counts),
            "rk4_step_sizes_s": [DT / n for n in step_counts],
            "scalar_relative_difference_limit_percent": scalar_limit_percent,
            "trajectory_max_abs_difference_limit_deg": trajectory_limit_deg,
        },
        "baseline_rule": "Original core applies the profile comparison_baseline_controller_overrides to its controller configuration; New core uses the full controller configuration unchanged. The plant, trajectory and seeded measurement draws match. If these overrides differ, the comparison includes controller tuning, not implementation alone.",
        "numerical_comparison": "Each variant compared against itself with 4 vs 8 RK4 substeps; percentages use the 8-substep metric as denominator. Tracking errors and position differences are measured at matching 1 ms timestamps after startup.",
        "source_sha256": source_hashes,
        "verification_script_sha256": sha256(__file__),
        "shared_library_sha256": sha256(updated.path),
        "upstream": original.metadata,
        "core_abi_sizes_bytes": updated.layout,
        "dependencies": {"python": platform.python_version(), "numpy": np.__version__},
        "profiles": {},
    }
    all_converged = True
    all_improved = True
    for filename in profile_names:
        profile = json.loads((root / "profiles" / filename).read_text())
        if not profile["simulation_only"] or profile["dt_s"] != DT:
            raise ValueError("Only explicit simulation-only 1 ms profiles are supported")
        scenario = make_scenario(profile, 2.0, 5.0)
        motor = profile["motor"]
        rated_point = ({"rated_velocity_rad_s": motor["rated_speed_rad_s"],
                        "rated_torque_nm": motor["rated_torque_nm"]}
                       if profile["protocol"]["kind"] == "dm_mit" else {})
        envelope = MotorEnvelope(scenario.torque_cap_nm, motor["zero_speed_anchor_nm"],
                                 motor["no_load_speed_rad_s"], **rated_point)
        gains = {key: profile["controller"][key] for key in ("k_position", "k_velocity")}
        entry = {"profile": profile, "plant": asdict(scenario.plant),
                 "motor_envelope": asdict(envelope), "variants": {}}
        report["profiles"][profile["name"]] = entry
        for label, core in (("Original core", original), ("New core", updated)):
            config = dict(profile["controller"])
            if label == "Original core":
                overrides = profile["comparison_baseline_controller_overrides"]
                if not set(overrides).issubset(config):
                    raise ValueError("Baseline override contains unknown controller fields")
                config.update(overrides)
            traces, outcomes = {}, {}
            for steps in step_counts:
                codec = WireCodec(profile, updated.lib)
                traces[steps], metrics = run_trial(
                    core, scenario, gains, seed=seed, duration=DURATION, substeps=steps,
                    limiter=lambda torque, velocity: actuator_limit(torque, velocity, envelope),
                    quantizer=codec, controller_config=config)
                metrics["wire_codec"] = codec.metrics()
                outcomes[str(steps)] = metrics
                if not metrics["completed"] or metrics["first_latched_fault"] is not None:
                    raise RuntimeError(f"{profile['name']} {label} substeps={steps} failed: {metrics}")
            coarse, fine = (traces[steps] for steps in step_counts)
            if coarse.shape != fine.shape or not np.array_equal(
                    coarse[:, IX["time_s"]], fine[:, IX["time_s"]]):
                raise RuntimeError("Refinement runs have different sample timelines")
            if not np.array_equal(coarse[:, IX["reference_position_rad"]],
                                  fine[:, IX["reference_position_rad"]]):
                raise RuntimeError("Refinement changed the commanded reference")
            window = fine[:, IX["time_s"]] >= RAMP
            delta_position_deg = np.rad2deg(coarse[window, IX["position_rad"]] -
                                            fine[window, IX["position_rad"]])
            scalar_deltas = {}
            for key in scalar_keys:
                coarse_value, fine_value = outcomes["4"][key], outcomes["8"][key]
                if fine_value <= 0:
                    raise RuntimeError(f"Nonpositive refinement denominator for {key}")
                relative = 100.0 * (coarse_value / fine_value - 1.0)
                scalar_deltas[key] = {"substeps_4": coarse_value, "substeps_8": fine_value,
                    "relative_difference_percent": relative,
                    "absolute_difference": abs(coarse_value - fine_value),
                    "within_limit": abs(relative) <= scalar_limit_percent}
            trajectory = {
                "position_difference_rmse_deg": float(np.sqrt(np.mean(delta_position_deg**2))),
                "position_difference_max_abs_deg": float(np.max(np.abs(delta_position_deg))),
                "velocity_difference_rmse_rad_s": float(np.sqrt(np.mean((
                    coarse[window, IX["velocity_rad_s"]] - fine[window, IX["velocity_rad_s"]])**2))),
                "actual_torque_difference_rmse_nm": float(np.sqrt(np.mean((
                    coarse[window, IX["actuator_torque_nm"]] - fine[window, IX["actuator_torque_nm"]])**2))),
            }
            converged = all(item["within_limit"] for item in scalar_deltas.values()) and (
                trajectory["position_difference_max_abs_deg"] <= trajectory_limit_deg)
            all_converged &= converged
            entry["variants"][label] = {"runs": outcomes, "metric_differences": scalar_deltas,
                "trajectory_differences": trajectory, "meets_predeclared_refinement_limits": converged}
            print(f"{profile['name']} {label}: refinement={'PASS' if converged else 'FAIL'}, "
                  f"RMSE difference={scalar_deltas['position_rmse_deg']['relative_difference_percent']:+.6f}%, "
                  f"max trajectory difference={trajectory['position_difference_max_abs_deg']:.8f} deg", flush=True)
        changes = {}
        for steps in step_counts:
            old = entry["variants"]["Original core"]["runs"][str(steps)]["position_rmse_deg"]
            new = entry["variants"]["New core"]["runs"][str(steps)]["position_rmse_deg"]
            changes[str(steps)] = 100.0 * (new / old - 1.0)
        improvement_preserved = all(change < 0 for change in changes.values())
        all_improved &= improvement_preserved
        entry["new_rmse_change_percent_by_substeps"] = changes
        entry["lower_new_rmse_at_both_resolutions"] = improvement_preserved
        print(f"{profile['name']}: New vs Original RMSE changes {changes}", flush=True)
    # Concurrent edits must not silently associate results with different inputs.
    if source_hashes != {name: sha256(root / name) for name in source_names}:
        raise RuntimeError("Source/profile changed during verification; do not publish this run")
    if report["shared_library_sha256"] != sha256(updated.path):
        raise RuntimeError("Shared library changed during verification; do not publish this run")
    if report["verification_script_sha256"] != sha256(__file__):
        raise RuntimeError("Verification script changed during this run; do not publish it")
    report["all_variants_meet_refinement_limits"] = bool(all_converged)
    report["new_rmse_is_lower_at_both_resolutions_for_both_profiles"] = bool(all_improved)
    report["pass"] = bool(all_converged and all_improved)
    output = args.output or root / "results/integration_convergence.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    print(f"Wrote {output}", flush=True)
    return 0 if report["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
