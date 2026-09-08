#!/usr/bin/env python3
"""SYNTHETIC common-condition comparison of pinned upstream C and new C cores."""
import argparse
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import platform
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import matplotlib
import numpy as np
import scipy

from sim.c_core import Core
from sim.run_benchmarks import (
    DT, DURATION, ESO_GAIN, IX, RAMP, SEED, Plant, config_dict, make_config,
    plot_case, run_case, write_csv,
)
from tools.tune_lqr import design_lqr


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "results/upstream_comparison")
    parser.add_argument("--library", type=Path, default=ROOT / "build/libyaw_controller.so")
    parser.add_argument("--upstream-source", type=Path, default=None)
    parser.add_argument("--upstream-library", type=Path, default=None)
    parser.add_argument("--seed", type=int, default=SEED)
    parser.add_argument("--substeps", type=int, default=5)
    args = parser.parse_args()
    if args.seed < 0 or args.substeps < 1:
        parser.error("seed must be nonnegative and substeps must be positive")
    from sim.upstream_reference import UpstreamCore
    old_core = UpstreamCore(source_dir=args.upstream_source, library=args.upstream_library)
    new_core = Core(args.library)
    design = design_lqr()
    gain = design["gain"]
    nominal = Plant()
    stressed = Plant(inertia_kg_m2=.039*1.25, damping_nm_s_rad=.30*.8,
                     external_torque_nm=.35, coulomb_nm=.12, torque_lag_s=.002,
                     command_delay_samples=1, position_quantum_rad=25./65535.,
                     velocity_quantum_rad_s=60./4095., position_noise_std_rad=.00007,
                     velocity_noise_std_rad_s=.01)
    args.output.mkdir(parents=True, exist_ok=True)
    result = {
        "data_type": "SYNTHETIC: no hardware measurements or original forum CSV",
        "comparison": "pinned upstream C versus new C; same DLQR gains and ESO gain 0.8",
        "scope": [
            "Common-condition software comparison, not original-author hardware reproduction.",
            "A normalized configuration, not a claim to compare each design's best achievable tuning.",
            "Forum /plan_yaw is not the sinusoidal benchmark used here; its original CSV is unavailable.",
            "Shared plant, reference derivatives, seed, DLQR gains, nominal J/B, hard torque and slew limits.",
            "Integral and upstream bias are disabled; unsupported upstream safeguards are not added by the adapter.",
            "Upstream status is unavailable; its fault and warmup counts are null, not evidence of fault detection.",
        ],
        "upstream": old_core.metadata,
        "upstream_adapter_source_sha256": sha256(ROOT / "sim/upstream_reference.py"),
        "upstream_fetch_source_sha256": sha256(ROOT / "tools/fetch_upstream.py"),
        "new_core": {
            "source_sha256": sha256(ROOT / "src/yaw_controller.c"),
            "header_sha256": sha256(ROOT / "include/yaw_controller.h"),
            "shared_library_sha256": sha256(new_core.path),
            "abi_struct_sizes_bytes": new_core.layout,
        },
        "simulation_source_sha256": sha256(ROOT / "sim/run_benchmarks.py"),
        "comparison_source_sha256": sha256(Path(__file__)),
        "clock": {"dt_s": DT, "duration_s": DURATION, "startup_s": RAMP,
                  "feedback_age_s": 0, "reference_age_s": 0,
                  "applied_torque_valid": False,
                  "sample_semantics": "reference and measurement at t; issue command at t; evolve to t+dt"},
        "seed": args.seed, "integration": {"method": "RK4", "substeps": args.substeps},
        "common_requested_config": config_dict(make_config(gain, ESO_GAIN)),
        "lqr_design": design,
        "dependencies": {"python": platform.python_version(), "numpy": np.__version__,
                         "scipy": scipy.__version__, "matplotlib": matplotlib.__version__,
                         "platform": platform.platform()},
        "cases": {},
    }
    for kind, plant in [("nominal", nominal), ("stressed", stressed)]:
        for frequency, amplitude in [(1., 5.), (3., 20.), (5., 20.)]:
            name = f"{kind}_{frequency:g}hz_{amplitude:g}deg"
            omega, amplitude_rad = 2*math.pi*frequency, math.radians(amplitude)
            required = amplitude_rad*math.hypot(plant.inertia_kg_m2*omega**2,
                                                plant.damping_nm_s_rad*omega)
            variants, traces = {}, {}
            for label, core in [("Original core", old_core), ("New core", new_core)]:
                try:
                    trace, metrics = run_case(core, plant, frequency, amplitude, gain,
                                               ESO_GAIN, args.seed, args.substeps)
                    if not np.all(np.isfinite(trace)):
                        raise RuntimeError("controller produced nonfinite trajectory samples")
                except RuntimeError as error:
                    variants[label] = {"completed": False, "error": str(error)}
                    continue
                window = trace[trace[:, IX["time_s"]] >= RAMP]
                metrics["observed_command_at_limit_fraction"] = float(np.mean(
                    np.abs(window[:, IX["command_nm"]]) >= 7. - 1e-6))
                if label == "Original core":
                    # The source has no corresponding safety/fault state machine.
                    metrics["fault_count"] = None
                    metrics["warmup_count"] = None
                    metrics["disturbance_limit_fraction"] = None
                    metrics["compensation_limit_fraction"] = None
                variants[label] = {"completed": True, **metrics}
                traces[label] = trace
            before = variants["Original core"].get("position_rmse_deg")
            after = variants["New core"].get("position_rmse_deg")
            change = (after/before - 1)*100 if before and after is not None else None
            result["cases"][name] = {
                "frequency_hz": frequency, "amplitude_deg": amplitude, "plant": asdict(plant),
                "steady_linear_required_torque_amplitude_nm": required,
                "torque_infeasible_linear_reference": required > 7.,
                "variants": variants, "new_rmse_change_percent": change,
            }
            if before is not None and after is not None:
                change_text = f"{change:+.2f}%" if change is not None else "undefined (zero baseline RMSE)"
                print(f"{name}: original {before:.5f}, new {after:.5f} deg RMSE; "
                      f"change {change_text}" + (" [TORQUE-INFEASIBLE]" if required > 7. else ""),
                      flush=True)
            else:
                print(f"{name}: incomplete comparison: {json.dumps(variants)}", flush=True)
            if name == "stressed_3hz_20deg" and len(traces) == 2:
                plot_case(args.output, name, frequency, amplitude, traces, required > 7.)
                write_csv(args.output / f"{name}.csv.gz", traces)
            # Preserve completed cases if a later external/build failure interrupts the run.
            (args.output / "metrics.json").write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    lines = ["# Original C versus new C — synthetic comparison", "",
             "Both implementations run against the same plant, seed, smooth sinusoidal reference, DLQR gains and ESO gain 0.8. Integral and upstream bias are disabled.",
             "The pinned upstream source is compiled unchanged through an adapter; see `metrics.json` for its commit, hashes, configuration mapping and unavailable features.",
             "This normalizes the controller settings. It does not reproduce the original forum's non-sinusoidal `/plan_yaw` trajectory or establish either controller's best hardware performance.", "",
             "| Scenario | Original RMSE (deg) | New RMSE (deg) | RMSE change | Original / new commands at torque limit |",
             "| --- | ---: | ---: | ---: | ---: |"]
    for name, case in result["cases"].items():
        old, new = case["variants"]["Original core"], case["variants"]["New core"]
        if old["completed"] and new["completed"]:
            change_text = (f"{case['new_rmse_change_percent']:+.2f}%"
                           if case["new_rmse_change_percent"] is not None else "undefined")
            lines.append(f"| {name} | {old['position_rmse_deg']:.5f} | {new['position_rmse_deg']:.5f} | "
                         f"{change_text} | "
                         f"{old['observed_command_at_limit_fraction']:.1%} / {new['observed_command_at_limit_fraction']:.1%} |")
        else:
            lines.append(f"| {name} | {'completed' if old['completed'] else 'failed'} | "
                         f"{'completed' if new['completed'] else 'failed'} | unavailable | unavailable |")
    lines += ["", "Negative RMSE change means the new controller has lower error; positive means higher error. All six cases are retained, including regressions.",
              "Metrics evaluate 2–8 s. The at-limit metric uses the actual command magnitude, so it can be compared without assuming matching flag support in the implementations.",
              "The 5 Hz / 20 deg synthetic cases exceed the assumed 7 N.m limit; this says nothing about the feasibility of the author's different real trajectory and load.",
              "7 N.m is a simulation challenge setting, not a hardware default. Neither the new ESO ablation nor these software comparisons prove a hardware advantage.", "",
              "`stressed_3hz_20deg.png` / `.svg` show representative tracking and true-position error; `.csv.gz` contains both full trajectories.",
              "Reproduce with `python sim/compare_upstream.py` after building the new shared library and obtaining the pinned upstream files as documented in `sim/upstream_reference.py`."]
    (args.output / "README.md").write_text("\n".join(lines) + "\n")
    if any(not variant["completed"] for case in result["cases"].values()
           for variant in case["variants"].values()):
        raise SystemExit("One or more variants failed; partial results were retained in metrics.json.")


if __name__ == "__main__":
    main()
