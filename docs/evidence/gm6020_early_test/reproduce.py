#!/usr/bin/env python3
"""Recheck the user's archived Python experiment; no firmware or motor access."""
import argparse
from dataclasses import asdict, replace
import hashlib
import importlib.util
import json
from pathlib import Path
import platform
import sys

import numpy as np
import scipy


SCRIPT_SHA256 = "192739c681af2595d714f7dc9a74cd1929aac98cf8078222e07912a3fef8044c"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True,
                        help="Extracted Rudder_Infantry_Gimbal directory")
    parser.add_argument("--output", type=Path,
                        default=Path("build/gm6020-early-test/step-diagnostics.json"))
    args = parser.parse_args()
    script = args.source / "tools/pitch_lqr_eso_sim.py"
    if hashlib.sha256(script.read_bytes()).hexdigest() != SCRIPT_SHA256:
        parser.error("Source hash differs from the received archive; this check is version-specific")

    sys.dont_write_bytecode = True
    spec = importlib.util.spec_from_file_location("archived_gimbal_sim", script)
    sim = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = sim
    spec.loader.exec_module(sim)

    rows = []
    original_cutoff = sim.GYRO_LPF_CUTOFF
    for cutoff in (original_cutoff, 80.0):
        # Only this module variable changes. The source file remains untouched.
        # This is a sensitivity check, not a reconstruction of the firmware.
        sim.GYRO_LPF_CUTOFF = cutoff
        for axis_name, delay in (("pitch", 1), ("pitch", 2), ("yaw", 1)):
            axis = sim.AXES[axis_name]
            for gain in (0.0, 0.8):
                case = sim.Case(
                    name="historical-step-check", gain=sim.axis_gain(axis)[0],
                    ref="step", amplitude_deg=5.0, duration_s=2.0,
                    delay_samples=delay, noise_deg=0.01, disturb_scale=1.3,
                    integral=True, eso_gain=gain, seeds=(1, 2, 3),
                )
                metrics = asdict(sim.run_case(case, axis))
                trials = []
                for seed in case.seeds:
                    trial = asdict(sim.run_case(replace(case, seeds=(seed,)), axis))
                    trials.append({"seed": seed, "metrics": trial,
                                   "settle_censored": trial["settle_ms"] >= 1800.0})
                censored = sum(trial["settle_censored"] for trial in trials)
                rows.append({
                    "gyro_lpf_hz": cutoff, "axis": axis_name,
                    "delay_samples": delay, "comp_gain": gain,
                    "metrics": metrics,
                    "trials": trials,
                    "settling": {
                        "censored_seed_count": censored,
                        "mean_ms_if_all_settled": None if censored else metrics["settle_ms"],
                    },
                })
    sim.GYRO_LPF_CUTOFF = original_cutoff
    if hashlib.sha256(script.read_bytes()).hexdigest() != SCRIPT_SHA256:
        raise RuntimeError("Source changed during the check")

    report = {
        "schema_version": 1,
        "scope": "Synthetic Python diagnostics, not firmware replay or hardware data.",
        "script_sha256": SCRIPT_SHA256,
        "dependencies": {"python": platform.python_version(),
                         "numpy": np.__version__, "scipy": scipy.__version__},
        "protocol": {
            "dt_s": sim.DT, "duration_s": 2.0, "seeds": [1, 2, 3],
            "reference": "0 to 5 degrees linear ramp during 0-0.2 s, then hold",
            "rmse_window_s": [0.8, 2.0], "peak_error_window_s": [0.0, 2.0],
            "settling": "+/-0.25 degree band; time measured from 0.2 s; capped at 1800 ms",
            "disturb_scale": 1.3, "j_scale": 1.0, "b_scale": 1.0,
            "noise_deg": 0.01, "integral_enabled": True,
            "axes": {name: asdict(axis) for name, axis in sim.AXES.items()},
            "gyro_noise_rad_s": sim.GYRO_NOISE_RAD_S,
            "gyro_lpf_q": sim.GYRO_LPF_Q, "imu_rate_hz": sim.IMU_RATE_HZ,
            "angle_quantum_deg": sim.ANGLE_QUANTUM_DEG,
            "actuator_lag_s": sim.ACTUATOR_LAG_S,
        },
        "limitations": [
            "Both variants retain the archived simulation's enabled integral and gravity model.",
            "The Python simulation does not implement the firmware's high-speed observer freeze.",
            "The 80 Hz diagnostic changes only the gyro cutoff and is not a new tuned controller.",
            "RMSE, peak, overshoot and settling are means over seeds; torque diagnostics use only the last seed in the original script.",
            "A settling value of 1800 ms denotes the observation limit, not demonstrated convergence.",
            "If any seed is censored, mean_ms_if_all_settled is null; the legacy capped mean is retained only in metrics.",
        ],
        "rows": rows,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    print(f"Saved {len(rows)} synthetic cases to {args.output}")


if __name__ == "__main__":
    main()
