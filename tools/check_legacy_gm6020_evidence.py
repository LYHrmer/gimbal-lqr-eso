#!/usr/bin/env python3
"""Check archived synthetic GM6020 evidence without requiring the private ZIP.

This checks case coverage, per-seed statistics and provenance declarations. It
does not rerun the archived model or accept controller/hardware performance.
"""
import argparse
import csv
import json
import math
from pathlib import Path
import statistics


EVIDENCE = Path(__file__).resolve().parents[1] / "docs/evidence/gm6020_early_test"
SCRIPT_SHA256 = "192739c681af2595d714f7dc9a74cd1929aac98cf8078222e07912a3fef8044c"
MEAN_METRICS = ("rmse_deg", "max_err_deg", "overshoot_pct", "settle_ms")
LAST_SEED_METRICS = ("limit_pct", "comp_rms_nm", "peak_tau_nm")
CASE_KEYS = {(cutoff, axis, delay, gain) for cutoff in (0.82, 80.0)
             for axis, delay in (("pitch", 1), ("pitch", 2), ("yaw", 1))
             for gain in (0.0, 0.8)}
EXPECTED_PROTOCOL = {
    "dt_s": 0.001, "duration_s": 2.0, "seeds": [1, 2, 3],
    "reference": "0 to 5 degrees linear ramp during 0-0.2 s, then hold",
    "rmse_window_s": [0.8, 2.0], "peak_error_window_s": [0.0, 2.0],
    "settling": "+/-0.25 degree band; time measured from 0.2 s; capped at 1800 ms",
    "disturb_scale": 1.3, "j_scale": 1.0, "b_scale": 1.0,
    "noise_deg": 0.01, "integral_enabled": True,
    "gyro_noise_rad_s": 0.02, "gyro_lpf_q": 1.0, "imu_rate_hz": 500.0,
    "angle_quantum_deg": 0.0219, "actuator_lag_s": 0.002,
}
COMMON_AXIS = {
    "q_position": 400.0, "q_velocity": 1.0, "r_torque": 1.0,
    "torque_limit": 2.2, "torque_slew": 500.0, "eso_bandwidth": 140.0,
    "eso_disturb_limit": 0.8, "eso_comp_limit": 1.0,
    "eso_comp_slew": 100.0, "eso_comp_gain": 0.8,
}
EXPECTED_AXES = {
    "yaw": {**COMMON_AXIS, "name": "yaw", "inertia": 0.0342594124,
            "damping": 0.242238596, "gain_row": 0, "mapping_sign": 1.0,
            "gravity_ff": False, "integral_gain": 10.0, "integral_limit": 0.2,
            "antiwindup_rate": 10.0},
    "pitch": {**COMMON_AXIS, "name": "pitch", "inertia": 0.0201187059,
              "damping": 0.207791924, "gain_row": 1, "mapping_sign": -1.0,
              "gravity_ff": True, "integral_gain": 20.0, "integral_limit": 0.3,
              "antiwindup_rate": 20.0},
}


def require(condition, message):
    if not condition:
        raise ValueError(message)


def finite_number(value):
    return type(value) in (int, float) and math.isfinite(value)


def close(actual, expected, label):
    require(finite_number(actual) and math.isclose(actual, expected, rel_tol=1e-10,
                                                  abs_tol=1e-10),
            f"Inconsistent statistic: {label}")


def check_metrics(metrics):
    require(set(metrics) == set(MEAN_METRICS + LAST_SEED_METRICS + ("diverged",)),
            "Missing or unexpected metrics")
    require(all(finite_number(metrics[key]) and metrics[key] >= 0
                for key in MEAN_METRICS + LAST_SEED_METRICS),
            "Metrics must be nonnegative finite numbers")
    require(metrics["diverged"] is False, "Historical cases must retain their divergence status")
    require(metrics["settle_ms"] <= 1800 and metrics["limit_pct"] <= 100,
            "Metric exceeds observation window or percentage range")


def check_case(row):
    metrics, trials = row["metrics"], row["trials"]
    check_metrics(metrics)
    require(isinstance(trials, list) and [trial.get("seed") for trial in trials] == [1, 2, 3]
            and all(type(trial.get("seed")) is int for trial in trials),
            "Expected exactly seeds 1, 2, 3 in order")
    for trial in trials:
        check_metrics(trial["metrics"])
        require(trial.get("settle_censored") is (trial["metrics"]["settle_ms"] >= 1800),
                "Incorrect per-seed censoring flag")
    for key in MEAN_METRICS:
        close(metrics[key], statistics.mean(trial["metrics"][key] for trial in trials), key)
    # Preserve the original script's aggregation bug as historical evidence.
    for key in LAST_SEED_METRICS:
        close(metrics[key], trials[-1]["metrics"][key], "last-seed " + key)
    censored = sum(trial["settle_censored"] for trial in trials)
    settling = row["settling"]
    require(type(settling.get("censored_seed_count")) is int and
            settling["censored_seed_count"] == censored, "Incorrect censored seed count")
    if censored:
        require(settling["mean_ms_if_all_settled"] is None,
                "A capped mean cannot be reported as demonstrated settling")
    else:
        close(settling["mean_ms_if_all_settled"], metrics["settle_ms"], "settling mean")


def check_report(report):
    require(type(report.get("schema_version")) is int and report["schema_version"] == 1,
            "Unsupported evidence schema")
    require(report["script_sha256"] == SCRIPT_SHA256, "Incorrect archived script hash")
    require(report["scope"] == "Synthetic Python diagnostics, not firmware replay or hardware data.",
            "Historical diagnostics cannot claim hardware or firmware validation")
    protocol = report["protocol"]
    for key, expected in EXPECTED_PROTOCOL.items():
        require(type(protocol.get(key)) is type(expected) and protocol[key] == expected,
                "Changed/missing historical protocol: " + key)
    require(all(type(seed) is int for seed in protocol["seeds"]), "Invalid protocol seeds")
    axes = protocol["axes"]
    require(set(axes) == {"yaw", "pitch"}, "Expected both archived axes")
    for name, expected_axis in EXPECTED_AXES.items():
        require(set(axes[name]) == set(expected_axis), "Changed axis configuration fields")
        for key, expected in expected_axis.items():
            require(type(axes[name].get(key)) is type(expected) and axes[name][key] == expected,
                    "Changed archived model: " + name + "." + key)
    rows = report["rows"]
    require(isinstance(rows, list) and len(rows) == len(CASE_KEYS), "Expected 12 cases")
    indexed = {}
    for row in rows:
        require(type(row["delay_samples"]) is int and
                finite_number(row["gyro_lpf_hz"]) and finite_number(row["comp_gain"]),
                "Invalid case parameters")
        key = (row["gyro_lpf_hz"], row["axis"], row["delay_samples"], row["comp_gain"])
        require(key in CASE_KEYS and key not in indexed, "Unexpected or duplicate case")
        check_case(row)
        indexed[key] = row
    return indexed


def check_evidence(directory=EVIDENCE):
    rows = check_report(json.loads((directory / "step_diagnostics.json").read_text()))
    manifest = json.loads((directory / "source_manifest.json").read_text())
    require(manifest["source_files_sha256"]["tools/pitch_lqr_eso_sim.py"] == SCRIPT_SHA256,
            "Manifest disagrees with the archived script hash")
    verification = manifest["verification"]
    require(verification["hardware_timeseries_provided"] is False and
            verification["hardware_ab_completed"] is False,
            "Historical evidence cannot claim completed hardware validation")
    require(verification["original_simulation_exit_code"] == 1 and
            verification["original_simulation_verdict"] == "FAIL" and
            "总判定: FAIL" in (directory / "simulation_as_received.log").read_text().splitlines(),
            "Preserve the original failed simulation verdict")
    with (directory / "reported_step_metrics.csv").open(newline="") as stream:
        table = list(csv.DictReader(stream))
    require(len(table) == 6, "Expected six user-reported conditions")
    seen = set()
    for entry in table:
        key = (0.82, entry["axis"], int(entry["delay_ms"]), float(entry["comp_gain"]))
        require(key in rows and key not in seen, "Unexpected or duplicate reported condition")
        seen.add(key)
        row = rows[key]
        for column, metric, tolerance in (("rmse_deg", "rmse_deg", 0.001),
                ("peak_error_deg", "max_err_deg", 0.001),
                ("overshoot_percent", "overshoot_pct", 0.051),
                ("settle_ms", "settle_ms", 0.51)):
            value = float(entry[column])
            require(math.isfinite(value) and abs(value - row["metrics"][metric]) <= tolerance,
                    "Reported table differs beyond its precision: " + column)
        count = row["settling"]["censored_seed_count"]
        require(entry["settle_censored"] == ("true" if count == 3 else "false") and
                count in (0, 3), "Reported table conceals censored settling")
    return {"checked_cases": len(rows), "checked_trials": sum(len(r["trials"]) for r in rows.values()),
            "scope": "Stored synthetic evidence consistency only; no model rerun or hardware acceptance"}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--evidence-dir", type=Path, default=EVIDENCE)
    args = parser.parse_args()
    try:
        result = check_evidence(args.evidence_dir)
    except (ValueError, KeyError, TypeError, OSError) as error:
        parser.exit(1, f"Evidence check failed: {error}\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
