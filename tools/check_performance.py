"""Check explicit engineering gates on recomputed paired motor simulations.

This validates the synthetic experiment only, not hardware performance or a
population confidence interval. Missing, failed, or regressed trials fail the
gate rather than disappearing from the denominator.
"""
import argparse
import ctypes as ct
import hashlib
import json
import math
from pathlib import Path
from statistics import mean

ROOT = Path(__file__).resolve().parents[1]
SEED_GROUPS = {"primary_paired_seeds": tuple(range(4310, 4315)),
               "validation_paired_seeds": tuple(range(4315, 4320))}
MOTORS = ("dm4310_24v", "gm6020_current_v1_4")
UPSTREAM_COMMIT = "665c5b4ab1067d6cb63122c120822f27502953e5"


def check_report(report, verify_sources=True):
    failures, summaries = [], {}

    def require(condition, message):
        if not condition:
            failures.append(message)

    require(report.get("upstream", {}).get("commit") == UPSTREAM_COMMIT,
            "reference must be the pinned upstream commit")
    require(report.get("upstream", {}).get("source_unmodified") is True,
            "upstream source must be verified unmodified")
    for key, seeds in (("primary_case_preselected", SEED_GROUPS["primary_paired_seeds"]),
                       ("validation_case_preselected", SEED_GROUPS["validation_paired_seeds"])):
        case = report.get(key, {})
        require(case.get("frequency_hz") == 2. and case.get("amplitude_deg") == 5. and
                case.get("paired_seeds") == list(seeds), f"changed experiment contract: {key}")
    require(all(report.get("clock", {}).get(key) == value for key, value in
                (("dt_s", .001), ("duration_s", 6.), ("startup_s", 2.))),
            "changed control period or evaluation window")
    if verify_sources:
        hashes = dict(report.get("source_sha256", {}))
        hashes["src/yaw_controller.c"] = report.get("new_core_source_sha256")
        for name in ("sim/run_motor_profiles.py", "sim/run_sensitivity.py", "sim/motor_envelope.py",
                     "profiles/dm4310_24v.json", "profiles/gm6020_current.json",
                     "src/yaw_controller.c", "src/dm_mit.c", "src/gm6020.c",
                     "include/yaw_controller.h", "sim/c_core.py"):
            require(hashes.get(name) == hashlib.sha256((ROOT/name).read_bytes()).hexdigest(),
                    f"stale or missing source hash: {name}")
    profiles = report.get("profiles", {})
    require(set(profiles) == set(MOTORS), "both expected motor profiles must be present")
    required_metrics = ("position_rmse_deg", "position_max_abs_error_deg", "command_rms_nm",
                        "command_at_limit_fraction", "envelope_limited_interval_fraction")
    for motor in MOTORS:
        entry = profiles.get(motor, {})
        profile = entry.get("profile", {})
        require(profile.get("simulation_only") is True,
                f"{motor}: missing simulation-only scope")
        expected_new = profile.get("controller", {})
        require(bool(expected_new), f"{motor}: missing controller configuration")
        expected_old = dict(expected_new)
        expected_old.update(profile.get("comparison_baseline_controller_overrides", {}))
        expected_configs = {"Original core": expected_old, "New core": expected_new}
        if expected_new.get("k_integral", 0) > 0:
            expected_configs["Original core (same Ki)"] = expected_new
        for group, expected_seeds in SEED_GROUPS.items():
            label = f"{motor}/{group}"
            rows = entry.get(group, [])
            require(len(rows) == len(expected_seeds) and
                    sorted(row.get("seed", -1) for row in rows) == list(expected_seeds),
                    f"{label}: missing, duplicate, or unexpected paired seeds")
            usable = []
            for row in rows:
                pair_valid = True
                variants = row.get("variants", {})
                for variant, expected_config in expected_configs.items():
                    metrics = variants.get(variant, {})
                    valid = metrics.get("completed") is True and all(
                        type(metrics.get(key)) in (int, float) and
                        math.isfinite(metrics[key]) and metrics[key] >= 0
                        for key in required_metrics)
                    valid = valid and metrics.get("actual_c_config") == {
                        key: ct.c_float(value).value for key, value in expected_config.items()}
                    valid = valid and all(metrics.get(key) == value for key, value in
                        (("evaluation_start_s", 2.), ("evaluation_end_s", 6.),
                         ("actual_last_sample_s", 6.)))
                    if variant == "New core":
                        valid = valid and metrics.get("fault_sample_count") == 0 and \
                            metrics.get("first_latched_fault") is None
                    require(valid, f"{label}/seed {row.get('seed')}/{variant}: incomplete, faulted, or invalid")
                    pair_valid = pair_valid and valid
                if pair_valid:
                    old, new = variants["Original core"], variants["New core"]
                    require(old["position_rmse_deg"] > 0, f"{label}: zero baseline RMSE")
                    if old["position_rmse_deg"] > 0:
                        usable.append((old, new))
            if len(usable) != len(expected_seeds):
                continue
            change = [(new["position_rmse_deg"]/old["position_rmse_deg"]-1)*100
                      for old, new in usable]
            means = {variant: {key: mean(pair[index][key] for pair in usable)
                               for key in required_metrics}
                     for index, variant in enumerate(("Original core", "New core"))}
            old, new = means["Original core"], means["New core"]
            summaries[label] = {"mean_paired_rmse_change_percent": mean(change),
                                "improved_pair_count": sum(value < 0 for value in change),
                                "paired_count": len(usable), "means": means}
            require(mean(change) <= -2., f"{label}: mean paired RMSE reduction below 2%")
            require(sum(value < 0 for value in change) >= 4,
                    f"{label}: fewer than 4/5 pairs improve RMSE")
            require(new["position_max_abs_error_deg"] <= old["position_max_abs_error_deg"],
                    f"{label}: mean maximum tracking error increased")
            require(new["command_rms_nm"] <= 1.05*old["command_rms_nm"],
                    f"{label}: mean command RMS increased by more than 5%")
            for key in ("command_at_limit_fraction", "envelope_limited_interval_fraction"):
                require(new[key] <= old[key]+1e-12, f"{label}: {key} increased")
    return {"passed": not failures, "scope": "paired synthetic engineering acceptance; no hardware claim",
            "failures": failures, "summaries": summaries}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", type=Path, default=ROOT/"results/motor_profiles/metrics.json")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = check_report(json.loads(args.report.read_text()))
    rendered = json.dumps(result, indent=2, allow_nan=False)+"\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered)
    print(rendered, end="")
    raise SystemExit(0 if result["passed"] else 1)


if __name__ == "__main__":
    main()
