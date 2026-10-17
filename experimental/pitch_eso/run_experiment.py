#!/usr/bin/env python3
"""Fixed Pitch ESO grid with protocol-preserving CAN timing and gated holdout."""
import argparse
from copy import deepcopy
import ctypes as ct
from dataclasses import asdict
from itertools import product
import hashlib
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from sim.c_core import Config
from sim.feedback_timing import ChannelTiming
from sim.pitch_core import PitchCore
from sim.run_benchmarks import config_dict
from sim.run_pitch_profiles import (DT, DURATION, RAMP, LABELS, PitchCase,
    load_profiles, physical_parameters, provenance, run_trial, write_trace)
from sim.run_sensitivity import file_hash
from tools.check_motor_profiles import validate_profile

PLAN = Path(__file__).with_name("plan.json")
METRICS = ("position_rmse_deg", "position_max_abs_error_deg", "startup_peak_error_deg", "command_rms_nm")


def candidate_id(candidate):
    return f"w{candidate['eso_bandwidth_rad_s']:g}_g{candidate['eso_gain']:g}"


def candidates(plan):
    return [{"eso_bandwidth_rad_s": w, "eso_gain": g}
            for w, g in product(plan["eso_bandwidth_rad_s"], plan["eso_gain"])]


def case_inputs(entry):
    clock = ChannelTiming(**entry["can_clock"])
    # Angle and RPM belong to the SAME GM6020 feedback packet. Never introduce
    # an independent gyro clock while claiming native motor-CAN feedback.
    if clock.period_samples != 1 or clock.phase_samples != 0:
        raise ValueError("GM6020 native feedback producer remains 1 kHz")
    return PitchCase(name=entry["name"], **entry["pitch"]), (clock, clock)


def load_plan():
    plan = json.loads(PLAN.read_text())
    if (plan["simulation_only"] is not True or plan["profile"] != "gm6020_current_pitch.json"
            or plan["criteria"]["complete_unfaulted_runs_required"] is not True):
        raise ValueError("unsupported experiment scope")
    groups = [plan[key] for key in ("development_seeds", "holdout_seeds")]
    if any(not seeds or len(set(seeds)) != len(seeds) or
           any(type(s) is not int or s < 0 for s in seeds) for seeds in groups) or set(groups[0]) & set(groups[1]):
        raise ValueError("seed groups must be nonempty, unique and disjoint")
    names = [entry["name"] for entry in plan["cases"]]
    if len(names) != len(set(names)) or plan["primary_case"] not in names:
        raise ValueError("invalid planned cases")
    for entry in plan["cases"]:
        case_inputs(entry)
    grid = candidates(plan)
    if not grid or len({candidate_id(c) for c in grid}) != len(grid) or plan["baseline"] not in grid:
        raise ValueError("unique grid must include the baseline")
    for c in grid:
        if any(type(v) not in (int, float) or not math.isfinite(v) for v in c.values()) or not (
                c["eso_bandwidth_rad_s"] > 0 and 0 <= c["eso_gain"] <= 1):
            raise ValueError("invalid ESO candidate")
    for key, value in plan["criteria"].items():
        if key == "complete_unfaulted_runs_required":
            continue
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            raise ValueError("criteria must contain finite positive thresholds")
        if key.endswith("win_fraction") and value > 1:
            raise ValueError("win fraction must not exceed one")
    return plan


def assess(rows, baseline_rows, plan, seeds, profile, candidate, holdout=False):
    """Recompute comparisons; missing runs or invalid metrics cannot pass."""
    failures, summaries = [], {}
    expected = [(case["name"], seed) for case in plan["cases"] for seed in seeds]
    expected_models = {}
    for entry in plan["cases"]:
        case, _ = case_inputs(entry)
        plant, ga, gb, _ = physical_parameters(profile, case)
        expected_models[entry["name"]] = (
            dict(asdict(plant), gravity_cos_nm=ga, gravity_sin_nm=gb),
            {key: ct.c_float(value*case.model_gravity_factor).value
             for key, value in profile["gravity_model"].items()})
    for data, parameters in ((rows, candidate), (baseline_rows, plan["baseline"])):
        if [(r.get("case"), r.get("seed")) for r in data] != expected:
            failures.append("missing, duplicate or reordered planned case/seed")
            continue
        expected_config = config_dict(Config(**dict(profile["controller"], **parameters)))
        for row in data:
            m = row["metrics"]
            if (m.get("completed") is not True or "failure" not in m or m["failure"] is not None
                    or "first_latched_fault" not in m or m["first_latched_fault"] is not None
                    or "first_physical_limit_crossing" not in m or m["first_physical_limit_crossing"] is not None):
                failures.append(f"{row['case']}/{row['seed']}: incomplete or faulted")
            if m.get("actual_c_config") != expected_config:
                failures.append("actual C configuration differs from the frozen candidate")
            if (m.get("actual_last_sample_s") != DURATION or m.get("evaluation_start_s") != RAMP
                    or m.get("evaluation_end_s") != DURATION):
                failures.append("incomplete or different evaluation window")
            plant, gravity = expected_models[row["case"]]
            if m.get("actual_plant") != plant:
                failures.append("physical/measurement model differs from the frozen case/profile")
            if m.get("actual_c_gravity_model") != gravity:
                failures.append("gravity model differs from the frozen case/profile")
            for key in METRICS:
                value = m.get(key)
                if type(value) not in (int, float) or not math.isfinite(value) or value < 0:
                    failures.append(f"invalid {key}")
    if failures:
        return {"passed": False, "failures": sorted(set(failures)), "cases": summaries}
    for entry in plan["cases"]:
        name = entry["name"]
        pairs = [(r["metrics"], b["metrics"]) for r, b in zip(rows, baseline_rows) if r["case"] == name]
        expected_clock = json.loads(json.dumps(asdict(case_inputs(entry)[1][0])))
        for current, baseline in pairs:
            for metric in (current, baseline):
                timing = metric.get("feedback_timing", {})
                for channel in ("position", "velocity"):
                    if json.loads(json.dumps(timing.get(channel))) != expected_clock:
                        failures.append(f"{name}: angle/RPM do not share the planned CAN clock")
        if any(b[key] <= 0 for _, b in pairs for key in METRICS):
            failures.append(f"{name}: zero baseline metric cannot define relative improvement")
            continue
        summary = {key: {"baseline_mean": sum(b[key] for _, b in pairs)/len(pairs),
                         "candidate_mean": sum(c[key] for c, _ in pairs)/len(pairs),
                         "mean_paired_change_percent": sum(100*(c[key]/b[key]-1) for c, b in pairs)/len(pairs)}
                   for key in METRICS}
        summary["improved_rmse_seeds"] = sum(c[METRICS[0]] < b[METRICS[0]] for c, b in pairs)
        summaries[name] = summary
        criteria = plan["criteria"]
        for key in METRICS:
            ratio = (criteria["maximum_primary_mean_peak_ratio"] if name == plan["primary_case"]
                     and key == "position_max_abs_error_deg" else criteria["maximum_other_mean_metric_ratio"])
            if summary[key]["candidate_mean"] > ratio*summary[key]["baseline_mean"]:
                failures.append(f"{name}: mean {key} exceeds ratio {ratio:g}")
        if name == plan["primary_case"]:
            if summary[METRICS[0]]["mean_paired_change_percent"] > -criteria["minimum_primary_mean_paired_rmse_improvement_percent"]:
                failures.append(f"{name}: insufficient mean paired RMSE improvement")
            fraction = criteria["minimum_holdout_primary_win_fraction" if holdout else "minimum_development_primary_win_fraction"]
            if summary["improved_rmse_seeds"] < math.ceil(fraction*len(seeds)):
                failures.append(f"{name}: insufficient improved seeds")
    return {"passed": not failures, "failures": sorted(set(failures)), "cases": summaries}


def select_candidate(entries, plan):
    eligible = [key for key, entry in entries.items() if entry["assessment"]["passed"]
                and key != candidate_id(plan["baseline"])]
    def score(key):
        summary = entries[key]["assessment"]["cases"]
        changes = [s[METRICS[0]]["mean_paired_change_percent"] for s in summary.values()]
        return max(changes), summary[plan["primary_case"]][METRICS[0]]["mean_paired_change_percent"], key
    return min(eligible, key=score) if eligible else None


def run_group(core, profile, plan, grid, seeds, output, record_trace=False):
    result = {}
    for candidate in grid:
        key = candidate_id(candidate)
        p = deepcopy(profile)
        p["controller"].update(candidate)
        rows = []
        for entry in plan["cases"]:
            case, clocks = case_inputs(entry)
            for seed in seeds:
                trace, metric = run_trial(p, case, LABELS[2], seed, None, core, feedback_clocks=clocks)
                rows.append({"case": entry["name"], "seed": seed, "metrics": metric})
                if record_trace and candidate == plan["baseline"] and entry["name"] == plan["primary_case"] and seed == seeds[0]:
                    write_trace(output/"baseline_primary.csv.gz", {key: trace})
        result[key] = {"candidate": candidate, "trials": rows}
        print(key, f"{len(rows)} runs retained", flush=True)
    return result


def save(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False)+"\n")


def sources_unchanged(initial, core):
    return (all(file_hash(ROOT/name) == digest for name, digest in initial["source_sha256"].items())
            and file_hash(core.path) == initial["shared_library_sha256"])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT/"build/pitch-eso-study")
    parser.add_argument("--phase", choices=("development", "full"), default="full")
    parser.add_argument("--require-improvement", action="store_true")
    args = parser.parse_args()
    plan, profile, core = load_plan(), load_profiles()[1], PitchCore()
    validate_profile(profile["motor_base"])
    if any(profile["controller"][key] != value for key, value in plan["baseline"].items()):
        raise ValueError("baseline no longer matches the published profile")
    args.output.mkdir(parents=True, exist_ok=True)
    if any(args.output.iterdir()):
        raise SystemExit("Use an empty output directory to preserve earlier selection/evidence.")
    initial = provenance(core)
    initial["source_sha256"].update({name: file_hash(ROOT/name) for name in (
        "experimental/pitch_eso/run_experiment.py", "experimental/pitch_eso/plan.json", "tools/check_motor_profiles.py")})
    save(args.output/"plan.json", plan)  # Freeze before the first simulation.
    report = {"data_type": "SYNTHETIC; actual C controller and command codec; motor-CAN feedback quantization",
        "phase": args.phase, "plan": plan, "provenance": initial,
        "clock": {"dt_s": DT, "duration_s": DURATION, "evaluation_start_s": RAMP},
        "formal_controller_parameters_changed": False, "development": {}, "holdout": None}
    entries = run_group(core, profile, plan, candidates(plan), plan["development_seeds"], args.output, True)
    baseline_id = candidate_id(plan["baseline"])
    for entry in entries.values():
        entry["assessment"] = assess(entry["trials"], entries[baseline_id]["trials"], plan,
            plan["development_seeds"], profile, entry["candidate"])
    report["development"] = entries
    if not sources_unchanged(initial, core):
        report["source_unchanged_during_run"] = False
        save(args.output/"metrics.json", report)
        raise SystemExit("Sources changed during development; do not open holdout seeds.")
    selected = select_candidate(entries, plan)
    selection = {"candidate_id": selected, "plan_sha256": file_hash(PLAN),
        "development_sha256": hashlib.sha256(json.dumps(entries, sort_keys=True, allow_nan=False).encode()).hexdigest(),
        "scope": "frozen before holdout; no replacement winner after holdout failure"}
    report["selection"] = selection
    save(args.output/"selection.json", selection)
    save(args.output/"metrics.json", report)
    if args.phase == "full" and selected is not None:
        holdout = run_group(core, profile, plan, [plan["baseline"], entries[selected]["candidate"]],
            plan["holdout_seeds"], args.output)
        assessment = assess(holdout[selected]["trials"], holdout[baseline_id]["trials"], plan,
            plan["holdout_seeds"], profile, entries[selected]["candidate"], holdout=True)
        report["holdout"] = {"entries": holdout, "assessment": assessment}
    report["holdout_status"] = ("evaluated" if report["holdout"] is not None else
        "not_run_no_eligible_candidate" if selected is None else "not_requested")
    report["source_unchanged_during_run"] = sources_unchanged(initial, core)
    report["baseline_trace_sha256"] = file_hash(args.output/"baseline_primary.csv.gz")
    save(args.output/"metrics.json", report)
    if not report["source_unchanged_during_run"]:
        raise SystemExit("Sources changed during the experiment; results are invalid.")
    print("Selected:", selected, "Holdout:", report["holdout_status"], flush=True)
    passed = report["holdout"]["assessment"]["passed"] if report["holdout"] else (selected is not None and args.phase == "development")
    if args.require_improvement and not passed:
        raise SystemExit("No candidate passed the requested performance phase; all failures retained.")


if __name__ == "__main__":
    main()
