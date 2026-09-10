#!/usr/bin/env python3
"""Independently check the declared synthetic RLS experiment, not controller gains.

Recompute numerical gates from per-trial estimates and raw CSV traces. Also
check the declared seeds, protocol, current source/library hashes and rejection
evidence. This checks experiment integrity; it is not a hardware acceptance.
"""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import statistics
import struct

ROOT = Path(__file__).resolve().parents[1]
EXPERIMENT = ROOT / "experimental/online_rls"
SCALE = (0.025, 0.060)
INITIAL = (0.6, 1.6)
INITIAL_FLOAT32 = tuple(struct.unpack("f", struct.pack("f", value))[0] for value in INITIAL)
SAMPLES = 3000
SOURCES = ("src/rls_shadow.c", "include/rls_shadow.h", "run_experiments.py",
           "src/rls_provenance.c.in")
EXPECTED_PROTOCOL = {
    "duration_s": 60.0, "source_dt_s": 0.001,
    "nonoverlap_window_samples": 20, "nominal_update_s": 0.02,
    "normalization_parameter_scale": list(SCALE),
    "normalization_impulse_nm_s": 0.001,
    "initial_normalized_parameters": list(INITIAL),
    "initial_inverse_information_diagonal": 100.0,
    "independent_label_noise_normalized_std": 0.03,
    "seeds": list(range(10)), "pe_window_samples": 100,
    "pe_min_eigenvalue": 0.002, "pe_max_condition": 1e4,
    "forgetting_time_valid_updates_s": 5.0,
    "acceptance": {"baseline_each_parameter_error_max": 0.05,
        "c_vs_weighted_batch_normalized_abs_max": 0.002,
        "drift_tail_b_error_nm_s_rad_max": 0.01,
        "drift_tail_error_ratio_to_no_forgetting_max": 0.5,
        "all_rejected_updates_preserve_theta_and_p_exactly": True},
    "diagnostic_only_cases": ["regressor_noise", "20ms_torque_label_timestamp_shift",
                              "torque_scale_1.25"],
}


def require(condition, message):
    if not condition:
        raise ValueError(message)


def finite_number(value):
    return type(value) in (int, float) and math.isfinite(value)


def vector(value):
    require(isinstance(value, list) and len(value) == 2 and
            all(finite_number(v) for v in value), "Expected two finite parameter values")
    return value


def close(actual, expected, label):
    require(finite_number(actual) and math.isclose(actual, expected, rel_tol=1e-8,
                                                  abs_tol=1e-10),
            f"Inconsistent recomputed metric: {label}")


def check_protocol(protocol):
    for key, expected in EXPECTED_PROTOCOL.items():
        require(protocol.get(key) == expected, f"Changed/missing experiment protocol: {key}")


def check_trials(trials, *, batch):
    require(isinstance(trials, list) and [r.get("seed") for r in trials] == list(range(10)) and
            all(type(r.get("seed")) is int for r in trials),
            "Expected exactly seeds 0 through 9, in order")
    errors, batch_differences = [], []
    for row in trials:
        parameters = vector(row["final_normalized_parameters"])
        require(all(0.02 <= value <= 5.0 for value in parameters),
                "Final estimate lies outside the declared parameter bounds")
        errors.append([value - 1 for value in parameters])
        accepted, rejected = row["accepted_updates"], row["rejected_updates"]
        require(type(accepted) is int and type(rejected) is int and
                2000 < accepted <= SAMPLES and accepted + rejected == SAMPLES,
                "Incomplete or implausible trial update counts")
        require(row["rejected_preserved_exactly"] is True,
                "A rejected trial update modified parameters or P")
        if batch:
            reference = vector(row["batch_normalized_parameters"])
            batch_differences.append(max(abs(a-b) for a, b in zip(parameters, reference)))
    summary = {
        "mean_relative_error": [statistics.mean(v[i] for v in errors) for i in range(2)],
        "max_absolute_relative_error": [max(abs(v[i]) for v in errors) for i in range(2)],
    }
    return summary, max(batch_differences, default=0.0)


def check_summary(actual, expected, label):
    for key in ("mean_relative_error", "max_absolute_relative_error"):
        for a, b in zip(vector(actual[key]), expected[key]):
            close(a, b, label + "." + key)


def read_trace(path):
    with path.open(newline="") as stream:
        reader = csv.DictReader(stream)
        require(reader.fieldnames == ["time_s", "J_kg_m2", "b_Nms_rad", "status",
                                     "excitation_ok", "gram_min_eigenvalue"],
                f"Changed trace schema: {path.name}")
        rows = [{key: float(value) for key, value in row.items()} for row in reader]
    require(len(rows) == SAMPLES, f"Incomplete trace: {path.name}")
    previous = tuple(value * scale for value, scale in zip(INITIAL_FLOAT32, SCALE))
    for index, row in enumerate(rows):
        require(all(math.isfinite(value) for value in row.values()), "Nonfinite trace value")
        close(row["time_s"], (index + 1) * 0.02, "trace sample period")
        require(row["status"] in range(11) and row["excitation_ok"] in (0, 1),
                "Invalid status or excitation flag")
        require(row["status"] != 0 or row["excitation_ok"] == 1,
                "Accepted update with insufficient excitation")
        parameters = (row["J_kg_m2"], row["b_Nms_rad"])
        require(row["status"] == 0 or parameters == previous,
                "Rejected CSV update changed parameter estimates")
        previous = parameters
    return rows


def accepted_count(trace):
    return sum(row["status"] == 0 for row in trace)


def check_report(report_path):
    report = json.loads(report_path.read_text())
    output = report_path.parent
    require(type(report.get("schema_version")) is int and report["schema_version"] == 1,
            "Unsupported report schema")
    require(report.get("protocol") == "protocol.json", "Unexpected protocol path")
    check_protocol(json.loads((output / "protocol.json").read_text()))
    require(report.get("performance_of_controller_tested") is False,
            "RLS regression checks cannot claim controller performance")
    require(set(report["source_sha256"]) == set(SOURCES), "Unexpected source manifest")
    for name in SOURCES:
        require(report["source_sha256"][name] ==
                hashlib.sha256((EXPERIMENT / name).read_bytes()).hexdigest(),
                f"Source hash mismatch: {name}")
    provenance = report["provenance"]
    require(provenance["source_unchanged_during_run"] is True and
            provenance["library_unchanged_during_run"] is True and
            provenance["library_source_matches"] is True, "Library provenance check failed")
    require(provenance["library_source_sha256"] ==
            {name: report["source_sha256"][name] for name in SOURCES[:2]},
            "Embedded source/header manifest does not match the current sources")
    library = ROOT / provenance["library_path"]
    require(library.is_file() and hashlib.sha256(library.read_bytes()).hexdigest() ==
            provenance["library_sha256"], "Library is missing or its hash changed")
    normal, batch_difference = check_trials(report["normal_trials"], batch=True)
    check_summary(report["normal_10_seeds"], normal, "normal")
    close(report["max_c_vs_batch_abs_normalized_difference"], batch_difference, "normal batch")
    require(set(report["diagnostic_trials"]) == {"eiv", "delay", "scale"} and
            set(report["diagnostic_mismatch_cases"]) == {"eiv", "delay", "scale"},
            "Missing or changed diagnostic scenarios")
    for case, trials in report["diagnostic_trials"].items():
        summary, _ = check_trials(trials, batch=False)
        check_summary(report["diagnostic_mismatch_cases"][case], summary, case)
    traces = {name: read_trace(output / (name + ".csv")) for name in
              ("normal", "drift_no_forgetting", "drift_forgetting", "rank_deficient", "faults")}
    close(traces["normal"][-1]["J_kg_m2"] / SCALE[0],
          report["normal_trials"][0]["final_normalized_parameters"][0], "normal CSV J")
    close(traces["normal"][-1]["b_Nms_rad"] / SCALE[1],
          report["normal_trials"][0]["final_normalized_parameters"][1], "normal CSV b")
    require(accepted_count(traces["normal"]) == report["normal_trials"][0]["accepted_updates"],
            "Normal CSV count differs from trial")
    drift = report["drift"]
    require(drift["seed"] == 100, "Incorrect drift seed")
    close(drift["forgetting_factor_float32"], 0.9960079789161682, "forgetting factor")
    drift_batch = vector(drift["weighted_batch_normalized_parameters"])
    drift_estimate = vector(drift["forgetting_final_normalized_parameters"])
    drift_batch_difference = max(abs(a-b) for a, b in zip(drift_estimate, drift_batch))
    close(drift["c_vs_weighted_batch_abs_normalized_difference"], drift_batch_difference,
          "drift batch")
    tail_errors = {}
    for mode in ("no_forgetting", "forgetting"):
        trace = traces["drift_" + mode]
        require(accepted_count(trace) == drift[mode + "_accepted_updates"],
                "Drift CSV count differs from report")
        require(drift[mode + "_rejected_preserved_exactly"] is True,
                "Rejected drift update modified parameters or P")
        for key, expected in zip(("J_kg_m2", "b_Nms_rad"),
                                 vector(drift[mode + "_final_normalized_parameters"])):
            scale = SCALE[0 if key == "J_kg_m2" else 1]
            close(trace[-1][key] / scale, expected, "drift CSV final parameter")
        tail_errors[mode] = statistics.mean(abs(row["b_Nms_rad"] - 0.1)
                                            for row in trace if row["time_s"] >= 50.0)
        close(drift[mode + "_tail_b_absolute_error"], tail_errors[mode], "drift tail MAE")
    rank = report["rank_deficient"]
    require(rank["seed"] == 101, "Incorrect rank-deficient seed")
    require(rank["accepted_updates"] == accepted_count(traces["rank_deficient"]) == 0 and
            rank["rejected_updates"] == SAMPLES and rank["unchanged_from_init"] is True and
            rank["rejected_preserved_exactly"] is True, "Rank-deficient stream must freeze")
    for row in traces["rank_deficient"]:
        close(row["J_kg_m2"] / SCALE[0], INITIAL_FLOAT32[0], "frozen J")
        close(row["b_Nms_rad"] / SCALE[1], INITIAL_FLOAT32[1], "frozen b")
        require(row["status"] == 5 and row["excitation_ok"] == 0,
                "Rank-deficient sample was not rejected for insufficient excitation")
    faults = report["faults"]
    require(faults["seed"] == 102, "Incorrect fault-injection seed")
    fault_rows = [i + offset for i in range(250, SAMPLES, 200) for offset in range(3)]
    statuses = [int(traces["faults"][i]["status"]) for i in fault_rows]
    require(faults["fault_rows"] == fault_rows and faults["fault_row_statuses"] == statuses and
            statuses == [4, 6, 7] * 14, "Incorrect injected-fault rejection")
    require(faults["accepted_updates"] == accepted_count(traces["faults"]) and
            faults["accepted_updates"] + faults["rejected_updates"] == SAMPLES and
            faults["rejected_preserved_exactly"] is True, "Fault evidence is inconsistent")
    for key, scale, error in zip(("J_kg_m2", "b_Nms_rad"), SCALE,
                                 vector(report["faults_final_relative_parameter_error"])):
        close(error, traces["faults"][-1][key] / scale - 1, "fault CSV final error")
    for key, value in (("rank_deficient_accepted", 0), ("fault_rows", len(fault_rows)),
                       ("faults_accepted_updates", faults["accepted_updates"])):
        require(report[key] == value, f"Inconsistent legacy summary: {key}")
    gates = {
        "normal_parameter_recovery": max(normal["max_absolute_relative_error"]) < 0.05,
        "normal_matches_same_prior_batch": batch_difference < 0.002,
        "forgetting_matches_same_prior_batch": drift_batch_difference < 0.002,
        "slow_drift_tracking": tail_errors["forgetting"] < 0.01 and
            tail_errors["forgetting"] < 0.5 * tail_errors["no_forgetting"],
        "rank_deficient_freezes": True, "fault_rows_rejected": True,
        "every_rejection_preserves_parameters_and_p": True,
    }
    require(report["gates"] == gates and all(type(v) is bool for v in report["gates"].values()),
            "Self-reported gates differ from independent checks")
    require(report["all_declared_algorithm_gates_passed"] is all(gates.values()),
            "Self-reported aggregate differs from independent checks")
    require(all(gates.values()), "One or more independently recomputed RLS gates failed")
    return {"passed": True, "scope": "Synthetic shadow RLS only; no controller performance claim",
            "gates": gates, "normal_max_abs_relative_error": normal["max_absolute_relative_error"],
            "forgetting_tail_b_absolute_error": tail_errors["forgetting"],
            "checked_trials": 44, "checked_trace_rows": 5 * SAMPLES,
            "report_sha256": hashlib.sha256(report_path.read_bytes()).hexdigest()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    try:
        result = check_report(args.report)
    except (KeyError, ValueError, OSError, TypeError) as error:
        parser.exit(1, f"Online RLS result check failed: {error}\n")
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
