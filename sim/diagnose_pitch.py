#!/usr/bin/env python3
"""Fixed development-only GM Pitch ablations; no gain search or holdout reuse."""
import argparse
from copy import deepcopy
from itertools import product
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from sim.pitch_core import PitchCore
from sim.run_pitch_profiles import (DEVELOPMENT_SEEDS, LABELS, PitchCase,
    assess_group, compare, load_profiles, provenance, run_trial, summarize, write_trace)
from sim.run_sensitivity import file_hash
from sim.upstream_reference import UpstreamCore


def analyze(original, new, output):
    profile = load_profiles()[1]
    report = {"data_type": "SYNTHETIC; actual C controllers and wire codec",
        "scope": "GM Pitch development diagnosis; fixed gains, seeds 5300-5302; no tuning or new holdout claim",
        "provenance": provenance(new), "upstream": original.metadata,
        "diagnostic_source_sha256": file_hash(Path(__file__)),
        "hypotheses": [
            "If quantization hides gravity benefits, removing quantization alone changes the paired result more than removing noise alone.",
            "If integral and ESO overlap, their fixed on/off factorial changes the gravity benefit.",
            "If command/actuator timing is contaminating disturbance estimates, removing each delay separately changes the result."],
        "sensor_and_timing_ablations": {}, "factorial": []}
    variants = ("configured", "no_noise", "no_position_quantization", "no_velocity_quantization",
                "no_command_delay", "no_actuator_lag")
    for name in variants:
        p, case = deepcopy(profile), PitchCase()
        sensor = p["motor_base"]["sensor"]
        if name == "no_noise":
            sensor.update(position_noise_std_rad=0., velocity_noise_std_rad_s=0.)
        elif name == "no_position_quantization":
            sensor["position_quantum_rad"] = 0.
        elif name == "no_velocity_quantization":
            sensor["velocity_quantum_rad_s"] = 0.
        elif name == "no_command_delay":
            from dataclasses import replace
            case = replace(case, delay_samples=0)
        elif name == "no_actuator_lag":
            p["motor_base"]["actuator_dynamic_assumptions"]["torque_lag_s"] = 0.
        rows = []
        for seed in DEVELOPMENT_SEEDS:
            traces, row = compare(p, case, seed, original, new)
            rows.append(row)
            if name == "configured" and seed == DEVELOPMENT_SEEDS[0]:
                write_trace(output/"pitch_configured.csv.gz", traces)
        entry = {"trials": rows, "summary": summarize(rows)}
        if name == "configured":
            entry["original_development_gate"] = assess_group(profile, rows, DEVELOPMENT_SEEDS)
        report["sensor_and_timing_ablations"][name] = entry
        print(name, entry["summary"]["change_vs_upstream_percent"], flush=True)
    # Cross the actual Ki, ESO gain and gravity switch; don't silently substitute
    # these weaker baselines into the original published acceptance contract.
    for integral, eso, gravity in product((False, True), repeat=3):
        p = deepcopy(profile)
        if not integral:
            p["controller"]["k_integral"] = 0.
        if not eso:
            p["controller"]["eso_gain"] = 0.
        label = LABELS[2] if gravity else LABELS[1]
        rows = []
        for seed in DEVELOPMENT_SEEDS:
            _, metrics = run_trial(p, PitchCase(), label, seed, None, new)
            rows.append({"seed": seed, "metrics": metrics})
        report["factorial"].append({"integral": integral, "eso_compensation": eso,
            "gravity_feedforward": gravity, "trials": rows})
    if provenance(new) != report["provenance"]:
        raise RuntimeError("simulation sources changed during diagnosis")
    report["trace_sha256"] = {"pitch_configured.csv.gz": file_hash(output/"pitch_configured.csv.gz")}
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT/"build/pitch-diagnosis")
    parser.add_argument("--require-development-pass", action="store_true",
                        help="Reproducer: exit nonzero on the unchanged original performance gate")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    report = analyze(UpstreamCore(), PitchCore(), args.output)
    (args.output/"metrics.json").write_text(json.dumps(report, indent=2, allow_nan=False)+"\n")
    if args.require_development_pass and not report["sensor_and_timing_ablations"]["configured"]["original_development_gate"]["passed"]:
        raise SystemExit("GM Pitch development performance gate failed; complete diagnosis retained")


if __name__ == "__main__":
    main()
