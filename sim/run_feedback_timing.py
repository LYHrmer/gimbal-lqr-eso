#!/usr/bin/env python3
"""Synthetic Yaw feedback-clock sensitivity; fixed profiles, no tuning or hardware claim."""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from sim.c_core import Core
from sim.motor_envelope import MotorEnvelope, actuator_limit
from sim.run_motor_profiles import WireCodec, make_scenario
from sim.run_sensitivity import DT, DURATION, RAMP, file_hash, run_trial

# Reuse development seeds. These are sensitivity probes, not new holdout data.
SEEDS = (4310, 4311, 4312)
TIMINGS = ((1, 0), (2, 0), (1, 2), (2, 2), (2, 5))


def analyze(core):
    report = {
        "data_type": "SYNTHETIC: actual C Yaw core and wire codecs; no hardware logs",
        "scope": "synchronous position/velocity sample-and-hold plus integer feedback transport delay; fixed gains, no prediction or retuning; Pitch not covered",
        "clock": {"dt_s": DT, "duration_s": DURATION, "evaluation_start_s": RAMP,
                  "startup": "t=0 feedback already synchronized; subsequent samples delayed",
                  "noise": "one independent pair per acquisition, held until the next arrival",
                  "reference_age_s": 0., "applied_torque_valid": False},
        "seeds": list(SEEDS), "seed_role": "reused development seeds, not independent holdout",
        "comparison": "same configured Ki and all other parameters; only ESO compensation gain 0 versus profile value; across clocks, the same seed does not imply identical continuous-time noise",
        "source_sha256": {name: file_hash(ROOT/name) for name in (
            "src/yaw_controller.c", "include/yaw_controller.h", "sim/c_core.py",
            "sim/run_sensitivity.py", "sim/run_motor_profiles.py", "sim/run_feedback_timing.py",
            "sim/feedback_timing.py",
            "profiles/dm4310_24v.json", "profiles/gm6020_current.json")},
        "library_sha256": file_hash(core.path), "profiles": {},
    }
    for filename in ("dm4310_24v.json", "gm6020_current.json"):
        profile = json.loads((ROOT/"profiles"/filename).read_text())
        scenario = make_scenario(profile, 2., 5.)
        motor = profile["motor"]
        rated = ({"rated_velocity_rad_s": motor["rated_speed_rad_s"],
                  "rated_torque_nm": motor["rated_torque_nm"]}
                 if profile["protocol"]["kind"] == "dm_mit" else {})
        envelope = MotorEnvelope(scenario.torque_cap_nm, motor["zero_speed_anchor_nm"],
                                 motor["no_load_speed_rad_s"], **rated)
        gain = {key: profile["controller"][key] for key in ("k_position", "k_velocity")}
        rows = []
        for period, delay in TIMINGS:
            for seed in SEEDS:
                variants = {}
                for label, compensation in (("observe_only", 0.),
                        ("configured_compensation", profile["controller"]["eso_gain"])):
                    config = dict(profile["controller"], eso_gain=compensation)
                    codec = WireCodec(profile, core.lib)
                    _, metrics = run_trial(core, scenario, gain, seed,
                        limiter=lambda u, v: actuator_limit(u, v, envelope),
                        quantizer=codec, controller_config=config,
                        feedback_period_samples=period, feedback_delay_samples=delay)
                    variants[label] = metrics
                rows.append({"period_samples": period, "delay_samples": delay,
                             "seed": seed, "variants": variants})
        report["profiles"][profile["name"]] = {"scenario": asdict(scenario), "cases": rows}
        print(f"{profile['name']}: {len(rows)*2} trials retained", flush=True)
    report["all_trials_completed"] = all(metric["completed"]
        for profile in report["profiles"].values() for row in profile["cases"]
        for metric in row["variants"].values())
    report["interpretation"] = "Completed means the simulation ran to its end, not that tracking passed. Timeout faults remain in the report; no RMSE acceptance threshold is inferred."
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--library", type=Path, default=ROOT/"build/libyaw_controller.so")
    parser.add_argument("--output", type=Path, default=ROOT/"build/feedback_timing.json")
    args = parser.parse_args()
    report = analyze(Core(args.library))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    if not report["all_trials_completed"]:
        raise SystemExit("An incomplete simulation was retained; inspect the report.")


if __name__ == "__main__":
    main()
