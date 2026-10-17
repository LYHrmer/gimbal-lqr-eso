#!/usr/bin/env python3
"""Both axes, independent feedback clocks; fixed development sensitivity probes."""
import argparse
from copy import deepcopy
from dataclasses import asdict
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from sim.c_core import Core
from sim.feedback_timing import ChannelTiming
from sim.motor_envelope import MotorEnvelope, actuator_limit
from sim.pitch_core import PitchCore
from sim.run_motor_profiles import WireCodec, make_scenario
from sim.run_pitch_profiles import (DEVELOPMENT_SEEDS, LABELS, PitchCase, load_profiles,
    provenance, run_trial as pitch_trial, write_trace as pitch_trace)
from sim.run_sensitivity import (DT, DURATION, RAMP, file_hash, run_trial as yaw_trial,
                                 write_trace as yaw_trace)


def clocks():
    # Declared before running: preserve the five previous synchronous cases.
    cases = {f"sync_{period}_{delay}": (ChannelTiming(period, delay),)*2
             for period, delay in ((1, 0), (2, 0), (1, 2), (2, 2), (2, 5))}
    cases.update({
        "angle500_gyro1000": (ChannelTiming(2), ChannelTiming()),
        "angle500_phase1_gyro1000_delay1": (ChannelTiming(2, phase_samples=1), ChannelTiming(1, 1)),
        "transport_jitter": (ChannelTiming(2, 1, delay_jitter_samples=(0, 1, -1)), ChannelTiming()),
        "drop_every_second_angle": (ChannelTiming(2, drop_every=2), ChannelTiming()),
        "out_of_order_angle": (ChannelTiming(1, delay_jitter_samples=(3, 0)), ChannelTiming()),
    })
    return cases


def analyze(output, library=None):
    yaw, pitch = Core(library), PitchCore(library)
    initial = provenance(pitch)
    sources = ("sim/run_multirate.py", "sim/run_sensitivity.py", "sim/feedback_timing.py")
    initial["source_sha256"].update({name: file_hash(ROOT/name) for name in sources})
    report = {"schema_version": 1, "data_type": "SYNTHETIC; actual public C and C motor codecs",
        "scope": "fixed 1ms control; independent acquisition, hold, transport jitter, drops and out-of-order delivery; no dynamic IMU fusion or hardware claim",
        "clock": {"dt_s": DT, "duration_s": DURATION, "evaluation_start_s": RAMP},
        "provenance": initial, "seed_role": "reused development seeds; no independent holdout claim",
        "clocks": {name: [asdict(c) for c in pair] for name, pair in clocks().items()},
        "comparison": "only ESO compensation gain changes within each pair; same Ki, gravity, model, reference and limits",
        "profiles": {}, "traces": {}}
    for axis in ("yaw", "pitch"):
        profiles = (load_profiles() if axis == "pitch" else
            [json.loads((ROOT/"profiles"/f).read_text()) for f in ("dm4310_24v.json", "gm6020_current.json")])
        seeds = DEVELOPMENT_SEEDS if axis == "pitch" else (4310, 4311, 4312)
        for profile in profiles:
            rows = []
            for name, clock in clocks().items():
                for seed in seeds:
                    traces, metrics = {}, {}
                    for label, gain in (("observe_only", 0.), ("compensated", profile["controller"]["eso_gain"])):
                        if axis == "pitch":
                            p = deepcopy(profile)
                            p["controller"]["eso_gain"] = gain
                            data, metric = pitch_trial(p, PitchCase(), LABELS[2], seed, None, pitch,
                                                      feedback_clocks=clock)
                        else:
                            scenario = make_scenario(profile, 2., 5.)
                            motor = profile["motor"]
                            rated = ({"rated_velocity_rad_s": motor["rated_speed_rad_s"],
                                      "rated_torque_nm": motor["rated_torque_nm"]}
                                     if profile["protocol"]["kind"] == "dm_mit" else {})
                            envelope = MotorEnvelope(scenario.torque_cap_nm, motor["zero_speed_anchor_nm"],
                                                     motor["no_load_speed_rad_s"], **rated)
                            config = dict(profile["controller"], eso_gain=gain)
                            data, metric = yaw_trial(yaw, scenario,
                                {k: config[k] for k in ("k_position", "k_velocity")}, seed,
                                limiter=lambda u, v: actuator_limit(u, v, envelope),
                                quantizer=WireCodec(profile, yaw.lib), controller_config=config,
                                feedback_clocks=clock)
                        traces[label], metrics[label] = data, metric
                    rows.append({"clock": name, "seed": seed, "expected_stale_fault": name == "sync_2_5",
                                 "variants": metrics})
                    # Preselected representative pairs, not a search for best traces.
                    if profile["name"].startswith("gm6020") and name == "angle500_gyro1000" and seed == seeds[0]:
                        path = output/f"{axis}_gm_angle500_gyro1000.csv.gz"
                        (pitch_trace if axis == "pitch" else yaw_trace)(path, traces)
                        report["traces"][path.name] = file_hash(path)
                print(axis, profile["name"], name, "retained", flush=True)
            report["profiles"][profile["name"]] = {"axis": axis, "seeds": list(seeds), "cases": rows}
    if any(file_hash(ROOT/name) != digest for name, digest in initial["source_sha256"].items()):
        raise RuntimeError("sources changed during experiment")
    report["source_unchanged_during_run"] = True
    report["interpretation"] = "All planned runs are retained. Timeout/physical boundary failures are not passes and partial metrics cannot be compared as complete tracking runs. No performance threshold is inferred."
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT/"build/multirate")
    parser.add_argument("--library", type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    report = analyze(args.output, args.library)
    (args.output/"metrics.json").write_text(json.dumps(report, indent=2, allow_nan=False)+"\n")
    # Experiment integrity, not a manufactured tracking performance gate.
    for entry in report["profiles"].values():
        if len(entry["cases"]) != len(clocks())*len(entry["seeds"]):
            raise SystemExit("incomplete timing matrix")
        for row in entry["cases"]:
            for metric in row["variants"].values():
                fault = metric["first_latched_fault"]
                if row["expected_stale_fault"]:
                    if not fault or fault["status"] != 6:
                        raise SystemExit("expected stale feedback was not rejected")
                    if not metric["completed"] and not (entry["axis"] == "pitch" and
                            metric.get("first_physical_limit_crossing") and
                            metric["failure"] == "physical joint boundary crossed; no position clamping/contact model"):
                        raise SystemExit("unexpected post-fault numerical failure retained")
                elif not metric["completed"] or fault:
                    raise SystemExit("unexpected incomplete/faulted run retained in report")


if __name__ == "__main__":
    main()
