#!/usr/bin/env python3
"""Retain a three-way development check without overwriting published benchmarks."""
import argparse
from dataclasses import asdict, replace
import json
import math
from pathlib import Path
import sys
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from sim.c_core import Core
from sim.motor_envelope import MotorEnvelope, actuator_limit
from sim.run_benchmarks import IX, Plant, reference_at as smooth_reference, run_case
from sim.run_motor_profiles import PRIMARY_SEEDS, WireCodec, make_scenario
from sim.run_sensitivity import file_hash, run_trial, scenarios
from sim.upstream_reference import UpstreamCore
from tools.tune_lqr import design_lqr


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--candidate", type=Path)
    parser.add_argument("--candidate-source", type=Path)
    parser.add_argument("--primary-only", "--motor-only", dest="motor_only", action="store_true",
                        help="skip legacy/matrix cases (optional secondary motor cases remain)")
    parser.add_argument("--profile", choices=("all", "dm", "gm"), default="all")
    parser.add_argument("--seeds", nargs="+", type=int, default=list(PRIMARY_SEEDS))
    parser.add_argument("--seed-role", choices=("development", "holdout"), default="development")
    parser.add_argument("--include-other-motor-cases", action="store_true")
    parser.add_argument("--include-motor-stress", action="store_true")
    parser.add_argument("--include-saturation-recovery", action="store_true")
    parser.add_argument("--velocity-error-filter-tau", type=float,
                        help="candidate configuration override on the current library, motor cases only")
    parser.add_argument("--integral-profile-check", action="store_true",
                        help="Ki0 upstream vs same-Ki tuned upstream/new; fixed Ki=2*Kposition and 0.2 N.m integral cap")
    parser.add_argument("--antiwindup-rate", type=float, default=20.,
                        help="explicit preselected back-calculation rate for integral-profile check")
    parser.add_argument("--output", type=Path, default=ROOT/"results/development_checks/candidate.json")
    args = parser.parse_args()
    if any(seed < 0 for seed in args.seeds) or len(args.seeds) != len(set(args.seeds)):
        parser.error("seeds must be unique nonnegative integers")
    if args.velocity_error_filter_tau is not None and (not math.isfinite(args.velocity_error_filter_tau) or
            args.velocity_error_filter_tau <= 0 or not args.motor_only):
        parser.error("filter override must be positive and used with --motor-only")
    if args.velocity_error_filter_tau is not None and args.candidate:
        parser.error("select either a candidate library or a candidate configuration override")
    if args.integral_profile_check and (args.candidate or args.velocity_error_filter_tau is not None or not args.motor_only):
        parser.error("integral profile check requires --motor-only and no other candidate mode")
    if not math.isfinite(args.antiwindup_rate) or args.antiwindup_rate <= 0:
        parser.error("antiwindup rate must be finite and positive")
    current = Core()
    upstream = UpstreamCore()
    cores = {"Upstream C": upstream, "Previous new C": current}
    if args.candidate:
        cores["Candidate C"] = Core(args.candidate)
    elif args.velocity_error_filter_tau is not None:
        cores["Candidate C"] = current
    elif args.integral_profile_check:
        cores = {"Upstream C": upstream, "Upstream tuned C": upstream, "Candidate C": current}
    report = {"data_type": "SYNTHETIC development comparison, no hardware measurements",
        "primary_preselection": "2 Hz / 5 deg, explicit motor profiles; no change to frequency/amplitude based on outcomes",
        "evaluation_seeds": args.seeds, "seed_role": args.seed_role, "profile_selection": args.profile,
        "upstream": upstream.metadata,
        "current_source_sha256": file_hash(ROOT/"src/yaw_controller.c"),
        "current_library_sha256": file_hash(current.path),
        "script_sha256": file_hash(Path(__file__)),
        "candidate_library_sha256": file_hash(args.candidate) if args.candidate else None,
        "candidate_source_sha256": file_hash(args.candidate_source) if args.candidate_source else None,
        "candidate_velocity_error_filter_tau_s": args.velocity_error_filter_tau,
        "integral_profile_check": args.integral_profile_check,
        "integral_design_rule": {"k_integral": "2*Kposition", "integral_cap_nm": .2,
                                 "antiwindup_rate_s": args.antiwindup_rate, "velocity_error_filter_tau_s": 0.,
                                 "selection": "prescribed engineering candidate, no grid search"} if args.integral_profile_check else None,
        "dependency_source_sha256": {name: file_hash(ROOT/name) for name in (
            "sim/run_sensitivity.py", "sim/run_motor_profiles.py", "sim/motor_envelope.py",
            "sim/c_core.py", "sim/upstream_reference.py", "include/yaw_controller.h")},
        "cases": {}}
    args.output.parent.mkdir(parents=True, exist_ok=True)

    def record(name, variants, basis):
        old = variants["Upstream C"].get("position_rmse_deg")
        entry = {"basis": basis, "variants": variants, "rmse_change_vs_upstream_percent": {}}
        for label, metrics in variants.items():
            value = metrics.get("position_rmse_deg")
            entry["rmse_change_vs_upstream_percent"][label] = (value/old-1)*100 if old and value is not None else None
        if "Candidate C" in cores:
            comparison_label = "Previous new C" if "Previous new C" in variants else "Upstream tuned C"
            before, after = variants[comparison_label].get("position_rmse_deg"), variants["Candidate C"].get("position_rmse_deg")
            entry["candidate_change_vs_"+("previous" if comparison_label == "Previous new C" else "tuned_upstream")+"_percent"] = (after/before-1)*100 if before and after is not None else None
        report["cases"][name] = entry
        print(name+": "+"; ".join(f"{label}={metrics.get('position_rmse_deg', float('nan')):.6f} deg"
            for label, metrics in variants.items()), flush=True)
        args.output.write_text(json.dumps(report, indent=2, allow_nan=False)+"\n")

    filenames = {"all": ("dm4310_24v.json", "gm6020_current.json"),
                 "dm": ("dm4310_24v.json",), "gm": ("gm6020_current.json",)}[args.profile]
    for filename in filenames:
        profile_path = ROOT/"profiles"/filename
        profile = json.loads(profile_path.read_text())
        scenario = make_scenario(profile, 2., 5.)
        motor = profile["motor"]
        rated = {"rated_velocity_rad_s": motor["rated_speed_rad_s"], "rated_torque_nm": motor["rated_torque_nm"]} if profile["protocol"]["kind"] == "dm_mit" else {}
        envelope = MotorEnvelope(scenario.torque_cap_nm, motor["zero_speed_anchor_nm"], motor["no_load_speed_rad_s"], **rated)
        gain = {key: profile["controller"][key] for key in ("k_position", "k_velocity")}
        def motor_trial(frequency, amplitude, seed, group, plant_override=None):
            case_scenario = make_scenario(profile, frequency, amplitude)
            if plant_override is not None:
                case_scenario = replace(case_scenario, plant=plant_override)
            variants = {}
            for label, core in cores.items():
                codec = WireCodec(profile, current.lib)
                configuration = dict(profile["controller"])
                if args.velocity_error_filter_tau is not None:
                    configuration["velocity_error_filter_tau_s"] = (
                        args.velocity_error_filter_tau if label == "Candidate C" else 0.)
                if args.integral_profile_check:
                    configuration.update(k_integral=0. if label == "Upstream C" else 2*gain["k_position"],
                        integral_limit_nm=0. if label == "Upstream C" else .2,
                        antiwindup_rate_s=args.antiwindup_rate, velocity_error_filter_tau_s=0.)
                _, variants[label] = run_trial(core, case_scenario, gain, seed, quantizer=codec,
                    limiter=lambda u, v: actuator_limit(u, v, envelope), controller_config=configuration)
                variants[label]["wire_codec"] = codec.metrics()
            record(f"{group}/{profile['name']}/{frequency:g}hz{amplitude:g}/{seed}", variants,
                   {"profile": profile, "scenario": asdict(case_scenario),
                    "profile_sha256": file_hash(profile_path), "seed": seed})
        for seed in args.seeds:
            motor_trial(2., 5., seed, "primary")
        if args.include_other_motor_cases:
            for frequency, amplitude in ((1., 5.), (3., 5.), (3., 10.)):
                motor_trial(frequency, amplitude, 4310, "motor_secondary")
        if args.include_motor_stress:
            plant = scenario.plant
            variations = {
                "inertia_x0.75": replace(plant, inertia_kg_m2=plant.inertia_kg_m2*.75),
                "inertia_x1.25": replace(plant, inertia_kg_m2=plant.inertia_kg_m2*1.25),
                "delay_5ms": replace(plant, command_delay_samples=5),
                "delay_10ms_boundary": replace(plant, command_delay_samples=10),
                "noise_std_x3": replace(plant, position_noise_std_rad=plant.position_noise_std_rad*3,
                                         velocity_noise_std_rad_s=plant.velocity_noise_std_rad_s*3),
            }
            for label, plant_override in variations.items():
                motor_trial(2., 5., 4310, "motor_stress/"+label, plant_override)
        if args.include_saturation_recovery:
            def recovery_reference(time_s, unused_frequency, unused_amplitude):
                left = smooth_reference(time_s, 3., math.radians(10.))
                right = smooth_reference(time_s, 2., math.radians(5.))
                s = min(1., max(0., time_s-3.))
                blend = 10*s**3-15*s**4+6*s**5
                first = 30*s**2-60*s**3+30*s**4 if 0 < s < 1 else 0.
                second = 60*s-180*s**2+120*s**3 if 0 < s < 1 else 0.
                return ((1-blend)*left[0]+blend*right[0],
                        (1-blend)*left[1]+blend*right[1]+first*(right[0]-left[0]),
                        (1-blend)*left[2]+blend*right[2]+2*first*(right[1]-left[1])+second*(right[0]-left[0]))
            variants = {}
            recovery_scenario = make_scenario(profile, 3., 10.)
            with patch("sim.run_sensitivity.reference_at", recovery_reference):
                for label, core in cores.items():
                    codec = WireCodec(profile, current.lib)
                    configuration = dict(profile["controller"])
                    if args.integral_profile_check:
                        configuration.update(k_integral=0. if label == "Upstream C" else 2*gain["k_position"],
                            integral_limit_nm=0. if label == "Upstream C" else .2,
                            antiwindup_rate_s=args.antiwindup_rate, velocity_error_filter_tau_s=0.)
                    trace, metrics = run_trial(core, recovery_scenario, gain, 4310, duration=8., quantizer=codec,
                        limiter=lambda u, v: actuator_limit(u, v, envelope), controller_config=configuration)
                    recovery = trace[trace[:, IX["time_s"]] >= 4.]
                    if len(recovery):
                        error = np.rad2deg(recovery[:, IX["error_rad"]])
                        metrics.update(recovery_window_s=[4., 8.],
                            recovery_position_rmse_deg=float(np.sqrt(np.mean(error**2))),
                            recovery_position_max_abs_error_deg=float(np.max(np.abs(error))),
                            recovery_command_rms_nm=float(np.sqrt(np.mean(recovery[:, IX["command_nm"]]**2))))
                    variants[label] = metrics
            record(f"saturation_recovery/{profile['name']}/4310", variants,
                   "3 Hz / 10 deg startup and saturation; C2 quintic crossfade over 3–4 s to 2 Hz / 5 deg; separate 4–8 s recovery metrics")
    if args.motor_only:
        return
    gain = design_lqr()["gain"]
    stressed = Plant(inertia_kg_m2=.039*1.25, damping_nm_s_rad=.30*.8, external_torque_nm=.35,
        coulomb_nm=.12, torque_lag_s=.002, command_delay_samples=1, position_quantum_rad=25./65535.,
        velocity_quantum_rad_s=60./4095., position_noise_std_rad=.00007, velocity_noise_std_rad_s=.01)
    for kind, plant in [("nominal", Plant()), ("stressed", stressed)]:
        for frequency, amplitude in ((1., 5.), (3., 20.), (5., 20.)):
            variants = {}
            for label, core in cores.items():
                _, variants[label] = run_case(core, plant, frequency, amplitude, gain, .8, 4310)
            record(f"legacy/{kind}/{frequency:g}hz{amplitude:g}", variants,
                   "unchanged original six synthetic benchmark scenarios; old-style constant 7 N.m cap")
    indexed = {scenario.name: scenario for scenario in scenarios()}
    for name in ("inertia_x0.5", "inertia_x0.75", "command_delay_10ms", "author_runtime_2hz10"):
        variants = {}
        for label, core in cores.items():
            _, variants[label] = run_trial(core, indexed[name], gain)
        record("sensitivity/"+name, variants, "same fixed matrix configuration including updated motor envelope")


if __name__ == "__main__":
    main()
