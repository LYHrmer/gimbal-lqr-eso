#!/usr/bin/env python3
"""Two motor profiles, actual C wire codecs, and five paired primary-case seeds."""
import argparse
import ctypes as ct
from functools import lru_cache
import json
import math
import os
from pathlib import Path
import platform
import shlex
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import numpy as np
import matplotlib
import scipy

from sim.c_core import Config, Core
from sim.motor_envelope import MotorEnvelope, actuator_limit
from sim.run_benchmarks import DT, RAMP, Plant
from sim.run_sensitivity import (
    COLUMNS, DURATION, EXTRA_COLUMNS, Scenario, file_hash, linear_feedback_diagnostics, plot_tracking, run_trial, write_trace,
)
from sim.upstream_reference import UpstreamCore

PRIMARY_SEEDS = (4310, 4311, 4312, 4313, 4314)
VALIDATION_SEEDS = (4315, 4316, 4317, 4318, 4319)
TRAJECTORIES = ((1., 5.), (2., 5.), (3., 5.), (3., 10.))


def show(value, spec=".5f"):
    return format(value, spec) if value is not None else "unavailable"


def change_text(value):
    return show(value, "+.2f")+"%" if value is not None else "incomplete"


class DMConfig(ct.Structure):
    _fields_ = [(name, ct.c_double) for name in
                ("p_max_rad", "v_max_rad_s", "t_max_nm", "torque_limit_nm")] + [
                ("motor_can_id", ct.c_uint32), ("master_can_id", ct.c_uint32)]


class DMCommand(ct.Structure):
    _fields_ = [("can_id", ct.c_uint32), ("data", ct.c_uint8*8), ("dlc", ct.c_uint8),
                ("valid", ct.c_bool), ("torque_wire_nm", ct.c_double)]


class GMConfig(ct.Structure):
    _fields_ = [("torque_constant_nm_a", ct.c_double), ("current_limit_a", ct.c_double),
                ("torque_limit_nm", ct.c_double), ("current_mode_confirmed", ct.c_bool)]


class GMWord(ct.Structure):
    _fields_ = [("raw", ct.c_int16), ("valid", ct.c_bool), ("limited", ct.c_bool),
                ("current_wire_a", ct.c_double), ("torque_wire_nm", ct.c_double)]


@lru_cache(maxsize=1)
def verify_codec_layout():
    classes = {"dm_mit_config_t": DMConfig, "dm_mit_command_t": DMCommand,
               "gm6020_config_t": GMConfig, "gm6020_current_word_t": GMWord}
    expected = {"gm6020_result_t": ct.sizeof(ct.c_int), "dm_mit_result_t": ct.sizeof(ct.c_int)}
    source = ['#include <stdio.h>', '#include <stddef.h>', '#include "dm_mit.h"',
              '#include "gm6020.h"', 'int main(void) {']
    for name in expected:
        source.append(f'printf("{name} %zu\\n", sizeof({name}));')
    for name, cls in classes.items():
        expected[name] = ct.sizeof(cls)
        source.append(f'printf("{name} %zu\\n", sizeof({name}));')
        for field, _ in cls._fields_:
            key = name+"."+field
            expected[key] = getattr(cls, field).offset
            source.append(f'printf("{key} %zu\\n", offsetof({name}, {field}));')
    source.append('return 0; }')
    with tempfile.TemporaryDirectory(prefix="motor-codec-abi-") as directory:
        source_file, program = Path(directory)/"probe.c", Path(directory)/"probe"
        source_file.write_text("\n".join(source)+"\n")
        subprocess.run(shlex.split(os.environ.get("CC", "cc"))+["-std=c99", "-I", str(ROOT/"include"),
            str(source_file), "-o", str(program)], check=True, capture_output=True, text=True)
        actual = dict((key, int(value)) for key, value in (line.split() for line in
            subprocess.check_output([str(program)], text=True).splitlines()))
    if expected != actual:
        raise RuntimeError("C protocol / ctypes ABI mismatch")
    return {name: ct.sizeof(cls) for name, cls in classes.items()}


class WireCodec:
    """Both variants pass through the same real C transport-free conversion."""
    def __init__(self, profile, library):
        self.layout = verify_codec_layout()
        protocol = profile["protocol"]
        self.kind = protocol["kind"]
        self.lib = library
        self.records = []
        if self.kind == "dm_mit":
            self.config = DMConfig(**{key: protocol[key] for key, _ in DMConfig._fields_})
            self.lib.dm_mit_encode_torque.argtypes = [ct.POINTER(DMConfig), ct.c_double, ct.POINTER(DMCommand)]
            self.lib.dm_mit_encode_torque.restype = ct.c_int
        elif self.kind == "gm6020_current_v1_4":
            self.config = GMConfig(**{key: protocol[key] for key, _ in GMConfig._fields_})
            self.lib.gm6020_torque_to_word.argtypes = [ct.POINTER(GMConfig), ct.c_double, ct.POINTER(GMWord)]
            self.lib.gm6020_torque_to_word.restype = ct.c_int
        else:
            raise ValueError(f"unsupported codec {self.kind}")

    def __call__(self, requested):
        if self.kind == "dm_mit":
            command = DMCommand()
            status = self.lib.dm_mit_encode_torque(ct.byref(self.config), requested, ct.byref(command))
            if status != 0 or not command.valid:
                raise RuntimeError(f"DM C codec rejected command {requested}: {status}")
            raw = ((command.data[6]&15)<<8) | command.data[7]
            wire, current = command.torque_wire_nm, None
        else:
            command = GMWord()
            status = self.lib.gm6020_torque_to_word(ct.byref(self.config), requested, ct.byref(command))
            if status != 0 or not command.valid:
                raise RuntimeError(f"GM C codec rejected command {requested}: {status}")
            raw, wire, current = command.raw, command.torque_wire_nm, command.current_wire_a
        self.records.append((raw, wire, current))
        return wire

    def metrics(self):
        records = self.records[round(RAMP/DT):]
        if not records:
            return {}
        result = {"kind": self.kind, "raw_code_min": int(min(row[0] for row in records)),
                  "raw_code_max": int(max(row[0] for row in records)),
                  "wire_torque_peak_abs_nm": float(max(abs(row[1]) for row in records))}
        if self.kind == "gm6020_current_v1_4":
            current = np.array([row[2] for row in records])
            result.update(wire_current_peak_abs_a=float(np.max(np.abs(current))),
                          wire_current_rms_a=float(np.sqrt(np.mean(current**2))))
        return result


def make_scenario(profile, frequency, amplitude):
    config = profile["controller"]
    # Predetermined common external load, not identified motor-specific friction.
    plant = Plant(inertia_kg_m2=config["inertia_kg_m2"], damping_nm_s_rad=config["damping_nm_s_rad"],
        external_torque_nm=.15, coulomb_nm=.06, **profile["sensor"],
        torque_lag_s=profile["actuator_dynamic_assumptions"]["torque_lag_s"],
        command_delay_samples=profile["actuator_dynamic_assumptions"]["command_delay_samples"])
    return Scenario(name=f"{profile['name']}_{frequency:g}hz_{amplitude:g}deg", group="motor_profile",
        description=f"{profile['name']}: {frequency:g} Hz / {amplitude:g} deg; actual C wire quantization",
        plant=plant, frequency_hz=frequency, amplitude_deg=amplitude, torque_cap_nm=config["torque_limit_nm"],
        parameter_basis=profile["nominal_model_scope"])


def compare_trial(profile, scenario, original, new, seed, codec_library=None):
    motor = profile["motor"]
    rated_point = ({"rated_velocity_rad_s": motor["rated_speed_rad_s"],
                    "rated_torque_nm": motor["rated_torque_nm"]}
                   if profile["protocol"]["kind"] == "dm_mit" else {})
    envelope = MotorEnvelope(scenario.torque_cap_nm, motor["zero_speed_anchor_nm"],
                             motor["no_load_speed_rad_s"], **rated_point)
    gain = {key: profile["controller"][key] for key in ("k_position", "k_velocity")}
    traces, variants = {}, {}
    baseline_overrides = profile.get("comparison_baseline_controller_overrides", {})
    has_integral_tuning = bool(baseline_overrides)
    implementations = [("Original core", original)]
    if has_integral_tuning:
        implementations.append(("Original core (same Ki)", original))
    implementations.append(("New core", new))
    for label, core in implementations:
        codec = WireCodec(profile, codec_library or new.lib)
        configuration = dict(profile["controller"])
        if label == "Original core":
            configuration.update(baseline_overrides)
        try:
            traces[label], variants[label] = run_trial(core, scenario, gain, seed,
                limiter=lambda torque, velocity: actuator_limit(torque, velocity, envelope),
                quantizer=codec, controller_config=configuration)
        except (RuntimeError, ValueError) as error:
            traces[label] = np.empty((0, len(COLUMNS)+len(EXTRA_COLUMNS)))
            variants[label] = {"completed": False, "failure": str(error), "first_latched_fault": None}
        variants[label]["wire_codec"] = codec.metrics()
    old, updated = variants["Original core"], variants["New core"]
    change = ((updated["position_rmse_deg"]/old["position_rmse_deg"]-1)*100
              if old["completed"] and updated["completed"] and old.get("position_rmse_deg", 0) else None)
    same_tuned = variants.get("Original core (same Ki)")
    same_tuned_change = ((updated["position_rmse_deg"]/same_tuned["position_rmse_deg"]-1)*100
                        if same_tuned and same_tuned["completed"] and updated["completed"]
                        and same_tuned.get("position_rmse_deg", 0) else None)
    return traces, {"seed": seed, "variants": variants, "new_rmse_change_percent": change,
                   "new_rmse_change_vs_same_tuned_upstream_percent": same_tuned_change}


def summarize_paired(paired, planned_count):
    changes = [item["new_rmse_change_percent"] for item in paired if item["new_rmse_change_percent"] is not None]
    summary = {
        "completed_pair_count": len(changes), "planned_pair_count": planned_count,
        "lower_rmse_pair_count": sum(change < 0 for change in changes),
        "mean_paired_rmse_change_percent": float(np.mean(changes)) if changes else None,
        "worst_paired_rmse_change_percent": max(changes) if changes else None,
        "best_paired_rmse_change_percent": min(changes) if changes else None,
    }
    for label in ("Original core", "Original core (same Ki)", "New core"):
        if not all(label in item["variants"] for item in paired):
            continue
        metrics = [item["variants"][label] for item in paired]
        summary[label] = {key: (float(np.mean([item[key] for item in metrics]))
            if all(key in item for item in metrics) else None)
            for key in ("position_rmse_deg", "position_max_abs_error_deg", "command_rms_nm",
                        "actual_torque_rms_nm", "command_at_limit_fraction", "envelope_limited_interval_fraction")}
    same_tuned = [item.get("new_rmse_change_vs_same_tuned_upstream_percent") for item in paired]
    summary["mean_change_vs_same_tuned_upstream_percent"] = (
        float(np.mean(same_tuned)) if same_tuned and all(value is not None for value in same_tuned) else None)
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT/"results/motor_profiles")
    parser.add_argument("--upstream-source", type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    original, new = UpstreamCore(args.upstream_source), Core()
    profiles = [json.loads((ROOT/"profiles"/name).read_text()) for name in ("dm4310_24v.json", "gm6020_current.json")]
    for profile in profiles:
        if not profile["simulation_only"] or profile["dt_s"] != DT:
            raise ValueError("these experiments require explicit simulation-only 1 ms profiles")
        config_fields = {name for name, _ in Config._fields_}
        if set(profile["controller"]) != config_fields:
            raise ValueError("profile controller keys must exactly match the current C configuration")
        if not set(profile.get("comparison_baseline_controller_overrides", {})) <= config_fields:
            raise ValueError("unknown baseline override configuration field")
    report = {
        "data_type": "SYNTHETIC; actual C controllers and actual C protocol quantization, no motor traffic",
        "primary_case_preselected": {"frequency_hz": 2., "amplitude_deg": 5., "paired_seeds": list(PRIMARY_SEEDS)},
        "validation_case_preselected": {"frequency_hz": 2., "amplitude_deg": 5., "paired_seeds": list(VALIDATION_SEEDS),
                                       "role": "held out from candidate selection"},
        "other_preselected_trajectories": [list(item) for item in TRAJECTORIES],
        "common_load_scope": "same assumed J=.039/B=.30 plus constant +.15 N.m and smooth Coulomb .06 N.m; not a motor-specific typical gimbal",
        "motor_comparison_scope": "assess original/new within each motor; differing motor limits and sensors are not a controller-treatment difference",
        "integral_comparison_scope": "when a profile enables integral, Original core is the preserved Ki0 baseline and Original core (same Ki) gets the identical tuned integral; distinguish configuration improvement from core-only improvement",
        "envelope_scope": "DM piecewise line through assumed (0,7 N.m), rated (120 rpm,3 N.m), no-load (200 rpm,0); GM source-derived 320/156 N.m zero-speed intercept; current cap and braking-only cap; no thermal/regeneration model",
        "protocol_scope": "wire torque is passed to the delayed actuator; observer still uses last controller command; DM zero torque is not exactly representable",
        "feedback_scope": "zero-centered uniform quantization after an assumed calibration, not bit-exact feedback CAN replay; raw DM MIT zero lies at a half-bin",
        "clock": {"dt_s": DT, "duration_s": DURATION, "startup_s": RAMP, "feedback_age_s": 0, "reference_age_s": 0, "applied_torque_valid": False},
        "upstream": original.metadata, "new_core_source_sha256": file_hash(ROOT/"src/yaw_controller.c"),
        "shared_library_sha256": file_hash(new.path), "codec_abi_sizes_bytes": verify_codec_layout(),
        "source_sha256": {name: file_hash(ROOT/name) for name in (
            "sim/run_motor_profiles.py", "sim/run_sensitivity.py", "sim/motor_envelope.py",
            "src/dm_mit.c", "src/gm6020.c", "include/dm_mit.h", "include/gm6020.h",
            "include/yaw_controller.h", "sim/c_core.py", "sim/upstream_reference.py", "tools/fetch_upstream.py",
            "profiles/dm4310_24v.json", "profiles/gm6020_current.json")},
        "profiles": {},
        "dependencies": {"python": platform.python_version(), "numpy": np.__version__,
                         "scipy": scipy.__version__, "matplotlib": matplotlib.__version__},
    }
    for profile in profiles:
        name = profile["name"]
        entry = {"profile": profile, "cases": {}, "primary_paired_seeds": [], "validation_paired_seeds": []}
        report["profiles"][name] = entry
        for frequency, amplitude in TRAJECTORIES:
            scenario = make_scenario(profile, frequency, amplitude)
            traces, result = compare_trial(profile, scenario, original, new, PRIMARY_SEEDS[0])
            omega = math.tau*frequency
            result["linear_required_torque_amplitude_nm"] = math.radians(amplitude)*math.hypot(
                scenario.plant.inertia_kg_m2*omega**2, scenario.plant.damping_nm_s_rad*omega)
            result["linear_feedback_diagnostics"] = linear_feedback_diagnostics(scenario.plant,
                {key: profile["controller"][key] for key in ("k_position", "k_velocity")})
            entry["cases"][scenario.name] = result
            old, updated = result["variants"]["Original core"], result["variants"]["New core"]
            print(f"{scenario.name}: {show(old.get('position_rmse_deg'))} -> "
                  f"{show(updated.get('position_rmse_deg'))} deg; change "
                  f"{change_text(result['new_rmse_change_percent'])}; new cap="
                  f"{show(updated.get('command_at_limit_fraction'), '.1%')}", flush=True)
            plot_tracking(args.output/scenario.name, scenario, traces)
            if (frequency, amplitude) == (2., 5.):
                entry["primary_paired_seeds"].append(result)
                write_trace(args.output/f"{scenario.name}.csv.gz", traces)
            (args.output/"metrics.json").write_text(json.dumps(report, indent=2, allow_nan=False)+"\n")
        scenario = make_scenario(profile, 2., 5.)
        for seed in PRIMARY_SEEDS[1:]:
            _, result = compare_trial(profile, scenario, original, new, seed)
            entry["primary_paired_seeds"].append(result)
            print(f"{name} primary seed {seed}: change {change_text(result['new_rmse_change_percent'])}", flush=True)
        entry["primary_summary"] = summarize_paired(entry["primary_paired_seeds"], len(PRIMARY_SEEDS))
        for seed in VALIDATION_SEEDS:
            _, result = compare_trial(profile, scenario, original, new, seed)
            entry["validation_paired_seeds"].append(result)
            print(f"{name} held-out seed {seed}: change {change_text(result['new_rmse_change_percent'])}", flush=True)
        entry["validation_summary"] = summarize_paired(entry["validation_paired_seeds"], len(VALIDATION_SEEDS))
        (args.output/"metrics.json").write_text(json.dumps(report, indent=2, allow_nan=False)+"\n")
    lines = ["# Two motor profiles — actual C controllers and C wire quantization", "",
        "SYNTHETIC, same assumed load and disturbances. Compare original/new within each motor; do not interpret motor-to-motor differences as an algorithm advantage.",
        "The primary case was fixed before running: 2 Hz / 5 deg, development seeds 4310–4314 and held-out validation seeds 4315–4319. Every planned seed and all four trajectories are retained.",
        "For an integral-enabled profile, Original core preserves Ki0; Original core (same Ki) runs identical integral tuning. Gains from adding integral must not be presented as a pure core-implementation advantage.", "",
        "| Motor / trajectory (seed 4310) | Original Ki0 RMSE (deg) | Original same-Ki RMSE (deg) | New RMSE (deg) | Change vs Ki0 | New command cap / envelope |",
        "| --- | ---: | ---: | ---: | ---: | ---: |"]
    for name, entry in report["profiles"].items():
        for case_name, case in entry["cases"].items():
            old, updated = case["variants"]["Original core"], case["variants"]["New core"]
            tuned = case["variants"].get("Original core (same Ki)", {})
            lines.append(f"| {case_name} | {show(old.get('position_rmse_deg'))} | {show(tuned.get('position_rmse_deg'))} | {show(updated.get('position_rmse_deg'))} | "
                f"{change_text(case['new_rmse_change_percent'])} | {show(updated.get('command_at_limit_fraction'), '.1%')} / "
                f"{show(updated.get('envelope_limited_interval_fraction'), '.1%')} |")
    for group, summary_key, group_label in (("primary_paired_seeds", "primary_summary", "Development"),
                                            ("validation_paired_seeds", "validation_summary", "Held-out validation")):
        lines += ["", f"{group_label} — preselected 2 Hz / 5 deg:", "",
            "| Motor / seed | Original Ki0 / new RMSE (deg) | Original / new max error (deg) | Original / new command RMS (N.m) | Command cap: old / new | Envelope clipping: old / new | Change vs Ki0 |",
            "| --- | ---: | ---: | ---: | ---: | ---: | ---: |"]
        for name, entry in report["profiles"].items():
            for item in entry[group]:
                old, updated = item["variants"]["Original core"], item["variants"]["New core"]
                lines.append(f"| {name} / {item['seed']} | {show(old.get('position_rmse_deg'))} / {show(updated.get('position_rmse_deg'))} | "
                    f"{show(old.get('position_max_abs_error_deg'))} / {show(updated.get('position_max_abs_error_deg'))} | "
                    f"{show(old.get('command_rms_nm'))} / {show(updated.get('command_rms_nm'))} | "
                    f"{show(old.get('command_at_limit_fraction'), '.1%')} / {show(updated.get('command_at_limit_fraction'), '.1%')} | "
                    f"{show(old.get('envelope_limited_interval_fraction'), '.1%')} / {show(updated.get('envelope_limited_interval_fraction'), '.1%')} | "
                    f"{change_text(item['new_rmse_change_percent'])} |")
        for name, entry in report["profiles"].items():
            summary = entry[summary_key]
            lines += ["", f"{name}: {summary['lower_rmse_pair_count']}/{summary['planned_pair_count']} seeds have lower RMSE; "
                f"mean paired change {change_text(summary['mean_paired_rmse_change_percent'])}, worst change {change_text(summary['worst_paired_rmse_change_percent'])}."]
        tuned_entries = [(name, entry) for name, entry in report["profiles"].items()
                         if "Original core (same Ki)" in entry[summary_key]]
        if tuned_entries:
            lines += ["", f"{group_label} — isolate the same-integral implementation comparison:", "",
                      "| Motor / seed | Same-Ki original RMSE (deg) | New RMSE (deg) | Change vs same-Ki original |",
                      "| --- | ---: | ---: | ---: |"]
            for name, entry in tuned_entries:
                for item in entry[group]:
                    tuned, updated = item["variants"]["Original core (same Ki)"], item["variants"]["New core"]
                    lines.append(f"| {name} / {item['seed']} | {show(tuned.get('position_rmse_deg'))} | "
                        f"{show(updated.get('position_rmse_deg'))} | {change_text(item['new_rmse_change_vs_same_tuned_upstream_percent'])} |")
                lines += ["", f"{name}: mean change vs same-Ki original "
                          f"{change_text(entry[summary_key]['mean_change_vs_same_tuned_upstream_percent'])}."]
    lines += ["Negative change means lower error. Five paired synthetic seeds are a sensitivity check, not a confidence interval or hardware-performance proof.",
        "All current/torque conversion uses this repository's real C codecs. GM6020 current mode is assumed confirmed only in this offline profile; no CAN is sent.",
        "The 2 ms actuator lag is an explicit assumption and is not the GM6020 datasheet's 3 ms mechanical time constant.",
        "Feedback uses an assumed calibrated, zero-centered uniform quantizer; only command conversion uses the actual C protocol. This is not bit-exact feedback CAN replay.",
        "The torque caps do not establish all-speed continuous or stall thermal feasibility. CSVs retain the primary seed-4310 trajectories, and JSON retains every seed's error, command RMS, saturation and wire metrics."]
    (args.output/"README.md").write_text("\n".join(lines)+"\n")
    if any(not variant["completed"] for entry in report["profiles"].values()
           for trial in list(entry["cases"].values())+entry["primary_paired_seeds"]+entry["validation_paired_seeds"]
           for variant in trial["variants"].values()):
        raise SystemExit("Incomplete trials retained in metrics.json; no failed cases were discarded.")


if __name__ == "__main__":
    main()
