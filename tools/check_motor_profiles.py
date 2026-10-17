#!/usr/bin/env python3
"""Check shipped motor-CAN profiles against protocol units and real C decoders.

This validates the existing motor feedback model, not an external IMU model.
Noise and actuator dynamics remain assumptions; a pass is not hardware approval.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import shlex
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
PROFILE_FILES = ("dm4310_24v.json", "gm6020_current.json")


def number(value, name, positive=False):
    if type(value) not in (int, float) or not math.isfinite(value) or value < 0 or (positive and value == 0):
        raise ValueError(f"{name} must be a finite {'positive' if positive else 'nonnegative'} number")
    return value


def validate_profile(profile):
    """Reject unit/quantization drift before interpreting a motor-CAN experiment."""
    if profile.get("simulation_only") is not True:
        raise ValueError("shipped profiles must explicitly remain simulation-only")
    if profile.get("feedback_source") != "motor_can":
        raise ValueError("this check requires motor_can feedback; external IMU needs its own explicit experiment")
    protocol, sensor = profile["protocol"], profile["sensor"]
    torque = number(protocol["torque_limit_nm"], "protocol torque limit", True)
    if number(profile["controller"]["torque_limit_nm"], "controller torque limit", True) != torque:
        raise ValueError("controller and protocol application torque limits differ")
    if protocol["kind"] == "gm6020_current_v1_4":
        kt = number(protocol["torque_constant_nm_a"], "Kt", True)
        current = number(protocol["current_limit_a"], "current limit", True)
        if (protocol["current_mode_confirmed"] is not True or kt > sys.float_info.max/3
                or current > 3 or torque / kt > 3):
            raise ValueError("GM6020 current mode or protocol range invalid")
        if type(protocol["motor_id"]) is not int or not 1 <= protocol["motor_id"] <= 7:
            raise ValueError("GM6020 motor ID must be 1..7")
        expected = {"position_quantum_rad": math.tau/8192,
                    "velocity_quantum_rad_s": math.tau/60}
    elif protocol["kind"] == "dm_mit":
        pmax = number(protocol["p_max_rad"], "PMAX", True)
        vmax = number(protocol["v_max_rad_s"], "VMAX", True)
        tmax = number(protocol["t_max_nm"], "TMAX", True)
        if torque > tmax:
            raise ValueError("DM application torque exceeds configured TMAX")
        for key, minimum in (("motor_can_id", 1), ("master_can_id", 0)):
            if type(protocol[key]) is not int or not minimum <= protocol[key] <= 0x7ff:
                raise ValueError(f"invalid standard CAN {key}")
        expected = {"position_quantum_rad": 2*pmax/65535,
                    "velocity_quantum_rad_s": 2*vmax/4095}
    else:
        raise ValueError("unsupported motor protocol")
    for key, value in expected.items():
        actual = number(sensor[key], key, True)
        if not math.isclose(actual, value, rel_tol=1e-12, abs_tol=0):
            raise ValueError(f"{key}={actual} disagrees with motor CAN mapping {value}")
    for key in ("position_noise_std_rad", "velocity_noise_std_rad_s"):
        number(sensor[key], key)
    return expected


# Native C structs avoid a second ctypes ABI definition. Probe adjacent codes
# through the production decoder; DM mapping ranges come from the JSON profile.
PROBE = r'''
#include "gm6020.h"
#include "dm_mit.h"
#include <stdio.h>
#include <stdlib.h>
int main(int argc, char **argv) {
    if (argc != 8) return 1;
    const dm_mit_config_t dc = {strtod(argv[1], NULL), strtod(argv[2], NULL),
        strtod(argv[3], NULL), strtod(argv[4], NULL), 1, 17};
    const gm6020_config_t gc = {strtod(argv[5], NULL), strtod(argv[6], NULL), strtod(argv[7], NULL), true};
    if (gm6020_validate_config(&gc) != GM6020_OK) return 6;
    const unsigned char gm_data[8] = {0, 1, 0, 1, 0, 17, 25, 0};
    unsigned char dm_data[8] = {1, 0x80, 0, 0x80, 0, 0, 25, 25};
    gm6020_feedback_t gm;
    dm_mit_feedback_t a, b;
    if (gm6020_decode_feedback(1, 0x205, false, false, gm_data, 8, &gm) != GM6020_OK) return 2;
    if (dm_mit_decode_feedback(&dc, 17, false, false, dm_data, 8, &a) != DM_MIT_OK) return 3;
    dm_data[2] = 1; dm_data[4] = 0x10;
    if (dm_mit_decode_feedback(&dc, 17, false, false, dm_data, 8, &b) != DM_MIT_OK) return 4;
    if (gm.current_raw != 17 || !gm.valid || !a.valid || !b.valid) return 5;
    printf("%.17g %.17g %.17g %.17g\n", gm.position_rad, gm.velocity_rad_s,
        b.position_rad-a.position_rad, b.velocity_rad_s-a.velocity_rad_s);
    return 0;
}
'''


def check_profiles():
    profiles = {name: json.loads((ROOT/"profiles"/name).read_text()) for name in PROFILE_FILES}
    expected = {name: validate_profile(p) for name, p in profiles.items()}
    dm = profiles["dm4310_24v.json"]["protocol"]
    gm = profiles["gm6020_current.json"]["protocol"]
    with tempfile.TemporaryDirectory(prefix="motor-profile-check-") as directory:
        source, binary = Path(directory)/"probe.c", Path(directory)/"probe"
        source.write_text(PROBE)
        subprocess.run(shlex.split(os.environ.get("CC", "cc"))+["-std=c11", "-Wall", "-Wextra", "-Werror",
            "-I", str(ROOT/"include"), str(source), str(ROOT/"src/gm6020.c"),
            str(ROOT/"src/dm_mit.c"), "-lm", "-o", str(binary)], check=True, capture_output=True, text=True)
        values = list(map(float, subprocess.check_output([str(binary)]+[
            str(dm[key]) for key in ("p_max_rad", "v_max_rad_s", "t_max_nm", "torque_limit_nm")]+[
            str(gm[key]) for key in ("torque_constant_nm_a", "current_limit_a", "torque_limit_nm")], text=True).split()))
    decoded = dict(zip(("gm6020_current.json", "dm4310_24v.json"), (values[:2], values[2:])))
    for name, quantities in expected.items():
        for value, actual in zip(quantities.values(), decoded[name]):
            if not math.isfinite(actual) or not math.isclose(value, actual, rel_tol=1e-10, abs_tol=0):
                raise ValueError(f"{name}: C feedback decoder disagrees with protocol/profile")
    sources = ["tools/check_motor_profiles.py", "src/gm6020.c", "src/dm_mit.c",
               "include/gm6020.h", "include/dm_mit.h"]+["profiles/"+name for name in PROFILE_FILES]
    return {"passed": True, "scope": "offline shipped motor-CAN profile and C decoder consistency; no traffic or hardware claim",
        "profiles": {name: {"expected": expected[name], "c_decoder_steps": decoded[name]} for name in PROFILE_FILES},
        "source_sha256": {name: hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in sources}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    report = check_profiles()
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2, allow_nan=False)+"\n")
    print("Motor CAN profiles match protocol quantization and real C feedback decoders (2/2).")


if __name__ == "__main__":
    main()
