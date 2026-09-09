"""Actual C gimbal API binding; verify all new ABI fields with a C compiler."""
import ctypes as ct
from functools import lru_cache
import os
from pathlib import Path
import shlex
import subprocess
import tempfile

from sim.c_core import Config, Controller, Core, Feedback, Output, Reference, ROOT


class GimbalConfig(ct.Structure):
    _fields_ = [("control", Config)] + [(name, ct.c_float) for name in (
        "gravity_cos_nm gravity_sin_nm joint_min_rad joint_max_rad joint_margin_rad").split()] + [
        ("pitch_enabled", ct.c_bool)]


class Pose(ct.Structure):
    _fields_ = [(name, ct.c_float) for name in (
        "joint_position_rad joint_velocity_rad_s gravity_angle_rad age_s").split()] + [
        ("valid", ct.c_bool)]


class GimbalController(ct.Structure):
    _fields_ = [("core", Controller), ("config", GimbalConfig),
                ("last_gravity_nm", ct.c_float), ("initialized", ct.c_bool)]


class GimbalOutput(ct.Structure):
    _fields_ = [("control", Output), ("gravity_feedforward_nm", ct.c_float),
                ("joint_reference_rad", ct.c_float)]


@lru_cache(maxsize=1)
def verify_gimbal_layout():
    classes = {"GimbalConfig": GimbalConfig, "GimbalPose": Pose,
               "GimbalController": GimbalController, "GimbalOutput": GimbalOutput}
    expected = {}
    lines = ['#include <stddef.h>', '#include <stdio.h>', '#include "gimbal_controller.h"',
             'int main(void) {']
    for name, cls in classes.items():
        expected[name] = ct.sizeof(cls)
        lines.append(f'printf("{name} %zu\\n", sizeof({name}));')
        for field, _ in cls._fields_:
            key = name+"."+field
            expected[key] = getattr(cls, field).offset
            lines.append(f'printf("{key} %zu\\n", offsetof({name}, {field}));')
    lines.append('return 0; }')
    with tempfile.TemporaryDirectory(prefix="pitch-abi-") as directory:
        source, binary = Path(directory)/"probe.c", Path(directory)/"probe"
        source.write_text("\n".join(lines)+"\n")
        subprocess.run(shlex.split(os.environ.get("CC", "cc"))+[
            "-std=c11", "-I", str(ROOT/"include"), str(source), "-o", str(binary)],
            check=True, capture_output=True, text=True)
        actual = dict((key, int(value)) for key, value in (line.split() for line in
            subprocess.check_output([str(binary)], text=True).splitlines()))
    if actual != expected:
        raise RuntimeError("C / ctypes gimbal ABI mismatch")
    return {name: ct.sizeof(cls) for name, cls in classes.items()}


class PitchCore:
    def __init__(self, library=None):
        self.base = Core(library)
        self.lib, self.path = self.base.lib, self.base.path
        self.layout = verify_gimbal_layout()
        self.lib.gimbal_controller_init.argtypes = [ct.POINTER(GimbalController),
                                                   ct.POINTER(GimbalConfig)]
        self.lib.gimbal_controller_init.restype = ct.c_bool
        self.lib.gimbal_controller_step.argtypes = [ct.POINTER(GimbalController),
            ct.POINTER(Feedback), ct.POINTER(Reference), ct.POINTER(Pose), ct.c_float,
            ct.POINTER(GimbalOutput)]
        self.lib.gimbal_controller_step.restype = ct.c_int

    def init(self, config):
        controller = GimbalController()
        if not self.lib.gimbal_controller_init(ct.byref(controller), ct.byref(config)):
            raise ValueError("C gimbal controller rejected pitch configuration")
        return controller

    def step(self, controller, feedback, reference, pose, dt):
        output = GimbalOutput()
        status = self.lib.gimbal_controller_step(ct.byref(controller), ct.byref(feedback),
            ct.byref(reference), ct.byref(pose), dt, ct.byref(output))
        if status != output.control.status:
            raise RuntimeError("C gimbal status/output mismatch")
        return output
