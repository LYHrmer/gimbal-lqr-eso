"""ctypes binding, with C-compiler checks of every struct size and field offset."""
import ctypes as ct
import json
import os
from pathlib import Path
import shlex
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]


class Config(ct.Structure):
    _fields_ = [(name, ct.c_float) for name in (
        "inertia_kg_m2 damping_nm_s_rad k_position k_velocity k_integral "
        "integral_limit_nm antiwindup_rate_s coulomb_nm coulomb_velocity_rad_s "
        "eso_bandwidth_rad_s eso_gain disturbance_limit_nm compensation_limit_nm "
        "compensation_slew_nm_s torque_limit_nm torque_slew_nm_s dt_min_s dt_max_s "
        "feedback_timeout_s reference_timeout_s position_min_rad position_max_rad "
        "velocity_limit_rad_s tracking_error_limit_rad velocity_error_filter_tau_s").split()]


class Feedback(ct.Structure):
    _fields_ = [(name, ct.c_float) for name in (
        "position_rad velocity_rad_s age_s applied_torque_nm").split()] + [
        ("valid", ct.c_bool), ("applied_torque_valid", ct.c_bool)]


class Reference(ct.Structure):
    _fields_ = [(name, ct.c_float) for name in (
        "position_rad velocity_rad_s acceleration_rad_s2 age_s").split()] + [
        ("valid", ct.c_bool)]


class Output(ct.Structure):
    _fields_ = [(name, ct.c_float) for name in (
        "torque_nm unconstrained_torque_nm feedforward_nm feedback_nm integral_nm "
        "compensation_nm disturbance_nm position_error_rad velocity_error_rad_s").split()] + [
        ("flags", ct.c_uint32), ("status", ct.c_int)]


class Controller(ct.Structure):
    _fields_ = [("config", Config)] + [(name, ct.c_float) for name in (
        "observer_position_rad observer_velocity_rad_s observer_disturbance_nm "
        "integral_nm compensation_nm last_torque_nm last_velocity_rad_s "
        "velocity_error_filtered_rad_s").split()] + [
        ("initialized", ct.c_bool), ("ready", ct.c_bool), ("fault", ct.c_int)]


def verify_layout():
    """Fail before loading a controller if bool/alignment/enum assumptions differ."""
    classes = {"YawConfig": Config, "YawFeedback": Feedback, "YawReference": Reference,
               "YawOutput": Output, "YawController": Controller}
    lines = ['#include <stddef.h>', '#include <stdio.h>', '#include "yaw_controller.h"',
             'int main(void) {']
    expected = {"sizeof.bool": ct.sizeof(ct.c_bool), "sizeof.YawStatus": ct.sizeof(ct.c_int),
                "sizeof.float": ct.sizeof(ct.c_float), "sizeof.uint32_t": ct.sizeof(ct.c_uint32)}
    for typename in ("bool", "YawStatus", "float", "uint32_t"):
        lines.append(f'printf("sizeof.{typename} %zu\\n", sizeof({typename}));')
    for name, cls in classes.items():
        expected[f"sizeof.{name}"] = ct.sizeof(cls)
        lines.append(f'printf("sizeof.{name} %zu\\n", sizeof({name}));')
        for field, _ in cls._fields_:
            expected[f"{name}.{field}"] = getattr(cls, field).offset
            lines.append(f'printf("{name}.{field} %zu\\n", offsetof({name}, {field}));')
    lines.append('return 0; }')
    with tempfile.TemporaryDirectory(prefix="yaw-abi-") as temp:
        source, binary = Path(temp) / "probe.c", Path(temp) / "probe"
        source.write_text("\n".join(lines) + "\n")
        subprocess.run(shlex.split(os.environ.get("CC", "cc")) + [
            "-std=c99", "-I", str(ROOT / "include"), str(source), "-o", str(binary)],
            check=True, capture_output=True, text=True)
        actual = dict((key, int(value)) for key, value in (
            line.split() for line in subprocess.check_output([str(binary)], text=True).splitlines()))
    if actual != expected:
        raise RuntimeError("C / ctypes ABI mismatch: " + json.dumps({
            key: [expected[key], actual.get(key)] for key in expected if expected[key] != actual.get(key)}))
    return {name: ct.sizeof(cls) for name, cls in classes.items()}


class Core:
    def __init__(self, library=None):
        self.layout = verify_layout()
        self.path = Path(library or ROOT / "build/libyaw_controller.so").resolve()
        if not self.path.is_file():
            raise FileNotFoundError(f"Build the shared C core first; missing {self.path}")
        self.lib = ct.CDLL(str(self.path))
        self.lib.yaw_controller_init.argtypes = [ct.POINTER(Controller), ct.POINTER(Config)]
        self.lib.yaw_controller_init.restype = ct.c_bool
        self.lib.yaw_controller_step.argtypes = [ct.POINTER(Controller), ct.POINTER(Feedback),
            ct.POINTER(Reference), ct.c_float, ct.POINTER(Output)]
        self.lib.yaw_controller_step.restype = ct.c_int
        self.lib.yaw_status_string.argtypes = [ct.c_int]
        self.lib.yaw_status_string.restype = ct.c_char_p

    def init(self, config):
        controller = Controller()
        if not self.lib.yaw_controller_init(ct.byref(controller), ct.byref(config)):
            raise ValueError("C controller rejected simulation configuration")
        return controller

    def step(self, controller, feedback, reference, dt):
        output = Output()
        status = self.lib.yaw_controller_step(ct.byref(controller), ct.byref(feedback),
                                              ct.byref(reference), dt, ct.byref(output))
        if status not in (0, 1):
            raise RuntimeError(f"C controller fault {status}: " +
                               self.lib.yaw_status_string(status).decode("ascii"))
        return output
