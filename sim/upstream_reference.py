"""Run the pinned, UNMODIFIED upstream C through a small field-mapping bridge.

No Python controller reimplementation, input sanitization, age checking, or
replacement of upstream NaN results occurs. The upstream API has no status/fault
return: every mapped Output.status is UPSTREAM_STATUS_UNAVAILABLE (-1), including
its startup zero and invalid-feedback zero. Do not treat it as a fault code.

Both controllers receive the same model, feedforward, feedback gains, ESO
bandwidth/gain and command limits. The old implementation has no disturbance
state bound, compensation slew limit, anti-windup, or latched fault contract;
this bridge deliberately does not add them. Integral gain maps from Config;
use k_integral=0 in the primary comparison. The separate old torque-bias loop is
always disabled. Source files are verified and cached only under build/upstream.
"""
from __future__ import annotations

import argparse
import ctypes as ct
import hashlib
import json
import os
from pathlib import Path
import shlex
import subprocess
import tempfile
import weakref

from sim.c_core import Config, Feedback, Reference, Output, ROOT, verify_layout
from tools.fetch_upstream import (UPSTREAM_COMMIT, UPSTREAM_SHA256, atomic_write,
                                  fetch_upstream, verify_source)

UPSTREAM_STATUS_UNAVAILABLE = -1

# Independently written glue. The original .c and .h are separate compiler inputs
# and are never patched, concatenated here, or copied into published source.
BRIDGE_SOURCE = r'''
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include "yaw_controller.h"
#include "yaw_auto_lqr_eso_controller.h"

typedef struct {
    YawAutoLqrEso_t controller;
    YawAutoLqrEsoConfig_t config;
} UpstreamHandle;

void *upstream_create(const YawConfig *new_config)
{
    if (new_config == NULL) return NULL;
    UpstreamHandle *h = calloc(1, sizeof(*h));
    if (h == NULL) return NULL;
    YawAutoLqrEsoConfig_t *c = &h->config;
    c->j_kg_m2 = new_config->inertia_kg_m2;
    c->b_nms_rad = new_config->damping_nm_s_rad;
    c->k_theta = new_config->k_position;
    c->k_omega = new_config->k_velocity;
    c->k_i = new_config->k_integral;
    c->theta_integral_limit_rad_s = new_config->k_integral > 0.0f ?
        new_config->integral_limit_nm / new_config->k_integral : 0.0f;
    c->tau_coulomb_nm = new_config->coulomb_nm;
    c->coulomb_smooth_rad_s = new_config->coulomb_velocity_rad_s;
    c->eso_bandwidth_rad_s = new_config->eso_bandwidth_rad_s;
    c->eso_comp_gain = new_config->eso_gain;
    c->eso_comp_limit_nm = new_config->compensation_limit_nm;
    c->eso_omega_gate_rad_s = 0.0f;
    c->eso_alpha_gate_rad_s2 = 0.0f;
    c->tau_bias_ki = 0.0f;
    c->tau_bias_limit_nm = 0.0f;
    c->tau_meas_lpf_alpha = 1.0f;
    c->theta_deadband_rad = 0.0f;
    c->torque_soft_limit_nm = 0.0f;
    c->torque_min_nm = -new_config->torque_limit_nm;
    c->torque_max_nm = new_config->torque_limit_nm;
    c->torque_slew_rate_nm_s = new_config->torque_slew_nm_s;
    c->eso_enable = new_config->eso_bandwidth_rad_s > 0.0f ? 1U : 0U;
    c->eso_comp_enable = new_config->eso_gain != 0.0f ? 1U : 0U;
    c->torque_slew_enable = 1U;
    YawAutoLqrEso_Init(&h->controller);
    return h;
}

void upstream_destroy(void *handle)
{
    free(handle);
}

void upstream_set_torque_limit(void *handle, float limit_nm)
{
    UpstreamHandle *h = handle;
    h->config.torque_min_nm = -limit_nm;
    h->config.torque_max_nm = limit_nm;
}

void upstream_step(void *handle, const YawFeedback *feedback,
                   const YawReference *reference, float dt, YawOutput *output)
{
    UpstreamHandle *h = handle;
    const YawAutoLqrEsoFeedback_t f = {
        .theta_rad = feedback->position_rad,
        .omega_rad_s = feedback->velocity_rad_s,
        .tau_meas_nm = feedback->applied_torque_valid ? feedback->applied_torque_nm : 0.0f,
        .feedback_ok = feedback->valid ? 1U : 0U,
    };
    const YawAutoLqrEsoReference_t r = {
        .theta_rad = reference->position_rad,
        .omega_rad_s = reference->velocity_rad_s,
        .alpha_rad_s2 = reference->acceleration_rad_s2,
    };
    YawAutoLqrEsoOutput_t out;
    YawAutoLqrEso_Calc(&h->controller, &h->config, &f, &r, dt, &out);
    *output = (YawOutput){0};
    output->torque_nm = out.tau_cmd_nm;
    output->unconstrained_torque_nm = out.tau_pre_limit_nm;
    output->feedforward_nm = out.tau_ff_nm;
    output->feedback_nm = out.tau_lqr_nm - out.tau_ff_nm - out.tau_i_nm;
    output->integral_nm = out.tau_i_nm;
    output->compensation_nm = out.tau_eso_active_nm;
    /* z3 is acceleration; divide by b0 to report estimated disturbance in N.m.
     * Preserve NaN/Inf when the original observer diverges. Only disabled or
     * invalid-model b0==0 has no torque estimate and is represented by zero. */
    output->disturbance_nm = h->controller.eso.b0 == 0.0f ? 0.0f :
        h->controller.eso.z3 / h->controller.eso.b0;
    output->position_error_rad = -out.e_theta_rad;
    output->velocity_error_rad_s = -out.e_omega_rad_s;
    if (out.soft_limit_active || out.hard_limit_active)
        output->flags |= YAW_TORQUE_LIMITED;
    if (out.slew_limit_active) output->flags |= YAW_SLEW_LIMITED;
    output->status = (YawStatus)-1; /* No counterpart in the upstream API. */
}
'''


def _digest(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


class UpstreamController:
    """Own one original C controller/config pair without guessing its ABI."""
    def __init__(self, core, pointer):
        self._core = core
        self._pointer = ct.c_void_p(pointer)
        self._finalizer = weakref.finalize(self, core.lib.upstream_destroy, self._pointer)

    def close(self):
        self._finalizer()


class UpstreamCore:
    def __init__(self, source_dir=None, library=None):
        self.layout = verify_layout()
        self.source_dir = fetch_upstream(source_dir)
        metadata = verify_source(self.source_dir)
        compiler = shlex.split(os.environ.get("CC", "cc"))
        if not compiler:
            raise ValueError("CC must name a C compiler")
        compiler_version = subprocess.check_output(compiler + ["--version"], text=True).splitlines()[0]
        options = ["-std=c99", "-O2", "-fno-fast-math", "-Wall", "-Wextra", "-Werror",
                   "-fPIC", "-shared"]
        build_inputs = {
            "upstream_commit": UPSTREAM_COMMIT,
            "upstream_sha256": UPSTREAM_SHA256,
            "bridge_sha256": _digest(BRIDGE_SOURCE.encode()),
            "interface_sha256": _digest((ROOT / "include/yaw_controller.h").read_bytes()),
            "compiler": compiler,
            "compiler_version": compiler_version,
            "options": options,
        }
        build_key = _digest(json.dumps(build_inputs, sort_keys=True).encode())
        build_dir = self.source_dir.parent
        self.path = Path(library).expanduser().resolve() if library else (
            build_dir / f"libupstream_reference_{build_key[:16]}.so")
        manifest_path = self.path.with_suffix(self.path.suffix + ".json")
        if self.path.is_file():
            if not manifest_path.is_file():
                raise ValueError(f"Missing provenance manifest for prebuilt bridge: {manifest_path}")
            stored = json.loads(manifest_path.read_text())
            if stored.get("build_key") != build_key:
                raise ValueError(f"Prebuilt bridge inputs do not match the pinned comparison: {self.path}")
            if stored.get("library_sha256") != _digest(self.path.read_bytes()):
                raise ValueError(f"Prebuilt bridge checksum mismatch: {self.path}")
        else:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            with tempfile.TemporaryDirectory(prefix="upstream-bridge-", dir=self.path.parent) as temp:
                bridge = Path(temp) / "bridge.c"
                binary = Path(temp) / "bridge.so"
                bridge.write_text(BRIDGE_SOURCE)
                command = compiler + options + ["-I", str(ROOT / "include"), "-I", str(self.source_dir),
                    str(bridge), str(self.source_dir / "yaw_auto_lqr_eso_controller.c"),
                    "-lm", "-o", str(binary)]
                subprocess.run(command, check=True, capture_output=True, text=True)
                payload = binary.read_bytes()
                stored = dict(build_inputs, build_key=build_key, library_sha256=_digest(payload))
                # Publish the manifest first: a simultaneous reader must never
                # observe a newly built library without its provenance sidecar.
                atomic_write(manifest_path, (json.dumps(stored, indent=2, sort_keys=True) + "\n").encode())
                atomic_write(self.path, payload)
        self.metadata = dict(metadata, **stored, library=str(self.path),
            source_unmodified=True, status_unavailable=UPSTREAM_STATUS_UNAVAILABLE,
            comparison_contract={
                "same_config_fields": ["J", "B", "K_position", "K_velocity", "K_integral",
                    "Coulomb", "ESO bandwidth/gain", "ESO compensation magnitude limit",
                    "hard torque limit", "torque slew"],
                "old_bias_gain": 0, "old_soft_limit": 0, "old_deadband": 0,
                "old_eso_gates": "disabled", "old_eso_input": "previous limited command",
                "unsupported_old_features": ["feedback/reference age", "reference valid flag",
                    "latched faults", "position/velocity/tracking guards", "disturbance state limit",
                    "compensation slew", "integral anti-windup", "measured applied torque in ESO"],
                "flags": "Only original soft/hard limit and slew flags map to bits 0 and 1",
                "integral": "Config k_integral maps directly; use zero for the primary comparison",
                "nonfinite_policy": "Do not filter original inputs or outputs",
            })
        self.lib = ct.CDLL(str(self.path))
        self.lib.upstream_create.argtypes = [ct.POINTER(Config)]
        self.lib.upstream_create.restype = ct.c_void_p
        self.lib.upstream_destroy.argtypes = [ct.c_void_p]
        self.lib.upstream_destroy.restype = None
        self.lib.upstream_step.argtypes = [ct.c_void_p, ct.POINTER(Feedback),
            ct.POINTER(Reference), ct.c_float, ct.POINTER(Output)]
        self.lib.upstream_step.restype = None
        self.lib.upstream_set_torque_limit.argtypes = [ct.c_void_p, ct.c_float]
        self.lib.upstream_set_torque_limit.restype = None

    def _handle(self, controller):
        if controller._core is not self or not controller._finalizer.alive:
            raise ValueError("Controller handle is closed or belongs to a different core")
        return controller._pointer

    def init(self, config):
        pointer = self.lib.upstream_create(ct.byref(config))
        if not pointer:
            raise MemoryError("Could not allocate the original C controller")
        return UpstreamController(self, pointer)

    def step(self, controller, feedback, reference, dt):
        output = Output()
        self.lib.upstream_step(self._handle(controller), ct.byref(feedback),
                               ct.byref(reference), dt, ct.byref(output))
        return output

    def set_torque_limit(self, controller, value):
        """Directly change the old hard limits; no new safety behavior is added."""
        self.lib.upstream_set_torque_limit(self._handle(controller), value)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-dir", type=Path)
    parser.add_argument("--library", type=Path)
    args = parser.parse_args()
    core = UpstreamCore(args.source_dir, args.library)
    print(json.dumps(core.metadata, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
