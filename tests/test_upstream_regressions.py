#!/usr/bin/env python3
"""Four real-C upstream regressions; ordinary unittest runs never write reports.

python3 -s tests/test_upstream_regressions.py --report results/upstream_regressions.json
python3 -s tests/test_upstream_regressions.py --source-dir /path/to/pinned/upstream

The adapter verifies the fixed upstream source version. The source cache defaults
to build/upstream. Build the new shared library before running these tests.
"""
import argparse
import ctypes as ct
import hashlib
import json
import math
from pathlib import Path
import platform
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from sim.c_core import Config, Controller, Core, Feedback, Output, Reference
from sim.upstream_reference import UPSTREAM_COMMIT, UPSTREAM_SHA256, UpstreamCore

SOURCE_DIR = None
LIBRARY = None
DT = 0.001
YAW_OK = 0
YAW_WARMUP = 1
YAW_BAD_TIMING = 4
YAW_STALE_FEEDBACK = 6
YAW_BAD_REFERENCE = 7


def fixture_config(observer=False):
    """Deterministic diagnostic inputs, not a recommended hardware tuning."""
    return Config(
        inertia_kg_m2=0.039, damping_nm_s_rad=0.30,
        k_position=20.0, k_velocity=2.0,
        k_integral=0.0, integral_limit_nm=0.0, antiwindup_rate_s=20.0,
        coulomb_nm=0.0, coulomb_velocity_rad_s=0.04,
        eso_bandwidth_rad_s=80.0 if observer else 0.0, eso_gain=0.0,
        disturbance_limit_nm=3.0, compensation_limit_nm=2.0,
        compensation_slew_nm_s=100.0, torque_limit_nm=5.0,
        torque_slew_nm_s=100.0, dt_min_s=0.0005, dt_max_s=0.002,
        feedback_timeout_s=0.005, reference_timeout_s=0.005,
        position_min_rad=-math.pi, position_max_rad=math.pi,
        velocity_limit_rad_s=60.0, tracking_error_limit_rad=math.pi,
    )


def feedback(position=0.0, age=0.0):
    return Feedback(position, 0.0, age, 0.0, True, False)


def reference(position=0.25):
    return Reference(position, 0.0, 0.0, 0.0, True)


def json_number(value):
    """Preserve nonfinite outcomes explicitly while emitting strict JSON."""
    value = float(value)
    if math.isnan(value):
        return "NaN"
    if math.isinf(value):
        return "+Infinity" if value > 0.0 else "-Infinity"
    return value


def output_record(output, include_status=True):
    result = {"torque_nm": json_number(output.torque_nm),
              "disturbance_nm": json_number(output.disturbance_nm)}
    if include_status:
        result["status"] = int(output.status)
        result["flags"] = int(output.flags)
    return result


class UpstreamRegressionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.new = Core(LIBRARY)
        cls.old = UpstreamCore(source_dir=SOURCE_DIR)
        cls.new.lib.yaw_controller_set_torque_limit.argtypes = [
            ct.POINTER(Controller), ct.c_float]
        cls.new.lib.yaw_controller_set_torque_limit.restype = ct.c_bool
        cls.records = {}

    def new_step(self, controller, measured, target, dt=DT):
        # Bypass Core.step's intentional exception so fault output is observable.
        output = Output()
        status = self.new.lib.yaw_controller_step(
            ct.byref(controller), ct.byref(measured), ct.byref(target),
            dt, ct.byref(output))
        self.assertEqual(status, output.status)
        return output

    def seeded_pair(self, config, measured, target):
        old_controller = self.old.init(config)
        new_controller = self.new.init(config)
        old_output = self.old.step(old_controller, measured, target, DT)
        new_output = self.new_step(new_controller, measured, target)
        self.assertEqual(old_output.torque_nm, 0.0)
        self.assertEqual(new_output.status, YAW_WARMUP)
        self.assertEqual(new_output.torque_nm, 0.0)
        return old_controller, new_controller

    def assert_latched_zero(self, controller, output, expected_status,
                            measured, target):
        self.assertEqual(output.status, expected_status)
        self.assertEqual(output.torque_nm, 0.0)
        self.assertEqual(controller.fault, expected_status)
        repaired = self.new_step(controller, measured, target)
        self.assertEqual(repaired.status, expected_status)
        self.assertEqual(repaired.torque_nm, 0.0)
        return repaired

    def test_nan_reference(self):
        config = fixture_config()
        measured, target = feedback(), reference()
        old_controller, new_controller = self.seeded_pair(config, measured, target)
        invalid_target = reference(float("nan"))
        old_output = self.old.step(old_controller, measured, invalid_target, DT)
        new_output = self.new_step(new_controller, measured, invalid_target)
        self.assertTrue(math.isnan(old_output.torque_nm))
        repaired = self.assert_latched_zero(
            new_controller, new_output, YAW_BAD_REFERENCE, measured, target)
        self.records["nan_reference"] = {
            "input": {"reference_position_rad": "NaN", "dt_s": DT},
            "upstream": output_record(old_output, include_status=False),
            "new": output_record(new_output),
            "new_after_valid_reference_without_reset": output_record(repaired),
            "assertion": "upstream propagates NaN; new emits zero and latches bad_reference",
        }

    def test_online_torque_derating(self):
        config = fixture_config()
        measured, target = feedback(), reference()
        old_controller, new_controller = self.seeded_pair(config, measured, target)
        for _ in range(60):
            old_before = self.old.step(old_controller, measured, target, DT)
            new_before = self.new_step(new_controller, measured, target)
            self.assertEqual(new_before.status, YAW_OK)
        self.assertAlmostEqual(old_before.torque_nm, 5.0, places=6)
        self.assertAlmostEqual(new_before.torque_nm, 5.0, places=6)
        self.old.set_torque_limit(old_controller, 1.0)
        self.assertTrue(self.new.lib.yaw_controller_set_torque_limit(
            ct.byref(new_controller), 1.0))
        old_output = self.old.step(old_controller, measured, target, DT)
        new_output = self.new_step(new_controller, measured, target)
        self.assertAlmostEqual(old_output.torque_nm, 4.9, places=6)
        self.assertGreater(old_output.torque_nm, 1.0)
        self.assertEqual(new_output.status, YAW_OK)
        self.assertGreater(new_output.torque_nm, 0.0)
        self.assertLessEqual(abs(new_output.torque_nm), 1.0)
        self.assertTrue(new_output.flags & 1)  # Final hard limit is active.
        self.records["online_torque_derating"] = {
            "input": {"old_limit_nm": 5.0, "new_limit_nm": 1.0,
                      "slew_rate_nm_s": 100.0, "dt_s": DT,
                      "warmup_steps": 1, "preconditioning_steps": 60},
            "upstream_before": output_record(old_before, include_status=False),
            "new_before": output_record(new_before),
            "upstream_after": output_record(old_output, include_status=False),
            "new_after": output_record(new_output),
            "assertion": "new hard torque limit takes precedence over slew continuity",
        }

    def test_out_of_contract_timing(self):
        config = fixture_config(observer=True)
        cases = []
        for dt in (0.02, 0.05):
            with self.subTest(dt_s=dt):
                old_controller, new_controller = self.seeded_pair(
                    config, feedback(), reference(0.0))
                measured, target = feedback(0.01), reference(0.01)
                first_new = self.new_step(new_controller, measured, target, dt)
                repaired = self.assert_latched_zero(
                    new_controller, first_new, YAW_BAD_TIMING, measured, target)
                first_large = None
                first_nonfinite = None
                max_finite = 0.0
                first_outputs = []
                last_finite = None
                for step in range(1, 301):
                    old_output = self.old.step(old_controller, measured, target, dt)
                    disturbance = float(old_output.disturbance_nm)
                    if step <= 3:
                        first_outputs.append({"step": step, **output_record(
                            old_output, include_status=False)})
                    if not math.isfinite(disturbance):
                        first_nonfinite = step
                        break
                    magnitude = abs(disturbance)
                    max_finite = max(max_finite, magnitude)
                    last_finite = {"step": step, **output_record(
                        old_output, include_status=False)}
                    if magnitude > 1e6 and first_large is None:
                        first_large = step
                self.assertIsNotNone(first_large)
                self.assertGreater(max_finite, 1e6)
                cases.append({
                    "dt_s": dt,
                    "upstream": {
                        "first_outputs": first_outputs,
                        "steps_executed": step,
                        "first_abs_disturbance_gt_1e6_nm_step": first_large,
                        "first_nonfinite_disturbance_step": first_nonfinite,
                        "max_abs_finite_disturbance_nm": max_finite,
                        "last_finite_output": last_finite,
                        "last_output": output_record(old_output, include_status=False),
                    },
                    "new_first_rejected_step": output_record(first_new),
                    "new_after_valid_dt_without_reset": output_record(repaired),
                })
        self.records["out_of_contract_timing"] = {
            "input": {"observer_bandwidth_rad_s": 80.0, "eso_compensation_gain": 0.0,
                      "measurement_step_rad": 0.01, "velocity_rad_s": 0.0,
                      "new_allowed_dt_s": [float(config.dt_min_s), float(config.dt_max_s)],
                      "maximum_replay_steps": 300},
            "replays": cases,
            "assertion": "upstream observer grows beyond 1e6 Nm; new rejects and latches bad_timing",
            "scope": "fixed measurement replay, not a plant or same-dt closed-loop stability comparison; new does not execute these periods",
        }

    def test_stale_feedback(self):
        config = fixture_config()
        measured, target = feedback(), reference()
        old_controller, new_controller = self.seeded_pair(config, measured, target)
        stale = feedback(age=0.01)
        old_output = self.old.step(old_controller, stale, target, DT)
        new_output = self.new_step(new_controller, stale, target)
        self.assertTrue(math.isfinite(old_output.torque_nm))
        self.assertGreater(old_output.torque_nm, 0.0)
        repaired = self.assert_latched_zero(
            new_controller, new_output, YAW_STALE_FEEDBACK, measured, target)
        self.records["stale_feedback"] = {
            "input": {"feedback_age_s": float(stale.age_s),
                      "feedback_timeout_s": float(config.feedback_timeout_s), "dt_s": DT},
            "upstream": output_record(old_output, include_status=False),
            "new": output_record(new_output),
            "new_after_fresh_feedback_without_reset": output_record(repaired),
            "assertion": "new emits zero and latches stale_feedback",
            "scope": "upstream API has feedback_ok but no age field; the adapter drops age rather than adding a policy",
        }


def report_data():
    config = fixture_config()
    return {
        "data_type": "SYNTHETIC deterministic C API regression replays; no hardware measurements",
        "upstream_repository": "https://github.com/LamdaDay/YAW_Auto_Controller",
        "upstream_commit": UPSTREAM_COMMIT,
        "upstream_expected_source_sha256": UPSTREAM_SHA256,
        "upstream_build": UpstreamRegressionTests.old.metadata,
        "source_verification": "UpstreamCore verifies pinned upstream source before compilation/loading",
        "comparison": "fixed unmodified upstream C via adapter versus this repository's real C core",
        "scope": "four defensive-behavior regressions; does not establish tracking/RMSE superiority or physical stopping",
        "new_status_names": {str(status): UpstreamRegressionTests.new.lib.yaw_status_string(
            status).decode("ascii") for status in (
                YAW_OK, YAW_WARMUP, YAW_BAD_TIMING, YAW_STALE_FEEDBACK, YAW_BAD_REFERENCE)},
        "nonfinite_encoding": "NaN and +/-Infinity are represented by strings, not invalid JSON numbers",
        "new_source_sha256": hashlib.sha256((ROOT / "src/yaw_controller.c").read_bytes()).hexdigest(),
        "new_header_sha256": hashlib.sha256((ROOT / "include/yaw_controller.h").read_bytes()).hexdigest(),
        "new_shared_library_sha256": hashlib.sha256(UpstreamRegressionTests.new.path.read_bytes()).hexdigest(),
        "test_source_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "adapter_source_sha256": hashlib.sha256((ROOT / "sim/upstream_reference.py").read_bytes()).hexdigest(),
        "python": platform.python_version(),
        "base_config": {name: getattr(config, name) for name, _ in Config._fields_},
        "cases": UpstreamRegressionTests.records,
    }


def main():
    global SOURCE_DIR, LIBRARY
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-dir", type=Path,
                        help="offline checkout/cache of the adapter's exact pinned upstream version")
    parser.add_argument("--library", type=Path, help="new shared C core path")
    parser.add_argument("--report", type=Path,
                        help="explicitly write actual results as strict JSON after all tests pass")
    args = parser.parse_args()
    SOURCE_DIR, LIBRARY = args.source_dir, args.library
    suite = unittest.defaultTestLoader.loadTestsFromTestCase(UpstreamRegressionTests)
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    if not result.wasSuccessful():
        return 1
    if args.report is not None:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(report_data(), indent=2, ensure_ascii=False,
                                         allow_nan=False) + "\n", encoding="utf-8")
        print(f"Wrote regression evidence: {args.report}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
