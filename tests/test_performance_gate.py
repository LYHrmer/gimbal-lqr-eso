"""A performance gate must not pass after losing difficult or failed trials."""
import copy
import ctypes as ct
import json
from pathlib import Path
import unittest

from tools.check_performance import MOTORS, SEED_GROUPS, UPSTREAM_COMMIT, check_report


def complete_report():
    old = {"completed": True, "position_rmse_deg": .1, "position_max_abs_error_deg": .2,
           "command_rms_nm": .5, "command_at_limit_fraction": 0.,
           "envelope_limited_interval_fraction": 0., "evaluation_start_s": 2.,
           "evaluation_end_s": 6., "actual_last_sample_s": 6.}
    new = {**old, "position_rmse_deg": .09, "position_max_abs_error_deg": .19,
           "fault_sample_count": 0, "first_latched_fault": None}
    report = {"upstream": {"commit": UPSTREAM_COMMIT, "source_unmodified": True},
              "clock": {"dt_s": .001, "duration_s": 6., "startup_s": 2.},
              "primary_case_preselected": {"frequency_hz": 2., "amplitude_deg": 5.,
                  "paired_seeds": list(SEED_GROUPS["primary_paired_seeds"])},
              "validation_case_preselected": {"frequency_hz": 2., "amplitude_deg": 5.,
                  "paired_seeds": list(SEED_GROUPS["validation_paired_seeds"])}, "profiles": {}}
    root = Path(__file__).resolve().parents[1]
    for motor, filename in zip(MOTORS, ("dm4310_24v.json", "gm6020_current.json")):
        profile = json.loads((root/"profiles"/filename).read_text())
        new_config = profile["controller"]
        old_config = {**new_config, **profile["comparison_baseline_controller_overrides"]}
        variants = {"Original core": {**old, "actual_c_config":
                        {key: ct.c_float(value).value for key, value in old_config.items()}},
                    "New core": {**new, "actual_c_config":
                        {key: ct.c_float(value).value for key, value in new_config.items()}}}
        if new_config["k_integral"] > 0:
            variants["Original core (same Ki)"] = {**old, "actual_c_config":
                {key: ct.c_float(value).value for key, value in new_config.items()}}
        report["profiles"][motor] = {"profile": profile,
            **{group: [{"seed": seed, "variants": copy.deepcopy(variants)} for seed in seeds]
               for group, seeds in SEED_GROUPS.items()}}
    return report


class PerformanceGateTests(unittest.TestCase):
    def test_complete_improvement_and_regression(self):
        report = complete_report()
        self.assertTrue(check_report(report, verify_sources=False)["passed"])
        for row in report["profiles"][MOTORS[0]]["validation_paired_seeds"]:
            row["variants"]["New core"]["position_rmse_deg"] = .101
        self.assertFalse(check_report(report, verify_sources=False)["passed"])

    def test_missing_or_duplicate_seed_cannot_pass(self):
        for duplicate in (False, True):
            report = complete_report()
            rows = report["profiles"][MOTORS[0]]["validation_paired_seeds"]
            rows.pop()
            if duplicate:
                rows.append(copy.deepcopy(rows[0]))
            self.assertFalse(check_report(report, verify_sources=False)["passed"])

    def test_failed_nonfinite_or_faulted_trial_cannot_pass(self):
        for key, value in (("completed", False), ("position_rmse_deg", float("nan")),
                           ("position_max_abs_error_deg", float("inf")),
                           ("fault_sample_count", 1), ("first_latched_fault", {"status": 4})):
            report = complete_report()
            row = report["profiles"][MOTORS[1]]["validation_paired_seeds"][0]
            row["variants"]["New core"][key] = value
            self.assertFalse(check_report(report, verify_sources=False)["passed"])

    def test_lower_rmse_does_not_hide_cost_or_saturation(self):
        for key, value in (("position_max_abs_error_deg", 1.), ("command_rms_nm", 1.),
                           ("command_at_limit_fraction", .01),
                           ("envelope_limited_interval_fraction", .01)):
            report = complete_report()
            row = report["profiles"][MOTORS[1]]["validation_paired_seeds"][0]
            row["variants"]["New core"][key] = value
            self.assertFalse(check_report(report, verify_sources=False)["passed"])

    def test_changed_experiment_or_missing_same_integral_control_cannot_pass(self):
        report = complete_report()
        report["primary_case_preselected"]["frequency_hz"] = 1.
        self.assertFalse(check_report(report, verify_sources=False)["passed"])
        report = complete_report()
        del report["profiles"][MOTORS[1]]["validation_paired_seeds"][0]["variants"]["Original core (same Ki)"]
        self.assertFalse(check_report(report, verify_sources=False)["passed"])

    def test_changed_actual_controller_configuration_cannot_pass(self):
        report = complete_report()
        row = report["profiles"][MOTORS[0]]["validation_paired_seeds"][0]
        row["variants"]["New core"]["actual_c_config"]["torque_limit_nm"] = 7.
        self.assertFalse(check_report(report, verify_sources=False)["passed"])


if __name__ == "__main__":
    unittest.main()
