"""The experimental screening verdict must not accept corrupt or incomplete data."""
from copy import deepcopy
import unittest

from experimental.feedback_timing.run_candidate import assess


class CandidateGateTests(unittest.TestCase):
    def fixture(self):
        clocks = ("sync_1_0", "sync_2_0", "angle500_gyro1000")
        metric = {"completed": True, "first_latched_fault": None,
                  "actual_c_config": {"k_integral": 1.},
                  "position_rmse_deg": 1., "position_max_abs_error_deg": 2.}
        entry = {"seeds": [1, 2, 3], "cases": [
            {"clock": clock, "seed": seed, "expected_stale_fault": False,
             "variants": {"observe_only": deepcopy(metric), "compensated": deepcopy(metric)}}
            for clock in clocks for seed in (1, 2, 3)]}
        baseline = {"clocks": dict.fromkeys(clocks), "profiles": {"gm6020_pitch": entry}}
        candidate = deepcopy(baseline)
        for row in candidate["profiles"]["gm6020_pitch"]["cases"]:
            if row["clock"] != "sync_1_0":
                row["variants"]["compensated"].update(position_rmse_deg=.8, position_max_abs_error_deg=1.8)
        return baseline, candidate

    def test_measured_improvement_and_regressions(self):
        baseline, candidate = self.fixture()
        self.assertTrue(assess(baseline, candidate)["passed"])
        candidate["profiles"]["gm6020_pitch"]["cases"][3]["variants"]["compensated"]["position_rmse_deg"] = 2.
        self.assertFalse(assess(baseline, candidate)["passed"])

    def test_missing_case_or_changed_gain_cannot_pass(self):
        baseline, candidate = self.fixture()
        candidate["profiles"]["gm6020_pitch"]["cases"].pop()
        self.assertFalse(assess(baseline, candidate)["passed"])
        baseline, candidate = self.fixture()
        candidate["profiles"]["gm6020_pitch"]["cases"][0]["variants"]["compensated"]["actual_c_config"]["k_integral"] = 0.
        self.assertFalse(assess(baseline, candidate)["passed"])

    def test_nonfinite_negative_and_boolean_metrics_cannot_pass(self):
        for value in (float("nan"), float("inf"), -1., True):
            with self.subTest(value=value):
                baseline, candidate = self.fixture()
                candidate["profiles"]["gm6020_pitch"]["cases"][0]["variants"]["compensated"]["position_rmse_deg"] = value
                self.assertFalse(assess(baseline, candidate)["passed"])

    def test_partial_baseline_cannot_be_compared_as_complete_tracking(self):
        baseline, candidate = self.fixture()
        metric = baseline["profiles"]["gm6020_pitch"]["cases"][0]["variants"]["compensated"]
        metric.update(completed=False, first_latched_fault={"status": 6})
        self.assertFalse(assess(baseline, candidate)["passed"])

    def test_matching_timeout_does_not_hide_new_post_fault_failure(self):
        baseline, candidate = self.fixture()
        for report in (baseline, candidate):
            for row in report["profiles"]["gm6020_pitch"]["cases"][:3]:
                row["expected_stale_fault"] = True
                for metric in row["variants"].values():
                    metric["first_latched_fault"] = {"time_s": .006, "status": 6}
        self.assertTrue(assess(baseline, candidate)["passed"])
        metric = candidate["profiles"]["gm6020_pitch"]["cases"][0]["variants"]["compensated"]
        metric.update(completed=False, failure="nonfinite plant")
        self.assertFalse(assess(baseline, candidate)["passed"])


if __name__ == "__main__":
    unittest.main()
