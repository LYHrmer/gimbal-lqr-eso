"""Protect historical evidence against omissions and misleading aggregates."""
import copy
import json
import statistics
import unittest

from tools.check_legacy_gm6020_evidence import EVIDENCE, check_case, check_evidence, check_report


class LegacyGm6020EvidenceTests(unittest.TestCase):
    def setUp(self):
        self.report = json.loads((EVIDENCE / "step_diagnostics.json").read_text())

    def test_archived_record_is_consistent(self):
        checked = check_evidence()
        self.assertEqual(checked["checked_cases"], 12)
        self.assertEqual(checked["checked_trials"], 36)

    def test_missing_duplicate_or_changed_conditions_rejected(self):
        missing = copy.deepcopy(self.report)
        missing["rows"].pop()
        duplicate = copy.deepcopy(self.report)
        duplicate["rows"][-1] = duplicate["rows"][0]
        changed = copy.deepcopy(self.report)
        changed["rows"][0]["delay_samples"] = 5
        for report in (missing, duplicate, changed):
            with self.assertRaises(ValueError):
                check_report(report)

    def test_missing_duplicate_and_boolean_seeds_rejected(self):
        for seeds in ((1, 2), (1, 2, 2), (True, 2, 3)):
            row = copy.deepcopy(self.report["rows"][0])
            row["trials"] = row["trials"][:len(seeds)]
            for trial, seed in zip(row["trials"], seeds):
                trial["seed"] = seed
            with self.subTest(seeds=seeds), self.assertRaises(ValueError):
                check_case(row)

    def test_nonfinite_nonnumeric_or_negative_metrics_rejected(self):
        for target in ("summary", "trial"):
            for value in (float("nan"), float("inf"), float("-inf"), True, "0", -1):
                row = copy.deepcopy(self.report["rows"][0])
                metrics = row["metrics"] if target == "summary" else row["trials"][0]["metrics"]
                metrics["rmse_deg"] = value
                with self.subTest(target=target, value=value), self.assertRaises(ValueError):
                    check_case(row)

    def test_altered_mean_is_detected(self):
        row = self.report["rows"][0]
        row["metrics"]["rmse_deg"] += 0.01
        with self.assertRaises(ValueError):
            check_case(row)

    def test_last_seed_torque_semantics_are_preserved(self):
        row = self.report["rows"][0]
        for trial, value in zip(row["trials"], (0.1, 0.2, 0.3)):
            trial["metrics"]["comp_rms_nm"] = value
        row["metrics"]["comp_rms_nm"] = 0.3
        check_case(row)
        row["metrics"]["comp_rms_nm"] = 0.2
        with self.assertRaises(ValueError):
            check_case(row)

    def test_one_censored_trial_cannot_be_hidden_by_averaging(self):
        row = self.report["rows"][0]
        row["trials"][0]["metrics"]["settle_ms"] = 1800.0
        row["trials"][0]["settle_censored"] = True
        row["metrics"]["settle_ms"] = statistics.mean(t["metrics"]["settle_ms"] for t in row["trials"])
        self.assertLess(row["metrics"]["settle_ms"], 1800.0)
        row["settling"] = {"censored_seed_count": 1, "mean_ms_if_all_settled": None}
        check_case(row)
        row["settling"]["mean_ms_if_all_settled"] = row["metrics"]["settle_ms"]
        with self.assertRaises(ValueError):
            check_case(row)
        row["settling"]["mean_ms_if_all_settled"] = None
        row["settling"]["censored_seed_count"] = 0
        with self.assertRaises(ValueError):
            check_case(row)

    def test_false_per_trial_settling_flag_rejected(self):
        row = self.report["rows"][1]
        row["trials"][0]["settle_censored"] = False
        with self.assertRaises(ValueError):
            check_case(row)

    def test_changed_protocol_source_or_scope_rejected(self):
        for field, value in (("dt_s", 0.002), ("seeds", [1]),
                             ("rmse_window_s", [0.0, 2.0]), ("integral_enabled", False)):
            report = copy.deepcopy(self.report)
            report["protocol"][field] = value
            with self.subTest(field=field), self.assertRaises(ValueError):
                check_report(report)
        for field, value in (("script_sha256", "0" * 64), ("scope", "Hardware validated")):
            report = copy.deepcopy(self.report)
            report[field] = value
            with self.subTest(field=field), self.assertRaises(ValueError):
                check_report(report)

    def test_all_archived_axis_parameters_are_fixed(self):
        for axis, config in self.report["protocol"]["axes"].items():
            for key, value in config.items():
                report = copy.deepcopy(self.report)
                if type(value) is bool:
                    changed = not value
                elif type(value) in (int, float):
                    changed = value + 1
                else:
                    changed = "changed"
                report["protocol"]["axes"][axis][key] = changed
                with self.subTest(axis=axis, key=key), self.assertRaises(ValueError):
                    check_report(report)


if __name__ == "__main__":
    unittest.main()
