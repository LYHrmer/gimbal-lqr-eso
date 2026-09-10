"""Regression tests of independent RLS evidence checks; no motor claims."""
import copy
import csv
from pathlib import Path
import tempfile
import unittest

from tools.check_online_rls_results import (EXPECTED_PROTOCOL, SAMPLES, check_protocol,
                                            check_trials, read_trace, vector)


def complete_trials():
    return [{"seed": seed, "final_normalized_parameters": [1.01, 0.98],
             "batch_normalized_parameters": [1.011, 0.979],
             "accepted_updates": 2901, "rejected_updates": 99,
             "rejected_preserved_exactly": True} for seed in range(10)]


class OnlineRlsEvidenceTests(unittest.TestCase):
    def test_protocol_is_frozen(self):
        check_protocol(copy.deepcopy(EXPECTED_PROTOCOL))
        for key, replacement in (("source_dt_s", 0.002), ("seeds", [0]),
                                  ("nominal_update_s", 0.01),
                                  ("diagnostic_only_cases", [])):
            protocol = copy.deepcopy(EXPECTED_PROTOCOL)
            protocol[key] = replacement
            with self.subTest(key=key), self.assertRaises(ValueError):
                check_protocol(protocol)
        protocol = copy.deepcopy(EXPECTED_PROTOCOL)
        protocol["acceptance"]["baseline_each_parameter_error_max"] = 0.5
        with self.assertRaises(ValueError):
            check_protocol(protocol)

    def test_summary_is_recomputed_from_per_seed_estimates(self):
        summary, difference = check_trials(complete_trials(), batch=True)
        self.assertAlmostEqual(summary["mean_relative_error"][0], 0.01)
        self.assertAlmostEqual(summary["max_absolute_relative_error"][1], 0.02)
        self.assertAlmostEqual(difference, 0.001)

    def test_missing_duplicate_reordered_and_boolean_seeds_rejected(self):
        missing = complete_trials()[:-1]
        duplicate = complete_trials()
        duplicate[1]["seed"] = 0
        boolean = complete_trials()
        boolean[1]["seed"] = True
        for trials in (missing, duplicate, boolean, list(reversed(complete_trials()))):
            with self.assertRaises(ValueError):
                check_trials(trials, batch=True)

    def test_nonfinite_parameters_and_batch_labels_rejected(self):
        for field in ("final_normalized_parameters", "batch_normalized_parameters"):
            for value in (float("nan"), float("inf"), True):
                trials = complete_trials()
                trials[3][field][0] = value
                with self.subTest(field=field, value=value), self.assertRaises(ValueError):
                    check_trials(trials, batch=True)

    def test_bad_update_counts_and_rejected_state_change_fail(self):
        for key, value in (("accepted_updates", 0), ("rejected_updates", 0),
                           ("rejected_preserved_exactly", False),
                           ("rejected_preserved_exactly", 1)):
            trials = complete_trials()
            trials[4][key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                check_trials(trials, batch=True)

    def test_vector_requires_two_numeric_finite_values(self):
        for values in ([1], [1, 2, 3], ["1", 2], [1, None]):
            with self.assertRaises(ValueError):
                vector(values)

    def test_trace_requires_complete_timestamps_and_valid_excitation(self):
        header = ["time_s", "J_kg_m2", "b_Nms_rad", "status", "excitation_ok",
                  "gram_min_eigenvalue"]
        rows = [[(i+1)*0.02, 0.025, 0.06, 0, 1, 0.003] for i in range(SAMPLES)]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "trace.csv"

            def write(values):
                with path.open("w", newline="") as stream:
                    writer = csv.writer(stream)
                    writer.writerow(header)
                    writer.writerows(values)

            write(rows)
            self.assertEqual(len(read_trace(path)), SAMPLES)
            write(rows[:-1])
            with self.assertRaises(ValueError):
                read_trace(path)
            for column, invalid in ((0, 1.0), (1, float("nan")), (3, 11), (4, 0)):
                bad = copy.deepcopy(rows)
                bad[100][column] = invalid
                write(bad)
                with self.subTest(column=column), self.assertRaises(ValueError):
                    read_trace(path)


if __name__ == "__main__":
    unittest.main()
