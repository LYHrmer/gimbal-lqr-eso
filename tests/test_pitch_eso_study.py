"""Selection must not hide protocol changes, failed runs or missing seeds."""
from copy import deepcopy
import ctypes as ct
from dataclasses import asdict
import unittest

from experimental.pitch_eso.run_experiment import (METRICS, assess, candidate_id,
    candidates, case_inputs, load_plan, select_candidate)
from sim.c_core import Config
from sim.run_benchmarks import config_dict
from sim.run_pitch_profiles import load_profiles, physical_parameters


class PitchESOStudyTests(unittest.TestCase):
    def fixture(self):
        plan, profile = load_plan(), load_profiles()[1]
        candidate = {"eso_bandwidth_rad_s": 60., "eso_gain": .4}
        groups = []
        for parameters, scale in ((plan["baseline"], 1.), (candidate, .8)):
            rows = []
            for entry in plan["cases"]:
                case, clocks = case_inputs(entry)
                clock = asdict(clocks[0])
                plant, ga, gb, _ = physical_parameters(profile, case)
                for seed in plan["development_seeds"]:
                    metrics = {key: scale for key in METRICS}
                    metrics.update(completed=True, failure=None, first_latched_fault=None,
                        first_physical_limit_crossing=None, actual_last_sample_s=6.,
                        evaluation_start_s=2., evaluation_end_s=6.,
                        actual_c_config=config_dict(Config(**dict(profile["controller"], **parameters))),
                        actual_plant=dict(asdict(plant), gravity_cos_nm=ga, gravity_sin_nm=gb),
                        actual_c_gravity_model={key: ct.c_float(value*case.model_gravity_factor).value
                            for key, value in profile["gravity_model"].items()},
                        feedback_timing={"position": deepcopy(clock), "velocity": deepcopy(clock)})
                    rows.append({"case": entry["name"], "seed": seed, "metrics": metrics})
            groups.append(rows)
        return plan, profile, candidate, groups[0], groups[1]

    def evaluate(self, fixture):
        plan, profile, candidate, baseline, rows = fixture
        return assess(rows, baseline, plan, plan["development_seeds"], profile, candidate)

    def test_can_angle_rpm_always_share_a_1khz_packet_clock(self):
        plan = load_plan()
        self.assertEqual(len(candidates(plan)), 9)
        self.assertFalse(set(plan["development_seeds"]) & set(plan["holdout_seeds"]))
        for entry in plan["cases"]:
            _, clocks = case_inputs(entry)
            self.assertEqual(clocks[0], clocks[1])
            self.assertEqual(clocks[0].period_samples, 1)
        with self.assertRaises(ValueError):
            case_inputs({"name": "not_native_can", "pitch": {}, "can_clock": {"period_samples": 2}})

    def test_valid_grid_uses_all_cases_for_selection(self):
        fixture = self.fixture()
        plan, _, candidate, _, _ = fixture
        assessment = self.evaluate(fixture)
        self.assertTrue(assessment["passed"], assessment)
        key = candidate_id(candidate)
        entries = {key: {"assessment": assessment}}
        self.assertEqual(select_candidate(entries, plan), key)
        assessment["passed"] = False
        self.assertIsNone(select_candidate(entries, plan))

    def test_missing_duplicate_seed_and_bad_metrics_fail_closed(self):
        for mutation in ("missing", "duplicate", "nan", "bool", "fault", "short", "wrong_gain", "quantization", "split_clock"):
            fixture = self.fixture()
            rows = fixture[-1]
            metric = rows[0]["metrics"]
            if mutation == "missing":
                rows.pop()
            elif mutation == "duplicate":
                rows[1] = deepcopy(rows[0])
            elif mutation in ("nan", "bool"):
                metric[METRICS[0]] = float("nan") if mutation == "nan" else True
            elif mutation == "fault":
                metric["first_latched_fault"] = {"status": 6}
            elif mutation == "short":
                metric["evaluation_end_s"] = 3.
            elif mutation == "wrong_gain":
                metric["actual_c_config"]["k_integral"] = 0.
            elif mutation == "quantization":
                # Even changing BOTH comparators is not a valid native-CAN run.
                metric["actual_plant"]["velocity_quantum_rad_s"] = 0.
                fixture[-2][0]["metrics"]["actual_plant"]["velocity_quantum_rad_s"] = 0.
            else:
                metric["feedback_timing"]["velocity"]["delay_samples"] = 1
            with self.subTest(mutation=mutation):
                self.assertFalse(self.evaluate(fixture)["passed"])

    def test_primary_win_alone_does_not_hide_stress_regression(self):
        fixture = self.fixture()
        for row in fixture[-1]:
            if row["case"] == "can_transport_jitter":
                row["metrics"]["position_rmse_deg"] = 1.06
        self.assertFalse(self.evaluate(fixture)["passed"])

    def test_changing_both_comparators_does_not_bypass_the_fixed_model(self):
        for key in ("gravity", "inertia_kg_m2", "torque_lag_s", "command_delay_samples"):
            fixture = self.fixture()
            for rows in fixture[-2:]:
                for row in rows:
                    metric = row["metrics"]
                    if key == "gravity":
                        metric["actual_c_gravity_model"] = {"cos_nm": 0., "sin_nm": 0.}
                    else:
                        metric["actual_plant"][key] = 999
            with self.subTest(field=key):
                self.assertFalse(self.evaluate(fixture)["passed"])

    def test_published_baseline_cannot_count_as_an_improved_candidate(self):
        plan, profile, _, baseline, _ = self.fixture()
        result = assess(baseline, baseline, plan, plan["development_seeds"], profile, plan["baseline"])
        self.assertFalse(result["passed"])


if __name__ == "__main__":
    unittest.main()
