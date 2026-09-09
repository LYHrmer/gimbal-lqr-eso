"""Pitch experiment physics, coordinate and fail-closed comparison checks."""
from copy import deepcopy
from dataclasses import asdict, replace
import math
import unittest

import numpy as np

from sim.c_core import Config
from sim.pitch_core import PitchCore
from sim.run_benchmarks import config_dict
from sim.run_pitch_profiles import (HOLDOUT_SEEDS, LABELS, PIX,
    PitchCase, assess_group, gravity_load, load_profiles, pitch_reference, run_trial)


class PitchPhysicsTests(unittest.TestCase):
    def test_gravity_matches_potential_energy_gradient(self):
        # d/dtheta [A*sin(theta)-B*cos(theta)] is the resisting torque.
        for theta in (-.4, 0., .35, .7):
            h, a, b = 1e-6, .45, .10
            potential = lambda q: a*math.sin(q)-b*math.cos(q)
            numerical = (potential(theta+h)-potential(theta-h))/(2*h)
            self.assertAlmostEqual(gravity_load(theta, a, b), numerical, delta=1e-10)
        self.assertGreater(gravity_load(0., .45, .10), 0.)

    def test_tilt_changes_world_reference_not_joint_trajectory(self):
        normal, tilted = PitchCase(), replace(PitchCase(), base_tilt_deg=15.)
        for t in (0., .4, 1.2, 2., 3.2):
            p, v, a = pitch_reference(t, normal)
            tp, tv, ta = pitch_reference(t, tilted)
            self.assertAlmostEqual(tp-p, math.radians(15.), places=14)
            self.assertEqual((v, a), (tv, ta))


class PitchActualCTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.core, cls.profiles = PitchCore(), load_profiles()

    def test_short_seeded_gravity_simulation_reproducible_total_torque_bounded(self):
        profile = self.profiles[0]
        first, metrics = run_trial(profile, PitchCase(), LABELS[2], 5300, None, self.core, duration=.15)
        repeated, repeated_metrics = run_trial(profile, PitchCase(), LABELS[2], 5300, None, self.core, duration=.15)
        np.testing.assert_array_equal(first, repeated)
        self.assertEqual(metrics, repeated_metrics)
        self.assertTrue(metrics["completed"])
        self.assertEqual(metrics["fault_sample_count"], 0)
        self.assertTrue(np.all(np.isfinite(first)))
        command = first[:, PIX["command_nm"]]
        self.assertEqual(command[0], 0.)
        self.assertGreater(float(np.max(first[:, PIX["gravity_feedforward_nm"]])), .4)
        self.assertLessEqual(float(np.max(np.abs(command))), profile["controller"]["torque_limit_nm"]+1e-6)
        self.assertLessEqual(float(np.max(np.abs(np.diff(command)))), .1+1e-6)

    def test_impossible_pitch_load_is_reported_as_boundary_failure(self):
        # More gravity than the GM current-limited actuator can hold. Do not
        # clip theta at its stop and report an apparently completed trial.
        case = replace(PitchCase(), gravity_factor=4., expected_boundary=True)
        trace, metrics = run_trial(self.profiles[1], case, LABELS[2], 5300, None, self.core, duration=2.)
        self.assertFalse(metrics["completed"])
        self.assertIsNotNone(metrics["first_physical_limit_crossing"])
        self.assertLess(metrics["first_physical_limit_crossing"]["time_s"], 2.)
        self.assertLess(len(trace), 2001)


class PitchGateTests(unittest.TestCase):
    def fixture(self):
        profile = load_profiles()[0]
        variants = {}
        for index, label in enumerate(LABELS):
            variants[label] = {"completed": True, "first_latched_fault": None,
                "first_physical_limit_crossing": None, "actual_c_config": config_dict(Config(**profile["controller"])),
                "position_rmse_deg": 1. if index < 2 else .8,
                "position_max_abs_error_deg": 2. if index < 2 else 1.8,
                "command_rms_nm": 1., "startup_peak_error_deg": .2,
                "command_at_limit_fraction": 0.}
        trials = [{"seed": seed, "case": asdict(PitchCase()), "variants": deepcopy(variants),
            "change_vs_upstream_percent": -20., "change_vs_gravity_off_percent": -20.}
            for seed in HOLDOUT_SEEDS]
        return profile, trials

    def test_gate_requires_all_planned_seeds_and_matching_gains(self):
        profile, trials = self.fixture()
        self.assertTrue(assess_group(profile, trials, HOLDOUT_SEEDS)["passed"])
        self.assertFalse(assess_group(profile, trials[:-1], HOLDOUT_SEEDS)["passed"])
        trials[0]["variants"][LABELS[0]]["actual_c_config"]["k_integral"] = 123.
        self.assertFalse(assess_group(profile, trials, HOLDOUT_SEEDS)["passed"])

    def test_gate_rejects_faults_and_invalid_metrics(self):
        profile, trials = self.fixture()
        trials[0]["variants"][LABELS[2]]["first_latched_fault"] = {"status": 9}
        self.assertFalse(assess_group(profile, trials, HOLDOUT_SEEDS)["passed"])
        profile, trials = self.fixture()
        trials[0]["variants"][LABELS[2]]["position_rmse_deg"] = float("nan")
        self.assertFalse(assess_group(profile, trials, HOLDOUT_SEEDS)["passed"])

    def test_gate_recomputes_claimed_percentages_from_actual_errors(self):
        profile, trials = self.fixture()
        for row in trials:
            row["variants"][LABELS[2]]["position_rmse_deg"] = 1.1
            # Stored "-20%" must not disguise worse actual errors.
        self.assertFalse(assess_group(profile, trials, HOLDOUT_SEEDS)["passed"])


if __name__ == "__main__":
    unittest.main()
