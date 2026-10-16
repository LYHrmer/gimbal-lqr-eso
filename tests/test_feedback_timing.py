"""Source-clock feedback sampling through the actual C controller."""
import unittest

import numpy as np

from sim.c_core import Core
from sim.run_benchmarks import COLUMNS, DT, IX, Plant, make_config, config_dict
from sim.run_sensitivity import Scenario, run_trial


class FeedbackTimingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.core = Core()
        cls.gain = {"k_position": 10., "k_velocity": 1.}

    def trial(self, period, delay, plant=None):
        scenario = Scenario("clock_test", "test", "synchronous sample and hold",
                            plant or Plant(), speed_envelope=False)
        return run_trial(self.core, scenario, self.gain, seed=23, duration=.012,
                         controller_config=config_dict(make_config(self.gain, 0.)),
                         feedback_period_samples=period, feedback_delay_samples=delay)

    def test_hold_and_delay_preserve_samples_noise_and_source_age(self):
        plant = Plant(position_noise_std_rad=.001, velocity_noise_std_rad_s=.01,
                      position_quantum_rad=.0001, velocity_quantum_rad_s=.001)
        data, metrics = self.trial(2, 1, plant)
        noise = np.random.default_rng(23).normal(size=(7, 2))
        sources = [0, 0, 0, 2, 2, 4, 4, 6, 6, 8, 8, 10, 10]
        for index, source in enumerate(sources):
            measured = data[source, [IX["position_rad"], IX["velocity_rad_s"]]] + \
                noise[source//2] * [.001, .01]
            expected = np.round(measured/[.0001, .001])*[.0001, .001]
            np.testing.assert_array_equal(data[index, [IX["measured_position_rad"],
                                                       IX["measured_velocity_rad_s"]]], expected)
            self.assertAlmostEqual(data[index, len(COLUMNS)+4], (index-source)*DT)
        self.assertEqual(metrics["feedback_timing"]["max_age_s"], .002)
        self.assertIsNone(metrics["first_latched_fault"])

    def test_overdue_held_feedback_reaches_real_timeout_guard(self):
        data, metrics = self.trial(2, 6)
        self.assertEqual(metrics["first_latched_fault"], {"time_s": .006, "status": 6})
        np.testing.assert_array_equal(data[6:, IX["status"]], 6)
        np.testing.assert_array_equal(data[6:, IX["command_nm"]], 0.)
        self.assertTrue(metrics["completed"])  # The failed control run is retained.

    def test_default_reads_are_fresh_and_invalid_clocks_are_rejected(self):
        data, metrics = self.trial(1, 0)
        np.testing.assert_array_equal(data[:, len(COLUMNS)+4], 0.)
        self.assertIsNone(metrics["first_latched_fault"])
        for period, delay in ((0, 0), (1, -1), (1.5, 0), (1, .5), (True, 0)):
            with self.subTest(period=period, delay=delay):
                with self.assertRaises(ValueError):
                    self.trial(period, delay)


if __name__ == "__main__":
    unittest.main()
