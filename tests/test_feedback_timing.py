"""Source-clock feedback sampling through the actual C controller."""
import unittest

import numpy as np

from sim.c_core import Core
from sim.feedback_timing import ChannelTiming, FeedbackSampler
from sim.pitch_core import PitchCore
from sim.run_pitch_profiles import LABELS, PIX, PitchCase, load_profiles, run_trial as pitch_trial
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


class IndependentClockTests(unittest.TestCase):
    def sampler(self, position, velocity):
        return FeedbackSampler(position, velocity, 23, (0., 0.), (0., 0.))

    def test_asynchronous_acquisitions_keep_distinct_source_times(self):
        sampler = self.sampler(ChannelTiming(2, phase_samples=1), ChannelTiming(1, 1))
        for tick, expected in enumerate(((0, 0), (1, 0), (1, 1), (3, 2), (3, 3))):
            values, sources = sampler.read(tick, 10.+tick, 20.+tick)
            self.assertEqual(sources, expected)
            self.assertEqual(values, (10.+expected[0], 20.+expected[1]))

    def test_jitter_does_not_let_an_old_packet_replace_a_newer_one(self):
        sampler = self.sampler(ChannelTiming(1, delay_jitter_samples=(3, 0)), ChannelTiming())
        sources = [sampler.read(t, float(t), 0.)[1][0] for t in range(6)]
        self.assertEqual(sources, [0, 0, 2, 2, 4, 4])
        self.assertEqual(sampler.discarded_old[0], 1)

    def test_drops_keep_last_value_and_age_without_resetting_clock(self):
        sampler = self.sampler(ChannelTiming(2, drop_every=2), ChannelTiming())
        sources = [sampler.read(t, float(t), 0.)[1][0] for t in range(9)]
        self.assertEqual(sources, [0, 0, 2, 2, 2, 2, 6, 6, 6])
        self.assertEqual(sampler.metrics(DT)["max_position_age_s"], .003)
        self.assertEqual(sampler.dropped, [2, 0])
        with self.assertRaises(ValueError):
            sampler.read(8, 0., 0.)
        with self.assertRaises(ValueError):
            sampler.read(10, 0., 0.)

    def test_pitch_uses_oldest_channel_age_in_actual_c_guard(self):
        profile = load_profiles()[1]
        data, metrics = pitch_trial(profile, PitchCase(), LABELS[2], 5300, None,
            PitchCore(), duration=.012,
            feedback_clocks=(ChannelTiming(1), ChannelTiming(2, 6)))
        self.assertEqual(metrics["first_latched_fault"], {"time_s": .006, "status": 6})
        np.testing.assert_array_equal(data[6:, PIX["command_nm"]], 0.)
        self.assertEqual(data[6, PIX["position_source_time_s"]], .006)
        self.assertEqual(data[6, PIX["velocity_source_time_s"]], 0.)

    def test_pitch_holds_noisy_quantized_position_at_its_source_timestamp(self):
        from dataclasses import replace
        case = replace(PitchCase(), base_tilt_deg=15.)
        data, metrics = pitch_trial(load_profiles()[1], case, LABELS[2], 5300, None,
            PitchCore(), duration=.012,
            feedback_clocks=(ChannelTiming(2, 1), ChannelTiming()))
        for tick in (2, 4, 6, 8, 10, 12):
            self.assertEqual(data[tick, PIX["measured_position_rad"]],
                             data[tick-1, PIX["measured_position_rad"]])
            self.assertAlmostEqual(data[tick, PIX["feedback_age_s"]], .002)
        self.assertIsNone(metrics["first_latched_fault"])


if __name__ == "__main__":
    unittest.main()
