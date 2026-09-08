"""Contract tests for the independent synthetic motor torque/speed envelope."""
from dataclasses import FrozenInstanceError, replace
import math
import sys
import unittest

from sim.motor_envelope import (
    DM4310_24V_REFERENCE, MotorEnvelope, actuator_limit, torque_bounds,
)


class MotorEnvelopeTests(unittest.TestCase):
    def setUp(self):
        self.continuous = DM4310_24V_REFERENCE
        self.peak = replace(self.continuous, current_torque_limit_nm=7.0)
        self.no_load = self.continuous.no_load_velocity_rad_s

    def test_explicit_reference_and_no_generic_defaults(self):
        self.assertEqual(self.continuous.current_torque_limit_nm, 3.0)
        self.assertEqual(self.continuous.peak_torque_anchor_nm, 7.0)
        self.assertAlmostEqual(self.no_load, 20.943951023931955, places=12)
        self.assertAlmostEqual(self.continuous.rated_velocity_rad_s,
                               12.566370614359172, places=12)
        self.assertEqual(self.continuous.rated_torque_nm, 3.0)
        with self.assertRaises(TypeError):
            MotorEnvelope()
        with self.assertRaises(TypeError):
            actuator_limit(1.0, 0.0)
        with self.assertRaises(FrozenInstanceError):
            self.continuous.current_torque_limit_nm = 7.0

    def test_standstill_and_no_load_endpoints(self):
        for parameters, expected in ((self.continuous, 3.0), (self.peak, 7.0)):
            with self.subTest(cap=expected):
                self.assertEqual(torque_bounds(0.0, parameters), (-expected, expected))
                self.assertEqual(actuator_limit(100.0, 0.0, parameters), expected)
                self.assertEqual(actuator_limit(-100.0, 0.0, parameters), -expected)
                self.assertEqual(actuator_limit(100.0, self.no_load, parameters), 0.0)
                self.assertEqual(actuator_limit(-100.0, -self.no_load, parameters), 0.0)
                self.assertEqual(actuator_limit(-100.0, self.no_load, parameters), -expected)
                self.assertEqual(actuator_limit(100.0, -self.no_load, parameters), expected)

    def test_linear_motoring_curve_and_unchanged_small_requests(self):
        # Omitting both rated fields preserves the original generic straight line.
        linear = MotorEnvelope(7.0, 7.0, self.no_load)
        self.assertIsNone(linear.rated_velocity_rad_s)
        self.assertIsNone(linear.rated_torque_nm)
        for fraction, expected in ((0.25, 5.25), (0.5, 3.5), (0.75, 1.75)):
            with self.subTest(speed_fraction=fraction):
                self.assertAlmostEqual(actuator_limit(
                    100.0, self.no_load * fraction, linear), expected, places=12)
        self.assertEqual(actuator_limit(1.0, 0.5 * self.no_load, self.peak), 1.0)
        self.assertEqual(actuator_limit(-1.0, 0.5 * self.no_load, self.peak), -1.0)
        self.assertEqual(actuator_limit(0.0, 0.5 * self.no_load, self.peak), 0.0)

    def test_continuous_cap_is_combined_with_speed_envelope(self):
        self.assertEqual(actuator_limit(100.0, 0.5 * self.no_load, self.continuous), 3.0)
        self.assertAlmostEqual(actuator_limit(
            100.0, 0.75 * self.no_load, self.continuous), 1.875, places=12)
        for velocity in (-100.0, -self.no_load, -10.0, 0.0, 10.0, self.no_load, 100.0):
            for request in (-100.0, -3.0, -0.1, 0.0, 0.1, 3.0, 100.0):
                with self.subTest(velocity=velocity, request=request):
                    actual = actuator_limit(request, velocity, self.continuous)
                    self.assertLessEqual(abs(actual), 3.0)
                    self.assertLessEqual(abs(actual), abs(request))
                    self.assertGreaterEqual(actual * request, 0.0)

    def test_braking_uses_current_cap_and_overspeed_blocks_acceleration(self):
        for parameters, cap in ((self.continuous, 3.0), (self.peak, 7.0)):
            for fraction in (0.75, 1.0, 1.5, 10.0):
                with self.subTest(cap=cap, speed_fraction=fraction):
                    speed = fraction * self.no_load
                    self.assertEqual(actuator_limit(-100.0, speed, parameters), -cap)
                    self.assertEqual(actuator_limit(100.0, -speed, parameters), cap)
                    if fraction >= 1.0:
                        self.assertEqual(actuator_limit(100.0, speed, parameters), 0.0)
                        self.assertEqual(actuator_limit(-100.0, -speed, parameters), 0.0)

    def test_direction_reversal_and_sign_symmetry(self):
        speed = 0.75 * self.no_load
        # A negative request first brakes positive motion, then motors in reverse.
        self.assertEqual(actuator_limit(-100.0, speed, self.peak), -7.0)
        self.assertEqual(actuator_limit(-100.0, 0.0, self.peak), -7.0)
        self.assertAlmostEqual(actuator_limit(-100.0, -speed, self.peak), -1.875, places=12)
        for parameters in (self.continuous, self.peak):
            for velocity in (0.0, 0.01, 1.0, 10.0, self.no_load, 100.0):
                lower, upper = torque_bounds(velocity, parameters)
                reverse_lower, reverse_upper = torque_bounds(-velocity, parameters)
                self.assertEqual(lower, -reverse_upper)
                self.assertEqual(upper, -reverse_lower)
                for request in (-100.0, -0.1, 0.0, 0.1, 100.0):
                    self.assertEqual(actuator_limit(request, velocity, parameters),
                                     -actuator_limit(-request, -velocity, parameters))

    def test_invalid_requests_and_velocities_are_rejected(self):
        for value in (math.nan, math.inf, -math.inf, None, "1", True, complex(1.0, 0.0)):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    actuator_limit(value, 0.0, self.peak)
                with self.assertRaises(ValueError):
                    actuator_limit(0.0, value, self.peak)
                with self.assertRaises(ValueError):
                    torque_bounds(value, self.peak)
        with self.assertRaises(ValueError):
            actuator_limit(1.0, 0.0, None)

    def test_invalid_specifications_are_rejected(self):
        for field in ("current_torque_limit_nm", "peak_torque_anchor_nm",
                      "no_load_velocity_rad_s"):
            for value in (0.0, -1.0, math.nan, math.inf, -math.inf, None, "3", True):
                with self.subTest(field=field, value=value):
                    with self.assertRaises(ValueError):
                        replace(self.continuous, **{field: value})
        with self.assertRaises(ValueError):
            replace(self.continuous, current_torque_limit_nm=7.01)

    def test_extreme_finite_inputs_do_not_overflow(self):
        self.assertEqual(actuator_limit(sys.float_info.max, sys.float_info.max,
                                        self.peak), 0.0)
        self.assertEqual(actuator_limit(-sys.float_info.max, sys.float_info.max,
                                        self.peak), -7.0)
        tiny_no_load = MotorEnvelope(3.0, 7.0, sys.float_info.min)
        self.assertEqual(actuator_limit(7.0, sys.float_info.max, tiny_no_load), 0.0)
        self.assertEqual(actuator_limit(-7.0, sys.float_info.max, tiny_no_load), -3.0)

    def test_rated_point_and_piecewise_midpoints(self):
        rated_speed = self.peak.rated_velocity_rad_s
        for parameters in (self.continuous, self.peak):
            self.assertEqual(actuator_limit(100.0, rated_speed, parameters), 3.0)
            self.assertEqual(actuator_limit(-100.0, -rated_speed, parameters), -3.0)
        # Midpoint of each straight segment equals the average of its endpoints.
        self.assertAlmostEqual(actuator_limit(
            100.0, rated_speed / 2.0, self.peak), 5.0, places=12)
        self.assertAlmostEqual(actuator_limit(
            100.0, (rated_speed + self.no_load) / 2.0, self.peak), 1.5, places=12)
        disparate_scales = MotorEnvelope(1.0, 1e30, 10.0, 5.0, 1.0)
        self.assertEqual(actuator_limit(1.0, 5.0, disparate_scales), 1.0)

    def test_piecewise_join_is_continuous_and_monotone(self):
        rated_speed = self.peak.rated_velocity_rad_s
        below = actuator_limit(100.0, math.nextafter(rated_speed, 0.0), self.peak)
        at = actuator_limit(100.0, rated_speed, self.peak)
        above = actuator_limit(100.0, math.nextafter(rated_speed, math.inf), self.peak)
        self.assertAlmostEqual(below, at, places=12)
        self.assertAlmostEqual(above, at, places=12)
        self.assertGreaterEqual(below, at)
        self.assertGreaterEqual(at, above)
        previous = self.peak.peak_torque_anchor_nm
        for step in range(1001):
            current = actuator_limit(100.0, step * self.no_load / 1000.0, self.peak)
            self.assertLessEqual(current, previous)
            self.assertGreaterEqual(current, 0.0)
            previous = current
        self.assertEqual(previous, 0.0)

    def test_rated_point_requires_valid_ordered_pair(self):
        for field in ("rated_velocity_rad_s", "rated_torque_nm"):
            for value in (None, 0.0, -1.0, math.nan, math.inf, -math.inf, "3", True):
                with self.subTest(field=field, value=value):
                    with self.assertRaises(ValueError):
                        replace(self.continuous, **{field: value})
        for velocity in (self.no_load, 2.0 * self.no_load):
            with self.assertRaises(ValueError):
                replace(self.continuous, rated_velocity_rad_s=velocity)
        with self.assertRaises(ValueError):
            replace(self.continuous, rated_torque_nm=7.01)
        # Equal rated/anchor torques are valid: the first segment is flat.
        flat = replace(self.peak, rated_torque_nm=7.0)
        self.assertEqual(actuator_limit(100.0, flat.rated_velocity_rad_s / 2.0, flat), 7.0)


if __name__ == "__main__":
    unittest.main()
