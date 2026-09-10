"""Fast numerical and real-C integration checks; build the shared library first.

Run: python3 -s -m unittest discover -s tests -p 'test_*.py'
"""
import ctypes as ct
import math
import unittest
from unittest.mock import patch

import numpy as np

from sim import run_benchmarks as benchmark
from sim.c_core import Config, Controller, Core, Feedback, Output, Reference
from tools.tune_lqr import design_lqr


class LqrDesignTests(unittest.TestCase):
    def test_discrete_model_and_closed_loop_poles(self):
        result = design_lqr()
        parameters = result["parameters"]
        inertia, damping, dt = (parameters[name] for name in ("inertia", "damping", "dt"))
        decay = math.exp(-damping * dt / inertia)
        # Independently integrate the scalar damped-velocity equation under held torque.
        analytic_a = np.array([[1., inertia / damping * (1. - decay)], [0., decay]])
        analytic_b = np.array([[dt / damping - inertia / damping**2 * (1. - decay)],
                               [(1. - decay) / damping]])
        np.testing.assert_allclose(result["ad"], analytic_a, atol=1e-14, rtol=1e-12)
        np.testing.assert_allclose(result["bd"], analytic_b, atol=1e-14, rtol=1e-12)
        gain = np.array([[result["gain"]["k_position"], result["gain"]["k_velocity"]]])
        poles = np.linalg.eigvals(analytic_a - analytic_b @ gain)
        recorded = np.array([complex(pole["real"], pole["imag"])
                             for pole in result["closed_loop_poles"]])
        np.testing.assert_allclose(np.sort_complex(poles), np.sort_complex(recorded), atol=1e-12)
        self.assertLess(float(np.max(np.abs(poles))), 1.)
        self.assertAlmostEqual(result["spectral_radius"], float(np.max(np.abs(poles))), places=12)
        self.assertLess(result["dare_residual_max_abs"], 1e-6)

    def test_zero_damping_matches_held_torque_double_integrator(self):
        inertia, dt = .025, .002
        result = design_lqr(inertia=inertia, damping=0., dt=dt)
        analytic_a = np.array([[1., dt], [0., 1.]])
        analytic_b = np.array([[dt * dt / (2. * inertia)], [dt / inertia]])
        np.testing.assert_allclose(result["ad"], analytic_a, atol=1e-14, rtol=1e-12)
        np.testing.assert_allclose(result["bd"], analytic_b, atol=1e-14, rtol=1e-12)
        gain = np.array([[result["gain"]["k_position"], result["gain"]["k_velocity"]]])
        self.assertLess(float(np.max(np.abs(np.linalg.eigvals(analytic_a - analytic_b @ gain)))), 1.)
        self.assertLess(result["dare_residual_max_abs"], 1e-6)
        near_zero = design_lqr(inertia=inertia, damping=1e-9, dt=dt)
        np.testing.assert_allclose(list(result["gain"].values()),
                                   list(near_zero["gain"].values()), rtol=1e-8, atol=1e-8)

    def test_invalid_parameters_are_rejected(self):
        for name in ("inertia", "damping", "dt", "q_position", "q_velocity", "r_torque"):
            invalid_values = [-1., float("nan"), float("inf")]
            if name != "damping":
                invalid_values.append(0.)
            message = "finite and nonnegative" if name == "damping" else "finite and positive"
            for value in invalid_values:
                with self.subTest(parameter=name, value=value):
                    with self.assertRaisesRegex(ValueError, message):
                        design_lqr(**{name: value})


class ReferenceTests(unittest.TestCase):
    def test_reference_derivatives_include_startup_envelope(self):
        h = 1e-6
        for frequency, amplitude_deg in ((1., 5.), (3., 20.), (5., 20.)):
            amplitude = math.radians(amplitude_deg)
            self.assertEqual(benchmark.reference_at(0., frequency, amplitude), (0., 0., 0.))
            for time_s in (.13, .47, 1.3, 1.999, 2., 2.25):
                with self.subTest(frequency=frequency, time=time_s):
                    p, v, a = benchmark.reference_at(time_s, frequency, amplitude)
                    p_before, v_before, _ = benchmark.reference_at(time_s - h, frequency, amplitude)
                    p_after, v_after, _ = benchmark.reference_at(time_s + h, frequency, amplitude)
                    self.assertAlmostEqual((p_after - p_before) / (2*h), v, delta=1e-5)
                    self.assertAlmostEqual((v_after - v_before) / (2*h), a, delta=1e-5)
                    self.assertTrue(all(math.isfinite(x) for x in (p, v, a)))


class CSimulationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Core verifies sizeof/offsetof using C before loading the real shared library.
        cls.core = Core()
        cls.gain = design_lqr()["gain"]
        cls.core.lib.yaw_controller_set_torque_limit.argtypes = [ct.POINTER(Controller), ct.c_float]
        cls.core.lib.yaw_controller_set_torque_limit.restype = ct.c_bool
        cls.core.lib.yaw_controller_reset.argtypes = [ct.POINTER(Controller)]
        cls.core.lib.yaw_controller_reset.restype = None

    def test_seeded_short_simulation_is_reproducible_and_bounded(self):
        plant = benchmark.Plant(
            inertia_kg_m2=.039*1.25, damping_nm_s_rad=.30*.8,
            external_torque_nm=.35, coulomb_nm=.12, torque_lag_s=.002,
            command_delay_samples=1, position_quantum_rad=25./65535.,
            velocity_quantum_rad_s=60./4095., position_noise_std_rad=.00007,
            velocity_noise_std_rad_s=.01)
        # Shorten only this test fixture; the published benchmark remains 8 s / 2 s ramp.
        with patch.multiple(benchmark, DURATION=.3, RAMP=.1):
            first, metrics = benchmark.run_case(self.core, plant, 1., 5., self.gain, .8, 4310)
            repeated, repeated_metrics = benchmark.run_case(self.core, plant, 1., 5., self.gain, .8, 4310)
            different, _ = benchmark.run_case(self.core, plant, 1., 5., self.gain, .8, 4311)
        np.testing.assert_array_equal(first, repeated)
        self.assertEqual(metrics, repeated_metrics)
        self.assertFalse(np.array_equal(first[:, benchmark.IX["measured_position_rad"]],
                                        different[:, benchmark.IX["measured_position_rad"]]))
        self.assertTrue(np.all(np.isfinite(first)))
        command = first[:, benchmark.IX["command_nm"]]
        self.assertLessEqual(float(np.max(np.abs(command))), 7. + 1e-6)
        self.assertLessEqual(float(np.max(np.abs(np.diff(command)))), 1000. * benchmark.DT + 1e-6)
        self.assertEqual(first[0, benchmark.IX["status"]], 1)  # exactly one warmup
        np.testing.assert_array_equal(first[1:, benchmark.IX["status"]], 0)
        self.assertEqual(metrics["fault_count"], 0)

    def test_real_c_limits_fault_latch_and_explicit_reset(self):
        config = benchmark.make_config(self.gain, 0.)
        config.torque_limit_nm = .2
        config.torque_slew_nm_s = 1.
        controller = self.core.init(config)
        feedback = Feedback(0., 0., 0., 0., True, False)
        reference = Reference(.1, 0., 0., 0., True)

        def raw_step():
            output = Output()
            status = self.core.lib.yaw_controller_step(
                ct.byref(controller), ct.byref(feedback), ct.byref(reference),
                benchmark.DT, ct.byref(output))
            self.assertEqual(status, output.status)
            return status, output

        status, output = raw_step()
        self.assertEqual(status, 1)
        self.assertEqual(output.torque_nm, 0.)
        last_torque = 0.
        for _ in range(40):
            status, output = raw_step()
            self.assertEqual(status, 0)
            self.assertLessEqual(abs(output.torque_nm), .2 + 1e-6)
            self.assertLessEqual(abs(output.torque_nm - last_torque), .001 + 1e-6)
            last_torque = output.torque_nm
        self.assertGreater(last_torque, .02)
        self.assertTrue(self.core.lib.yaw_controller_set_torque_limit(ct.byref(controller), .005))
        status, output = raw_step()
        self.assertEqual(status, 0)
        self.assertLessEqual(abs(output.torque_nm), .005 + 1e-7)  # derating wins over slew

        feedback.age_s = config.feedback_timeout_s * 2
        status, output = raw_step()
        self.assertEqual(status, 6)  # YAW_STALE_FEEDBACK
        self.assertEqual(output.torque_nm, 0.)
        feedback.age_s = 0.
        status, output = raw_step()
        self.assertEqual(status, 6)  # fresh input does not silently clear a fault
        self.assertEqual(output.torque_nm, 0.)
        self.core.lib.yaw_controller_reset(ct.byref(controller))
        status, output = raw_step()
        self.assertEqual(status, 1)
        self.assertEqual(output.torque_nm, 0.)
        status, output = raw_step()
        self.assertEqual(status, 0)
        self.assertLessEqual(abs(output.torque_nm), .005 + 1e-7)


if __name__ == "__main__":
    unittest.main()
