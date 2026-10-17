"""Protect protocol-defined resolution while allowing explicit synthetic noise."""
from copy import deepcopy
import json
import unittest

from tools.check_motor_profiles import ROOT, PROFILE_FILES, check_profiles, validate_profile


class MotorProfileContractTests(unittest.TestCase):
    def profiles(self):
        return [json.loads((ROOT/"profiles"/name).read_text()) for name in PROFILE_FILES]

    def test_shipped_quantization_agrees_with_production_c_decoders(self):
        self.assertTrue(check_profiles()["passed"])

    def test_rejects_changed_resolution_and_false_imu_substitution(self):
        for profile in self.profiles():
            for key in ("position_quantum_rad", "velocity_quantum_rad_s"):
                for value in (0., 1e-6, float("nan"), True):
                    changed = deepcopy(profile)
                    changed["sensor"][key] = value
                    with self.subTest(profile=profile["name"], field=key, value=value):
                        with self.assertRaises(ValueError):
                            validate_profile(changed)
            changed = deepcopy(profile)
            changed["feedback_source"] = "imu"
            with self.assertRaises(ValueError):
                validate_profile(changed)

    def test_ranges_ids_modes_and_application_limits_are_distinct(self):
        dm, gm = self.profiles()
        for key, value in (("current_mode_confirmed", False), ("current_limit_a", 3.1),
                           ("motor_id", 8), ("motor_id", True), ("torque_constant_nm_a", 1e308)):
            changed = deepcopy(gm)
            changed["protocol"][key] = value
            with self.assertRaises(ValueError):
                validate_profile(changed)
        dm["protocol"]["v_max_rad_s"] *= 2
        with self.assertRaises(ValueError):
            validate_profile(dm)
        gm["controller"]["torque_limit_nm"] = 2.2
        with self.assertRaises(ValueError):
            validate_profile(gm)


if __name__ == "__main__":
    unittest.main()
