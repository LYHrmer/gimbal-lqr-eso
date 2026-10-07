"""Compile/link an existing C++17 target without configuring this repo's project."""
from pathlib import Path
import json
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


@unittest.skipUnless(shutil.which("cmake") and shutil.which("c++"), "CMake and C++ compiler required")
class CmakePortTests(unittest.TestCase):
    def run_command(self, *args, success=True):
        result = subprocess.run(args, text=True, capture_output=True)
        if success:
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        else:
            self.assertNotEqual(result.returncode, 0, result.stdout + result.stderr)
        return result

    def test_both_motor_consumers_link_and_run(self):
        for motor in ("GM6020", "DM4310"):
            with self.subTest(motor=motor), tempfile.TemporaryDirectory() as directory:
                self.run_command("cmake", "-S", str(ROOT / "tests/cmake_port"), "-B", directory,
                                 "-DCMAKE_BUILD_TYPE=Release", "-DPORT_TEST_MOTOR=" + motor,
                                 "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON")
                self.run_command("cmake", "--build", directory, "--parallel", "2")
                self.run_command(str(Path(directory) / "port_consumer"))
                self.assertFalse((Path(directory) / "CTestTestfile.cmake").exists())
                commands = json.loads((Path(directory) / "compile_commands.json").read_text())
                files = {Path(entry["file"]).name for entry in commands}
                codec = "gm6020.c" if motor == "GM6020" else "dm_mit.c"
                self.assertEqual(files, {"main.cpp", "yaw_controller.c", "gimbal_controller.c",
                                         "gimbal_coordinates.c", "gimbal_periodic.c", codec})

    def test_invalid_motor_and_duplicate_attachment_rejected(self):
        for option, error in (("-DPORT_TEST_MOTOR=unknown", "GM6020 or DM4310"),
                              ("-DPORT_TEST_DUPLICATE=ON", "already has gimbal sources")):
            with self.subTest(option=option), tempfile.TemporaryDirectory() as directory:
                result = self.run_command("cmake", "-S", str(ROOT / "tests/cmake_port"),
                                          "-B", directory, option, success=False)
                self.assertIn(error, result.stdout + result.stderr)

    def test_unsafe_host_flags_are_not_silently_overridden(self):
        with tempfile.TemporaryDirectory() as directory:
            self.run_command("cmake", "-S", str(ROOT / "tests/cmake_port"), "-B", directory,
                             "-DCMAKE_C_FLAGS=-Ofast")
            result = self.run_command("cmake", "--build", directory, success=False)
            self.assertIn("Finite-value checks require disabling fast-math", result.stdout + result.stderr)


if __name__ == "__main__":
    unittest.main()
