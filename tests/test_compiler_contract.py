"""Reject compiler modes that can remove the C finite-input checks."""
import os
from pathlib import Path
import shlex
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
SOURCES = (
    "src/yaw_controller.c",
    "src/gimbal_controller.c",
    "src/gimbal_coordinates.c",
    "src/dm_mit.c",
    "src/gm6020.c",
    "examples/stm32/yaw_periodic.c",
    "examples/stm32/gimbal_periodic.c",
    "experimental/online_rls/src/rls_shadow.c",
)


class CompilerContractTests(unittest.TestCase):
    def compile_sources(self, flags, accepted):
        # Exercise the actual translation units, including optional/standalone
        # code. A CMake-only flag check would miss STM32 source-file imports.
        compiler = shlex.split(os.environ.get("CC", "cc"))
        with tempfile.TemporaryDirectory(prefix="gimbal-compiler-") as directory:
            for source in SOURCES:
                with self.subTest(source=source, flags=flags):
                    result = subprocess.run(
                        [*compiler, "-std=c11", *flags,
                         "-I", str(ROOT / "include"),
                         "-I", str(ROOT / "examples/stm32"),
                         "-I", str(ROOT / "experimental/online_rls/include"),
                         "-c", str(ROOT / source),
                         "-o", str(Path(directory) / "probe.o")],
                        capture_output=True, text=True, check=False,
                    )
                    if accepted:
                        self.assertEqual(result.returncode, 0, result.stderr)
                    else:
                        self.assertNotEqual(result.returncode, 0)
                        self.assertIn(
                            "Finite-value checks require disabling fast-math and finite-math-only",
                            result.stderr,
                        )

    def test_normal_optimization_builds(self):
        self.compile_sources(["-O2", "-fno-fast-math"], accepted=True)

    def test_finite_math_only_is_rejected(self):
        # This mode does not define __FAST_MATH__, but can fold isfinite(NAN)
        # to true. It previously bypassed every source-level compiler guard.
        self.compile_sources(["-O2", "-ffinite-math-only"], accepted=False)

    def test_fast_math_is_rejected(self):
        self.compile_sources(["-O2", "-ffast-math"], accepted=False)

    def test_ofast_is_rejected(self):
        self.compile_sources(["-Ofast"], accepted=False)


if __name__ == "__main__":
    unittest.main()
