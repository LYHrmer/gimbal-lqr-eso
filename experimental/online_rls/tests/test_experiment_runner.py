"""Host integration checks: artifact identity, paths and excitation frontend.

Run from any directory with:
python -m unittest discover -s experimental/online_rls/tests -p 'test_*.py' -v
"""

import argparse
import importlib.util
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np


EXPERIMENT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('online_rls_runner', EXPERIMENT / 'run_experiments.py')
runner = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(runner)


class RunnerIntegrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temporary = tempfile.TemporaryDirectory(prefix='shadow RLS integration ')
        cls.build = Path(cls.temporary.name) / 'host library with spaces'
        cls.snapshot = runner.source_snapshot()
        cls.library, cls.compiler = runner.build_library(
            cls.build, os.environ.get('CC', 'cc'), cls.snapshot)
        cls.lib, cls.provenance = runner.load_library(cls.library, cls.snapshot)

    @classmethod
    def tearDownClass(cls):
        cls.temporary.cleanup()

    def test_library_built_and_loaded_with_spaces(self):
        self.assertTrue(self.library.is_file())
        self.assertIn(' ', str(self.library))
        self.assertEqual(self.provenance['library_sha256'], runner.sha256_file(self.library))
        self.assertEqual(self.provenance['embedded_compiler'], self.compiler['version'])
        self.assertEqual(self.lib.rls_shadow_config_size(), runner.ct.sizeof(runner.Config))
        self.assertEqual(self.lib.rls_shadow_state_size(), runner.ct.sizeof(runner.State))

    def test_old_library_rejected_before_any_estimator_call(self):
        for changed_source in ('src/rls_shadow.c', 'include/rls_shadow.h'):
            with self.subTest(source=changed_source):
                newer_sources = dict(self.snapshot)
                newer_sources[changed_source] = '0' * 64
                with self.assertRaisesRegex(ValueError, 'different C source/header'):
                    runner.load_library(self.library, newer_sources)

    def test_identical_library_at_new_path_preserves_identity(self):
        # Artifact checking is based on contents/embedded metadata, not a
        # hard-coded filename, directory, or the last compiler invocation.
        renamed = Path(self.temporary.name) / 'relocated library.so'
        shutil.copyfile(self.library, renamed)
        _, metadata = runner.load_library(renamed, self.snapshot)
        self.assertEqual(metadata['library_sha256'], self.provenance['library_sha256'])
        self.assertEqual(metadata['library_source_sha256'], self.provenance['library_source_sha256'])

    def test_same_size_wrong_field_offsets_are_rejected(self):
        for name, first, second in (('Config', 6, 7), ('State', 3, 4)):
            original = getattr(runner, name)
            fields = list(original._fields_)
            fields[first], fields[second] = fields[second], fields[first]
            incompatible = type('Wrong' + name, (runner.ct.Structure,), {'_fields_': fields})
            self.assertEqual(runner.ct.sizeof(incompatible), runner.ct.sizeof(original))
            with self.subTest(struct=name), patch.object(runner, name, incompatible):
                with self.assertRaisesRegex(ValueError, 'ABI offset'):
                    runner.load_library(self.library, self.snapshot)

    def test_python_optimization_still_initializes_the_actual_c_estimator(self):
        code = '''
import importlib.util
import sys
spec = importlib.util.spec_from_file_location('runner', sys.argv[1])
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)
lib, _ = runner.load_library(sys.argv[2], runner.source_snapshot())
result = runner.run(lib, runner.source(0), 1.0)
if not result['accepted'] or result['state'].initialized != 1:
    raise SystemExit('The C estimator was not initialized under python -O.')
'''
        subprocess.run([sys.executable, '-s', '-O', '-c', code,
                        str(EXPERIMENT / 'run_experiments.py'), str(self.library)],
                       check=True, capture_output=True, text=True)

    def test_output_defaults_and_protected_published_results(self):
        args = runner.parse_args([])
        self.assertEqual(args.output, runner.REPOSITORY / 'build' / 'online-rls-results')
        self.assertEqual(args.build_dir, runner.REPOSITORY / 'build' / 'online-rls-host')
        self.assertEqual(runner.output_directory(self.build), self.build.resolve())
        for path in (runner.REPOSITORY / 'results', runner.REPOSITORY / 'results' / 'online_rls',
                     EXPERIMENT, EXPERIMENT / 'src'):
            with self.subTest(path=str(path)):
                with self.assertRaises(argparse.ArgumentTypeError):
                    runner.output_directory(path)

    def test_rank_deficient_stream_freezes_and_recovers_with_new_excitation(self):
        rank = runner.source(101, 'rank_deficient')
        normal = runner.source(0)
        prefix = 120  # Longer than the complete 100-window PE frontend memory.
        # A single estimator first sees rank-deficient data, then informative
        # data; its PE frontend must keep observing even while updates freeze.
        mixed = tuple(np.concatenate((rank[i][:prefix], normal[i]))
                      for i in range(5)) + ([],)
        result = runner.run(self.lib, mixed, 1.0)
        self.assertTrue(np.all(result['status'][:prefix] == 5))
        self.assertTrue(np.all(result['estimates'][:prefix] ==
                               np.array(runner.INITIAL, dtype=np.float32) * runner.SCALE))
        self.assertFalse(np.any(result['pe'][:prefix]))
        self.assertTrue(np.any(result['status'][prefix:] == 0))
        self.assertTrue(result['rejected_preserved_exactly'])
        self.assertLess(np.max(np.abs(result['estimates'][-1] / runner.SCALE - 1)), 0.01)

    def test_invalid_and_nonfinite_samples_do_not_poison_pe_history(self):
        result = runner.run(self.lib, runner.source(102, 'faults'),
                            float(np.exp(-runner.PERIOD / 5.0)))
        self.assertEqual(len(result['fault_rows']), 42)
        self.assertTrue(np.isfinite(result['min_eigenvalue']).all())
        self.assertTrue(np.all(result['status'][result['fault_rows']] != 0))
        self.assertTrue(result['rejected_preserved_exactly'])
        self.assertGreater(len(result['accepted']), 2500)
        self.assertLess(np.max(np.abs(result['estimates'][-1] / runner.SCALE - 1)), 0.01)


if __name__ == '__main__':
    unittest.main()
