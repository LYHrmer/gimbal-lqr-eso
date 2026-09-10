#!/usr/bin/env python3
"""Synthetic regression-stream checks of the actual float32 C shadow RLS.

Not an experiment on a motor, or a closed-loop control-performance benchmark.
Only this script's past-window PE statistic is implemented; C accepts its flag.
"""
import argparse
import ctypes as ct
import hashlib
import json
import os
from pathlib import Path
import platform
import shlex
import subprocess
import sys

import numpy as np

ROOT = Path(__file__).resolve().parent
REPOSITORY = ROOT.parents[1]
SOURCE_FILES = ('include/rls_shadow.h', 'src/rls_shadow.c',
                'src/rls_provenance.c.in', 'run_experiments.py')
DIM = 6
DT = 0.001
WINDOW = 20
PERIOD = DT * WINDOW
SCALE = np.array([0.025, 0.060])
YSCALE = 0.001
P0 = 100.0
INITIAL = np.array([0.6, 1.6])
SEEDS = list(range(10))
PROTOCOL = {
    'scope': 'Synthetic integral regression stream, actual float32 C RLS; no control-gain update or motor experiment',
    'duration_s': 60.0, 'source_dt_s': DT, 'nonoverlap_window_samples': WINDOW,
    'nominal_update_s': PERIOD, 'normalization_parameter_scale': SCALE.tolist(),
    'normalization_impulse_nm_s': YSCALE, 'initial_normalized_parameters': INITIAL.tolist(),
    'initial_inverse_information_diagonal': P0, 'independent_label_noise_normalized_std': 0.03,
    'seeds': SEEDS, 'pe_window_samples': 100, 'pe_min_eigenvalue': 0.002,
    'pe_max_condition': 1e4, 'forgetting_time_valid_updates_s': 5.0,
    'acceptance': {'baseline_each_parameter_error_max': 0.05,
                   'c_vs_weighted_batch_normalized_abs_max': 0.002,
                   'drift_tail_b_error_nm_s_rad_max': 0.01,
                   'drift_tail_error_ratio_to_no_forgetting_max': 0.5,
                   'all_rejected_updates_preserve_theta_and_p_exactly': True},
    'diagnostic_only_cases': ['regressor_noise', '20ms_torque_label_timestamp_shift', 'torque_scale_1.25'],
    'drift_scope': 'One seed (100), b ramps 20-40 s; MAE scored only at 50-60 s',
    'delay_scope': 'Torque labels shifted against prescribed motion; NOT a solved actuator-delay closed loop',
    'not_implemented': ['closed-loop bias correction', 'firmware timestamp/PE frontend',
                        'online gain switching', 'hardware validation', 'three-axis dynamics'],
}


class Config(ct.Structure):
    _fields_ = [('dimension', ct.c_uint32), ('forgetting_factor', ct.c_float),
                ('covariance_initial', ct.c_float), ('covariance_max', ct.c_float),
                ('innovation_limit', ct.c_float), ('theta_initial', ct.c_float * DIM),
                ('theta_min', ct.c_float * DIM), ('theta_max', ct.c_float * DIM)]


class State(ct.Structure):
    _fields_ = [('config', Config), ('theta', ct.c_float * DIM), ('p', ct.c_float * (DIM * DIM)),
                ('accepted_count', ct.c_uint32), ('rejected_count', ct.c_uint32),
                ('last_status', ct.c_uint32), ('initialized', ct.c_uint32)]


def sha256_file(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def source_snapshot():
    return {name: sha256_file(ROOT / name) for name in SOURCE_FILES}


def output_directory(value):
    path = Path(value).expanduser().resolve()
    # Keep published controller results and experiment sources read-only even
    # when a caller accidentally selects them as the output/build directory.
    for protected in (REPOSITORY / 'results', ROOT):
        if path == protected or protected in path.parents:
            raise argparse.ArgumentTypeError(
                'Use a build/ or external output directory; source and published results are protected.')
    return path


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=output_directory,
                        default=REPOSITORY / 'build' / 'online-rls-results',
                        help='Report/CSV/plot directory (default: repository build/online-rls-results).')
    parser.add_argument('--build-dir', type=output_directory,
                        default=REPOSITORY / 'build' / 'online-rls-host',
                        help='Host compilation directory (default: repository build/online-rls-host).')
    parser.add_argument('--cc', default=os.environ.get('CC', 'cc'),
                        help='Compiler command, e.g. "cc" or "ccache clang" (default: CC or cc).')
    parser.add_argument('--library', type=Path,
                        help='Use a matching shadow_rls_host library from this CMake project.')
    parser.add_argument('--no-plots', action='store_true',
                        help='Write JSON and CSV only; do not import matplotlib.')
    return parser.parse_args(argv)


def build_library(build_dir, compiler, snapshot):
    build_dir = output_directory(build_dir)
    build_dir.mkdir(parents=True, exist_ok=True)
    command = shlex.split(compiler)
    if not command:
        raise ValueError('The compiler command is empty.')
    version = subprocess.run(command + ['--version'], check=True, text=True,
                             capture_output=True).stdout.strip().splitlines()[0]
    # Use a generated host-only translation unit. The estimator remains
    # independent of host provenance metadata.
    substitutions = {
        'RLS_SOURCE_SHA256': snapshot['src/rls_shadow.c'],
        'RLS_HEADER_SHA256': snapshot['include/rls_shadow.h'],
        'RLS_COMPILER': json.dumps(version, ensure_ascii=True)[1:-1],
    }
    generated = (ROOT / 'src/rls_provenance.c.in').read_text()
    for key, value in substitutions.items():
        generated = generated.replace('@' + key + '@', value)
    provenance_source = build_dir / 'rls_provenance.c'
    provenance_source.write_text(generated)
    library = build_dir / ('libshadow_rls_host.dylib' if sys.platform == 'darwin'
                           else 'libshadow_rls_host.so')
    flags = ['-std=c11', '-O2', '-Wall', '-Wextra', '-Wpedantic', '-Werror',
             '-Wconversion', '-Wshadow', '-fPIC', '-shared']
    subprocess.run(command + flags + ['-I', str(ROOT / 'include'),
                   str(ROOT / 'src/rls_shadow.c'), str(provenance_source),
                   '-lm', '-o', str(library)], check=True)
    if source_snapshot() != snapshot:
        raise RuntimeError('Experiment sources changed while building; rerun from stable sources.')
    return library, {'command': [Path(part).name if Path(part).is_absolute() else part
                                for part in command],
                     'version': version, 'flags': flags}


def load_library(path, snapshot):
    library = Path(path).expanduser().resolve(strict=True)
    lib = ct.CDLL(str(library))
    try:
        for symbol in ('rls_shadow_source_sha256', 'rls_shadow_header_sha256',
                       'rls_shadow_compiler'):
            getattr(lib, symbol).argtypes = []
            getattr(lib, symbol).restype = ct.c_char_p
        for symbol in ('rls_shadow_config_size', 'rls_shadow_state_size'):
            getattr(lib, symbol).argtypes = []
            getattr(lib, symbol).restype = ct.c_size_t
        for symbol in ('rls_shadow_config_offset', 'rls_shadow_state_offset'):
            getattr(lib, symbol).argtypes = [ct.c_uint32]
            getattr(lib, symbol).restype = ct.c_size_t
        embedded = {
            'src/rls_shadow.c': lib.rls_shadow_source_sha256().decode('ascii'),
            'include/rls_shadow.h': lib.rls_shadow_header_sha256().decode('ascii'),
        }
        compiler = lib.rls_shadow_compiler().decode('utf-8')
    except (AttributeError, UnicodeError) as error:
        raise ValueError('The library lacks matching host provenance; build shadow_rls_host with this project.') from error
    if any(snapshot[name] != digest for name, digest in embedded.items()):
        raise ValueError('The library was built from different C source/header; rebuild shadow_rls_host.')
    if lib.rls_shadow_config_size() != ct.sizeof(Config) or lib.rls_shadow_state_size() != ct.sizeof(State):
        raise ValueError('The library ABI does not match the Python Config/State layout.')
    # Use fixed semantic field names, not the order of a possibly incorrect
    # ctypes _fields_ list. Equal total sizes do not imply equal field offsets.
    layouts = ((Config, lib.rls_shadow_config_offset,
                ('dimension', 'forgetting_factor', 'covariance_initial', 'covariance_max',
                 'innovation_limit', 'theta_initial', 'theta_min', 'theta_max')),
               (State, lib.rls_shadow_state_offset,
                ('config', 'theta', 'p', 'accepted_count', 'rejected_count',
                 'last_status', 'initialized')))
    for struct, c_offset, fields in layouts:
        for index, field in enumerate(fields):
            if getattr(struct, field).offset != c_offset(index):
                raise ValueError(f'The library ABI offset does not match Python {struct.__name__}.{field}.')
    lib.rls_shadow_init.argtypes = [ct.POINTER(State), ct.POINTER(Config)]
    lib.rls_shadow_init.restype = ct.c_int
    lib.rls_shadow_update.argtypes = [ct.POINTER(State), ct.POINTER(ct.c_float), ct.c_float, ct.c_int, ct.c_int]
    lib.rls_shadow_update.restype = ct.c_int
    return lib, {'library_file': library.name, 'library_sha256': sha256_file(library),
                 'library_source_sha256': embedded, 'library_source_matches': True,
                 'embedded_compiler': compiler}


def config(lam):
    c = Config()
    c.dimension, c.forgetting_factor = 2, lam
    c.covariance_initial, c.covariance_max, c.innovation_limit = P0, 1e6, 10.0
    for k in range(2):
        c.theta_initial[k], c.theta_min[k], c.theta_max[k] = INITIAL[k], 0.02, 5.0
    return c


def source(seed, case='normal'):
    rng = np.random.default_rng(seed)
    t = np.arange(round(60.0 / DT) + 1) * DT
    f = np.array([0.7, 2.3, 4.1])
    a = np.array([0.5, 0.2, 0.15])
    phase = 2 * np.pi * f[:, None] * t + rng.uniform(-np.pi, np.pi, (3, 1))
    w = (a[:, None] * np.sin(phase)).sum(axis=0)
    alpha = (a[:, None] * (2 * np.pi * f[:, None]) * np.cos(phase)).sum(axis=0)
    q = (-a[:, None] / (2 * np.pi * f[:, None]) * np.cos(phase)).sum(axis=0)
    b = np.full_like(t, SCALE[1])
    if case == 'drift':
        b += 0.04 * np.clip((t - 20) / 20, 0, 1)
    if case == 'rank_deficient':
        w = np.full_like(t, 0.3)
        alpha, q = np.zeros_like(t), 0.3 * t
    torque = SCALE[0] * alpha + b * w
    if case == 'delay':
        torque = np.r_[np.repeat(torque[0], 20), torque[:-20]]
    if case == 'scale':
        torque = 1.25 * torque
    # Endpoints are shared by neighboring windows, a relevant caveat in EIV.
    sampled_w = w[::WINDOW].copy()
    sampled_q = q[::WINDOW].copy()
    if case == 'eiv':
        sampled_w += rng.normal(0, 0.02, len(sampled_w))
    phi = np.column_stack([np.diff(sampled_w), np.diff(sampled_q)]) * SCALE / YSCALE
    impulse = ((torque[:-1] + torque[1:]) * (DT / 2)).reshape(-1, WINDOW).sum(axis=1)
    y = impulse / YSCALE + rng.normal(0, 0.03, len(impulse))
    valid = np.ones(len(y), dtype=bool)
    fault_rows = []
    if case == 'faults':
        for k in range(250, len(y), 200):
            # Validity flags emulate rejected source data, not a real CAN parser.
            valid[k] = False
            phi[k + 1, 0] = np.nan
            y[k + 2] = 1000.0
            fault_rows.extend([k, k + 1, k + 2])
    return t[WINDOW::WINDOW], phi, y, valid, b[WINDOW::WINDOW], fault_rows


def run(lib, dataset, lam):
    t, phi, y, valid, b, fault_rows = dataset
    c, s = config(lam), State()
    init_status = lib.rls_shadow_init(ct.byref(s), ct.byref(c))
    if init_status != 0:
        raise RuntimeError(f'C RLS initialization failed with status {init_status}.')
    start_theta, start_p = bytes(s.theta), bytes(s.p)
    estimates, statuses, flags, evs = [], [], [], []
    recent = []
    accepted = []
    preserved = True
    for k, (x, label) in enumerate(zip(phi, y)):
        finite = np.isfinite(x).all() and np.isfinite(label)
        good = bool(valid[k] and finite and abs(label) < 100)
        contribution = np.zeros((2, 2))
        if good:
            # Accumulate in float64 even when an input array uses float32.
            # Finite features can still overflow their outer product; such a
            # contribution must not poison the subsequent observation window.
            with np.errstate(over='ignore', invalid='ignore'):
                candidate = np.outer(np.asarray(x, dtype=np.float64),
                                     np.asarray(x, dtype=np.float64))
            if np.isfinite(candidate).all():
                contribution = candidate
        recent.append(contribution)
        if len(recent) > 100:
            recent.pop(0)
        # Rebuild the small past-window Gram matrix instead of subtracting an
        # expired large value from an accumulated sum. Otherwise cancellation
        # can permanently erase the clean samples still in the window. Scale
        # before summation so averaging finite contributions need not overflow.
        with np.errstate(over='ignore', invalid='ignore'):
            matrix = np.sum(np.asarray(recent) / len(recent), axis=0)
        eig = (np.linalg.eigvalsh(matrix) if np.isfinite(matrix).all()
               else np.full(2, np.nan))
        informative = (len(recent) == 100 and np.isfinite(eig).all()
                       and eig[0] > 0.002 and eig[1] / 1e4 < eig[0])
        arr = (ct.c_float * 2)(*x)
        before = (bytes(s.theta), bytes(s.p))
        status = lib.rls_shadow_update(ct.byref(s), arr, ct.c_float(label), int(valid[k]), int(informative))
        if status == 0:
            accepted.append(k)
        else:
            preserved &= before == (bytes(s.theta), bytes(s.p))
        estimates.append(np.ctypeslib.as_array(s.theta)[:2].copy() * SCALE)
        statuses.append(status)
        flags.append(informative)
        evs.append(eig[0])
    return {'time': t, 'estimates': np.array(estimates), 'status': np.array(statuses),
            'pe': np.array(flags), 'min_eigenvalue': np.array(evs), 'accepted': accepted,
            'state': s, 'rejected_preserved_exactly': preserved,
            'unchanged_from_init': start_theta == bytes(s.theta) and start_p == bytes(s.p),
            'b_truth': b, 'fault_rows': fault_rows}


def batch_same_prior(phi, y, accepted, lam):
    # Includes float32-rounded features, labels, initial parameters and lambda.
    x = phi[accepted].astype(np.float32).astype(float)
    labels = y[accepted].astype(np.float32).astype(float)
    n = len(accepted)
    weights = float(np.float32(lam)) ** np.arange(n - 1, -1, -1)
    initial = INITIAL.astype(np.float32).astype(float)
    prior = float(np.float32(lam)) ** n / P0
    lhs = (x.T * weights) @ x + prior * np.eye(2)
    rhs = (x.T * weights) @ labels + prior * initial
    return np.linalg.solve(lhs, rhs)


def summarize_errors(values):
    v = np.array(values)
    return {'mean_relative_error': v.mean(axis=0).tolist(),
            'max_absolute_relative_error': np.abs(v).max(axis=0).tolist()}


def trial_record(seed, result, batch=None):
    record = {'seed': seed,
              'final_normalized_parameters': (result['estimates'][-1] / SCALE).tolist(),
              'accepted_updates': len(result['accepted']),
              'rejected_updates': int(result['state'].rejected_count),
              'rejected_preserved_exactly': bool(result['rejected_preserved_exactly'])}
    if batch is not None:
        record['batch_normalized_parameters'] = batch.tolist()
    return record


def main(argv=None):
    args = parse_args(argv)
    out = output_directory(args.output)
    out.mkdir(parents=True, exist_ok=True)
    snapshot = source_snapshot()
    if args.library is None:
        library, compiler = build_library(args.build_dir, args.cc, snapshot)
    else:
        library = args.library.expanduser().resolve(strict=True)
        compiler = None  # This process did not invoke the compiler for --library.
    lib, provenance = load_library(library, snapshot)
    provenance.update({
        'library_path': os.path.relpath(library, REPOSITORY),
        'build_mode': 'runner_compiled' if compiler is not None else 'existing_library',
        'compiler': compiler,
        'platform': {'system': platform.system(), 'machine': platform.machine(),
                     'python': platform.python_version(), 'numpy': np.__version__},
    })
    # OUT is deliberately local: importing this module never creates output or
    # compiles C, and two independent command invocations can select directories.
    OUT = out
    (OUT / 'protocol.json').write_text(json.dumps(PROTOCOL, indent=2) + '\n')
    baseline_errors, batch_errors = [], []
    normal_trials = []
    diagnostic_trials = {name: [] for name in ('eiv', 'delay', 'scale')}
    diagnostics = {'eiv': [], 'delay': [], 'scale': []}
    traces = {}
    frozen_ok = True
    for seed in SEEDS:
        data = source(seed)
        result = run(lib, data, 1.0)
        baseline_errors.append(result['estimates'][-1] / SCALE - 1)
        batch = batch_same_prior(data[1], data[2], result['accepted'], 1.0)
        normal_trials.append(trial_record(seed, result, batch))
        batch_errors.append(np.max(np.abs(result['estimates'][-1] / SCALE - batch)))
        frozen_ok &= result['rejected_preserved_exactly']
        if seed == 0:
            traces['normal'] = result
        for case in diagnostics:
            bad = run(lib, source(seed, case), 1.0)
            diagnostics[case].append(bad['estimates'][-1] / SCALE - 1)
            diagnostic_trials[case].append(trial_record(seed, bad))
            frozen_ok &= bad['rejected_preserved_exactly']
    drift = source(100, 'drift')
    lam = float(np.exp(-PERIOD / 5.0))
    slow, forgetting = run(lib, drift, 1.0), run(lib, drift, lam)
    drift_batch = batch_same_prior(drift[1], drift[2], forgetting['accepted'], lam)
    drift_batch_difference = float(np.max(np.abs(forgetting['estimates'][-1] / SCALE - drift_batch)))
    tail = drift[0] >= 50.0
    old_error = float(np.mean(np.abs(slow['estimates'][tail, 1] - drift[4][tail])))
    new_error = float(np.mean(np.abs(forgetting['estimates'][tail, 1] - drift[4][tail])))
    rank = run(lib, source(101, 'rank_deficient'), lam)
    faults = run(lib, source(102, 'faults'), lam)
    fault_rejected = all(faults['status'][k] != 0 for k in faults['fault_rows'])
    frozen_ok &= all(r['rejected_preserved_exactly'] for r in (slow, forgetting, rank, faults))
    gates = {
        'normal_parameter_recovery': bool(np.max(np.abs(baseline_errors)) < 0.05),
        'normal_matches_same_prior_batch': max(batch_errors) < 0.002,
        'forgetting_matches_same_prior_batch': drift_batch_difference < 0.002,
        'slow_drift_tracking': new_error < 0.01 and new_error < 0.5 * old_error,
        'rank_deficient_freezes': rank['unchanged_from_init'] and len(rank['accepted']) == 0,
        'fault_rows_rejected': fault_rejected,
        'every_rejection_preserves_parameters_and_p': bool(frozen_ok),
    }
    gates = {name: bool(passed) for name, passed in gates.items()}
    report = {
        'schema_version': 1,
        'scope': PROTOCOL['scope'], 'protocol': 'protocol.json',
        'source_sha256': snapshot,
        'provenance': provenance,
        'ctypes_sizes': {'config': ct.sizeof(Config), 'state': ct.sizeof(State)},
        'normal_trials': normal_trials,
        'diagnostic_trials': diagnostic_trials,
        'normal_10_seeds': summarize_errors(baseline_errors),
        'max_c_vs_batch_abs_normalized_difference': max(batch_errors),
        'drift': {'seed': 100,
                  'forgetting_factor_float32': float(np.float32(lam)),
                  'no_forgetting_tail_b_absolute_error': old_error,
                  'forgetting_tail_b_absolute_error': new_error,
                  'c_vs_weighted_batch_abs_normalized_difference': drift_batch_difference,
                  'no_forgetting_final_normalized_parameters': (slow['estimates'][-1] / SCALE).tolist(),
                  'forgetting_final_normalized_parameters': (forgetting['estimates'][-1] / SCALE).tolist(),
                  'weighted_batch_normalized_parameters': drift_batch.tolist(),
                  'no_forgetting_accepted_updates': len(slow['accepted']),
                  'forgetting_accepted_updates': len(forgetting['accepted']),
                  'no_forgetting_rejected_preserved_exactly': bool(slow['rejected_preserved_exactly']),
                  'forgetting_rejected_preserved_exactly': bool(forgetting['rejected_preserved_exactly'])},
        'rank_deficient_accepted': len(rank['accepted']),
        'rank_deficient': {'seed': 101,
                           'accepted_updates': len(rank['accepted']),
                           'rejected_updates': int(rank['state'].rejected_count),
                           'unchanged_from_init': rank['unchanged_from_init'],
                           'rejected_preserved_exactly': bool(rank['rejected_preserved_exactly'])},
        'fault_rows': len(faults['fault_rows']), 'faults_accepted_updates': len(faults['accepted']),
        'faults': {'seed': 102, 'fault_rows': faults['fault_rows'],
                   'fault_row_statuses': faults['status'][faults['fault_rows']].tolist(),
                   'accepted_updates': len(faults['accepted']),
                   'rejected_updates': int(faults['state'].rejected_count),
                   'rejected_preserved_exactly': bool(faults['rejected_preserved_exactly'])},
        'faults_final_relative_parameter_error': (faults['estimates'][-1] / SCALE - 1).tolist(),
        'diagnostic_mismatch_cases': {k: summarize_errors(v) for k, v in diagnostics.items()},
        'gates': gates, 'all_declared_algorithm_gates_passed': all(gates.values()),
        'performance_of_controller_tested': False,
    }
    for name, r in {'normal': traces['normal'], 'drift_no_forgetting': slow,
                    'drift_forgetting': forgetting, 'rank_deficient': rank, 'faults': faults}.items():
        np.savetxt(OUT / (name + '.csv'), np.column_stack([r['time'], r['estimates'], r['status'], r['pe'], r['min_eigenvalue']]),
                   delimiter=',', header='time_s,J_kg_m2,b_Nms_rad,status,excitation_ok,gram_min_eigenvalue', comments='')
    if not args.no_plots:
        plot_traces(OUT, traces, drift, slow, forgetting, rank, baseline_errors, diagnostics)
    provenance['source_unchanged_during_run'] = source_snapshot() == snapshot
    provenance['library_unchanged_during_run'] = sha256_file(library) == provenance['library_sha256']
    (OUT / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))
    if not provenance['source_unchanged_during_run'] or not provenance['library_unchanged_during_run']:
        raise SystemExit('Sources or library changed during the run; discard this run and rerun.')
    if not all(gates.values()):
        raise SystemExit('One or more predeclared algorithm checks failed; retain the report.')


def plot_traces(OUT, traces, drift, slow, forgetting, rank, baseline_errors, diagnostics):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axs = plt.subplots(2, 2, figsize=(11, 7), constrained_layout=True)
    r = traces['normal']
    axs[0, 0].plot(r['time'], r['estimates'][:, 0], label='C RLS estimate')
    axs[0, 0].axhline(SCALE[0], c='k', ls='--', label='synthetic truth')
    axs[0, 0].set(title='A. Inertia, sufficient excitation', ylabel='J [kg m^2]', xlabel='Time [s]')
    axs[0, 0].legend()
    axs[0, 1].plot(drift[0], drift[4], 'k--', label='synthetic truth')
    axs[0, 1].plot(drift[0], slow['estimates'][:, 1], label='lambda = 1')
    axs[0, 1].plot(drift[0], forgetting['estimates'][:, 1], label='5 s memory over valid updates')
    axs[0, 1].set(title='B. Slow damping change', ylabel='b [N m s/rad]', xlabel='Time [s]')
    axs[0, 1].legend(fontsize=8)
    axs[1, 0].plot(rank['time'], rank['estimates'][:, 0] / SCALE[0], label='J estimate / truth')
    axs[1, 0].plot(rank['time'], rank['estimates'][:, 1] / SCALE[1], label='b estimate / truth')
    axs[1, 0].set(title='C. Constant speed: no identifiable pair, frozen', xlabel='Time [s]', ylabel='Normalized parameters')
    axs[1, 0].legend(fontsize=8)
    labels = ['Correct data', 'Velocity noise', '20 ms label shift', 'Torque scale x1.25']
    vals = [np.array(baseline_errors)] + [np.array(diagnostics[k]) for k in ['eiv', 'delay', 'scale']]
    xpos = np.arange(4)
    for d, label in enumerate(['J', 'b']):
        axs[1, 1].bar(xpos + (d - 0.5) * 0.34, [100 * v.mean(axis=0)[d] for v in vals], width=0.34, label=label)
    axs[1, 1].axhline(0, color='k', linewidth=0.5)
    axs[1, 1].set(title='D. Stable estimates can still be wrong (10 seeds)', ylabel='Mean signed parameter error [%]')
    axs[1, 1].set_xticks(xpos)
    axs[1, 1].set_xticklabels(labels, rotation=15, ha='right', fontsize=8)
    axs[1, 1].legend()
    fig.suptitle('Synthetic regression streams, actual float32 C RLS — no controller adaptation', fontsize=12)
    fig.savefig(OUT / 'online_rls_checks.png', dpi=180)
    fig.savefig(OUT / 'online_rls_checks.svg')
    plt.close(fig)


if __name__ == '__main__':
    main()
