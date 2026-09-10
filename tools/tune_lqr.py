#!/usr/bin/env python3
"""Discrete ZOH LQR design for torque -> [position, velocity]."""
import argparse
import json
import math
from pathlib import Path

import numpy as np
import scipy
from scipy.linalg import solve_discrete_are
from scipy.signal import cont2discrete


def design_lqr(inertia=0.039, damping=0.30, dt=0.001,
               q_position=1600.0, q_velocity=1.0, r_torque=0.2):
    values = dict(inertia=inertia, damping=damping, dt=dt,
                  q_position=q_position, q_velocity=q_velocity, r_torque=r_torque)
    for name, value in values.items():
        if name == "damping":
            if not math.isfinite(value) or value < 0:
                raise ValueError("damping must be finite and nonnegative")
            continue
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f"{name} must be finite and positive")
    continuous_a = np.array([[0., 1.], [0., -damping / inertia]])
    continuous_b = np.array([[0.], [1. / inertia]])
    a, b, _, _, _ = cont2discrete(
        (continuous_a, continuous_b, np.eye(2), np.zeros((2, 1))), dt, method="zoh")
    q = np.diag([q_position, q_velocity])
    r = np.array([[r_torque]])
    p = solve_discrete_are(a, b, q, r)
    gain = np.linalg.solve(r + b.T @ p @ b, b.T @ p @ a)
    poles = np.linalg.eigvals(a - b @ gain)
    radius = float(np.max(np.abs(poles)))
    if not np.all(np.isfinite(gain)) or not math.isfinite(radius) or radius >= 1.0:
        raise ValueError("design is non-finite or closed-loop spectral radius >= 1")
    residual = a.T @ p @ a - p - a.T @ p @ b @ gain + q
    return {
        "model": "J * acceleration + B * velocity = torque; exact ZOH",
        "feedback_convention": "torque = K @ (reference_state - measured_state)",
        "parameters": values,
        "state_units": ["rad", "rad/s"], "input_units": "N.m",
        "ad": a.tolist(), "bd": b.tolist(), "q": q.tolist(), "r": r.tolist(),
        "gain": {"k_position": float(gain[0, 0]), "k_velocity": float(gain[0, 1])},
        "closed_loop_poles": [{"real": float(p.real), "imag": float(p.imag)} for p in poles],
        "spectral_radius": radius,
        "dare_residual_max_abs": float(np.max(np.abs(residual))),
        "dependencies": {"numpy": np.__version__, "scipy": scipy.__version__},
        "scope": "nominal linear unsaturated model; not a hardware stability certificate",
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--inertia", type=float, default=.039, help="kg m^2, assumed load")
    parser.add_argument("--damping", type=float, default=.30, help="N.m s/rad, nonnegative")
    parser.add_argument("--dt", type=float, default=.001, help="seconds")
    parser.add_argument("--q-position", type=float, default=1600.)
    parser.add_argument("--q-velocity", type=float, default=1.)
    parser.add_argument("--r-torque", type=float, default=.2)
    parser.add_argument("--output", type=Path, default=Path("results/lqr_design.json"))
    args = parser.parse_args()
    try:
        result = design_lqr(args.inertia, args.damping, args.dt,
                            args.q_position, args.q_velocity, args.r_torque)
    except ValueError as error:
        parser.error(str(error))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    print(json.dumps({key: result[key] for key in
                      ("gain", "closed_loop_poles", "spectral_radius", "scope")}, indent=2))
    print(f"Saved {args.output}")


if __name__ == "__main__":
    main()
