# Original C versus new C — synthetic comparison

Both implementations run against the same plant, seed, smooth sinusoidal reference, DLQR gains and ESO gain 0.8. Integral and upstream bias are disabled.
The pinned upstream source is compiled unchanged through an adapter; see `metrics.json` for its commit, hashes, configuration mapping and unavailable features.
This normalizes the controller settings. It does not reproduce the original forum's non-sinusoidal `/plan_yaw` trajectory or establish either controller's best hardware performance.

| Scenario | Original RMSE (deg) | New RMSE (deg) | RMSE change | Original / new commands at torque limit |
| --- | ---: | ---: | ---: | ---: |
| nominal_1hz_5deg | 0.00091 | 0.00024 | -73.11% | 0.0% / 0.0% |
| nominal_3hz_20deg | 0.06094 | 0.02291 | -62.40% | 0.0% / 0.0% |
| nominal_5hz_20deg | 13.11190 | 13.11222 | +0.00% | 87.0% / 87.0% |
| stressed_1hz_5deg | 0.05371 | 0.05314 | -1.06% | 0.0% / 0.0% |
| stressed_3hz_20deg | 0.48102 | 0.42247 | -12.17% | 0.0% / 0.0% |
| stressed_5hz_20deg | 16.21533 | 16.20905 | -0.04% | 86.9% / 86.9% |

Negative RMSE change means the new controller has lower error; positive means higher error. All six cases are retained, including regressions.
Metrics evaluate 2–8 s. The at-limit metric uses the actual command magnitude, so it can be compared without assuming matching flag support in the implementations.
The 5 Hz / 20 deg synthetic cases exceed the assumed 7 N.m limit; this says nothing about the feasibility of the author's different real trajectory and load.
7 N.m is a simulation challenge setting, not a hardware default. Neither the new ESO ablation nor these software comparisons prove a hardware advantage.

`stressed_3hz_20deg.png` / `.svg` show representative tracking and true-position error; `.csv.gz` contains both full trajectories.
Reproduce with `python sim/compare_upstream.py` after building the new shared library and obtaining the pinned upstream files as documented in `sim/upstream_reference.py`.
