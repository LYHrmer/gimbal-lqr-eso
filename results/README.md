# Synthetic benchmark results

These are generated plant simulations, not DM4310 measurements. Both variants call the same C core.

| Scenario | RMSE: ESO 0 (deg) | RMSE: ESO 0.8 (deg) | ESO torque saturation | Torque-infeasible reference |
| --- | ---: | ---: | ---: | --- |
| nominal_1hz_5deg | 0.0003 | 0.0002 | 0.0% | no |
| nominal_3hz_20deg | 0.0205 | 0.0229 | 0.0% | no |
| nominal_5hz_20deg | 13.1105 | 13.1122 | 92.0% | yes |
| stressed_1hz_5deg | 0.2418 | 0.0531 | 0.0% | no |
| stressed_3hz_20deg | 0.6863 | 0.4225 | 0.0% | no |
| stressed_5hz_20deg | 16.0197 | 16.2090 | 93.0% | yes |

Metrics evaluate 2–8 s after a C2 smooth startup. Negative `eso_rmse_change_percent` means lower RMSE.
At 5 Hz / 20 deg, the assumed load requires more than the 7 N.m simulation torque cap; controller tuning cannot remove this physical constraint.
7 N.m is a synthetic challenge condition, not an approved hardware setting.

`metrics.json` records all configurations, seed, dependency versions, source hashes, and ABI sizes.
`stressed_3hz_20deg.csv.gz` retains aligned full trajectories for both variants; rerun with `--all-csv` for every case.
Each scenario has an editable SVG and PNG containing tracking and actual-position error panels.
