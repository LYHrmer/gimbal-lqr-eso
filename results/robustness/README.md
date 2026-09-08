# Synthetic DM4310 sensitivity matrix

Fixed controller gains and nominal J/B. All original 24 scenarios are retained, plus two paired 4 Hz boundary cases added when the initial set showed no speed-envelope activity. Both actual C implementations receive the same plant and seed. This is not a sample of typical RoboMaster hardware.

| Scenario | Original / new RMSE (deg) | Change | New command cap | New envelope clipping | New fault |
| --- | ---: | ---: | ---: | ---: | --- |
| rated3_base_3hz10 | 0.0882 / 0.0736 | -16.52% | 11.1% | 0.0% | None |
| rated3_1hz5 | 0.0550 / 0.0544 | -1.12% | 0.0% | 0.0% | None |
| rated3_2hz10 | 0.0599 / 0.0562 | -6.15% | 0.0% | 0.0% | None |
| rated3_3hz20 | 15.2802 / 15.2800 | -0.00% | 96.1% | 0.0% | None |
| peak7_3hz20 | 0.1126 / 0.0846 | -24.88% | 0.0% | 0.0% | None |
| inertia_x0.5 | 0.2904 / 0.2920 | +0.55% | 0.0% | 0.0% | None |
| inertia_x0.75 | 0.1321 / 0.1425 | +7.84% | 0.0% | 0.0% | None |
| inertia_x1.25 | 0.7179 / 0.6743 | -6.07% | 37.4% | 0.0% | None |
| inertia_x1.5 | 7.9372 / 7.9174 | -0.25% | 92.6% | 0.0% | None |
| damping_x0.5 | 0.1510 / 0.1454 | -3.68% | 0.1% | 0.0% | None |
| damping_x1.5 | 0.3479 / 0.3237 | -6.96% | 21.2% | 0.0% | None |
| damping_x2 | 0.8193 / 0.7909 | -3.47% | 33.0% | 0.0% | None |
| ideal_measurement | 0.0866 / 0.0716 | -17.26% | 10.9% | 0.0% | None |
| noise_std_x3 | 0.0929 / 0.0773 | -16.73% | 10.7% | 0.0% | None |
| command_delay_0ms | 0.0876 / 0.0724 | -17.35% | 10.6% | 0.0% | None |
| command_delay_3ms | 0.0949 / 0.0802 | -15.51% | 11.3% | 0.0% | None |
| command_delay_5ms | 0.1081 / 0.0941 | -12.96% | 11.6% | 0.0% | None |
| command_delay_10ms | 0.1590 / 0.1495 | -5.95% | 12.2% | 0.0% | None |
| actuator_lag_5ms | 0.1038 / 0.0889 | -14.38% | 11.7% | 0.0% | None |
| peak7_3hz20_no_speed_envelope | 0.1126 / 0.0846 | -24.88% | 0.0% | 0.0% | None |
| compound_2hz10 | 0.1828 / 0.1650 | -9.72% | 0.0% | 0.0% | None |
| author_runtime_2hz10 | 0.2542 / 0.2324 | -8.58% | 0.3% | 0.0% | None |
| author_runtime_3hz10 | 6.5490 / 6.5454 | -0.05% | 91.0% | 0.0% | None |
| author_candidate_3hz10 | 0.1898 / 0.1658 | -12.63% | 17.6% | 0.0% | None |
| peak7_4hz20_speed_envelope | 10.7358 / 10.7519 | +0.15% | 89.5% | 62.8% | None |
| peak7_4hz20_no_speed_envelope | 9.6615 / 9.6001 | -0.64% | 88.6% | 0.0% | None |

Negative change means reduced RMSE. Regressions, saturations and faults remain in the matrix.
The 3 N.m cap uses a rated-specification reference, not an all-speed continuous or thermal guarantee. 7 N.m cases are explicit peak assumptions without a permissible duration model.
Actuator clipping is applied during every RK4 evaluation, before assumed torque lag. The clipping fraction counts control intervals with any such limiting; command-cap saturation is reported separately.
The recorded crossover/margins are for the linear feedback-only ZOH model with lag/delay. They exclude ESO, friction and saturation and are not a stability certificate for the full controller.
Representative traces include the base case and the largest relative regression, selected after retaining every case in this table.
Read ../../docs/robustness.md and ../../docs/parameter_sources.md for parameter meanings and limits.
