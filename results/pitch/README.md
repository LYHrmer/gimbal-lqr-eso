# Synthetic pitch validation

Actual C controllers and actual C motor command codecs. All three variants have identical gains, including Ki; GM6020 is not compared against a substituted Ki=0 baseline.
The fixed primary is +20 deg centre, 1 Hz / 5 deg amplitude, gravity 0.45*cos(theta)+0.10*sin(theta) N.m, 2 ms actuator lag + 1 ms command delay. J/B and gravity are illustrative assumptions, not identified RoboMaster parameters.
Development seeds 5300-5302; held-out seeds 5303-5307. Startup 0-2 s is reported separately; primary error uses 2-6 s. Physical boundary crossings are failures, never clipped trajectories.

| Motor / split | Original same-Ki RMSE (deg) | New gravity off | New gravity on | Mean paired change vs original | Vs gravity off | Fixed acceptance |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| dm4310_24v_pitch / development | 0.055997 | 0.055693 | 0.015566 | -72.201209% | -72.048784% | True |
| dm4310_24v_pitch / holdout | 0.056054 | 0.055859 | 0.015395 | -72.534716% | -72.437896% | True |

## dm4310_24v_pitch: every preselected stress case (seed 5300)

| Case | Original / off / on RMSE (deg) | Change vs original | Fault or boundary failure |
| --- | ---: | ---: | --- |
| primary | 0.056248 / 0.055931 / 0.015421 | -72.583094% | none |
| gravity_x0.75 | 0.042080 / 0.041846 / 0.025840 | -38.591593% | none |
| gravity_x1.25 | 0.071025 / 0.070915 / 0.016543 | -76.708666% | none |
| gravity_phase_15deg | 0.052073 / 0.052133 / 0.017698 | -66.013404% | none |
| inertia_x0.75 | 0.056269 / 0.055779 / 0.015479 | -72.490872% | none |
| inertia_x1.25 | 0.056977 / 0.056625 / 0.016662 | -70.756253% | none |
| noise_x3 | 0.057355 / 0.057588 / 0.017344 | -69.761104% | none |
| delay_10ms | 0.057101 / 0.057003 / 0.018171 | -68.178478% | none |
| tracking_1.5hz | 0.057088 / 0.056733 / 0.016709 | -70.730551% | none |
| negative_pitch | 0.050014 / 0.050022 / 0.015558 | -68.892408% | none |
| near_upper_limit | 0.047336 / 0.047247 / 0.010840 | -77.100387% | none |
| base_tilt_15deg | 0.052320 / 0.052221 / 0.015397 | -70.571373% | none |
| wrong_gravity_sign | 0.056248 / 0.055931 / 0.116051 | 106.320531% | none |
| balanced_load_wrong_model | 0.016226 / 0.015334 / 0.068550 | 322.480113% | none |
| high_gravity_boundary | 0.237634 / 0.237675 / 0.176729 | -25.629960% | none |
| reference_outside_joint_margin | incomplete / 61.393690 / 61.361296 | incomplete% | Original C (same Ki): physical joint boundary crossed; no position clamping/contact model; New C, gravity off: physical joint boundary crossed; no position clamping/contact model; New C, gravity on: physical joint boundary crossed; no position clamping/contact model |

Independent seed 5308, RK4 4 vs 8 substeps: True (all four scalar metrics within 1%, complete position traces within 0.01 deg).
| gm6020_current_v1_4_pitch / development | 0.028789 | 0.028735 | 0.028372 | -1.411918% | -1.228684% | False |
| gm6020_current_v1_4_pitch / holdout | 0.027355 | 0.028304 | 0.026511 | -2.937698% | -6.303781% | True |

## gm6020_current_v1_4_pitch: every preselected stress case (seed 5300)

| Case | Original / off / on RMSE (deg) | Change vs original | Fault or boundary failure |
| --- | ---: | ---: | --- |
| primary | 0.028752 / 0.028553 / 0.028869 | 0.409389% | none |
| gravity_x0.75 | 0.029395 / 0.028602 / 0.028772 | -2.117368% | none |
| gravity_x1.25 | 0.036377 / 0.036222 / 0.029140 | -19.893686% | none |
| gravity_phase_15deg | 0.028395 / 0.029564 / 0.029092 | 2.453912% | none |
| inertia_x0.75 | 0.025346 / 0.027211 / 0.025989 | 2.539435% | none |
| inertia_x1.25 | 0.031433 / 0.032237 / 0.031026 | -1.294924% | none |
| noise_x3 | 0.021694 / 0.023025 / 0.023112 | 6.537851% | none |
| delay_10ms | 0.067714 / 0.064471 / 0.119461 | 76.419321% | none |
| tracking_1.5hz | 0.025414 / 0.021765 / 0.023146 | -8.924569% | none |
| negative_pitch | 0.028374 / 0.028127 / 0.029889 | 5.338424% | none |
| near_upper_limit | 0.041454 / 0.044798 / 0.045353 | 9.404144% | none |
| base_tilt_15deg | 0.029206 / 0.030262 / 0.027415 | -6.132310% | none |
| wrong_gravity_sign | 0.028752 / 0.028553 / 0.178747 | 521.694623% | none |
| balanced_load_wrong_model | 0.026416 / 0.027160 / 0.031614 | 19.677357% | none |
| high_gravity_boundary | incomplete / incomplete / incomplete | incomplete% | Original C (same Ki): physical joint boundary crossed; no position clamping/contact model; New C, gravity off: physical joint boundary crossed; no position clamping/contact model; New C, gravity on: physical joint boundary crossed; no position clamping/contact model |
| reference_outside_joint_margin | incomplete / 61.607654 / 61.591622 | incomplete% | Original C (same Ki): physical joint boundary crossed; no position clamping/contact model; New C, gravity off: physical joint boundary crossed; no position clamping/contact model; New C, gravity on: physical joint boundary crossed; no position clamping/contact model |

development acceptance failures: Original C (same Ki): mean paired RMSE improvement below fixed threshold; Original C (same Ki): insufficient improved seeds; New C, gravity off: mean paired RMSE improvement below fixed threshold; New C, gravity off: insufficient improved seeds; New C, gravity off: mean position_max_abs_error_deg exceeds comparison limit

Independent seed 5308, RK4 4 vs 8 substeps: True (all four scalar metrics within 1%, complete position traces within 0.01 deg).

Negative percentage means lower RMSE. A passed primary gate does not establish improvement in every stress case or on hardware.
The gravity-off/new comparison isolates the gravity feature; the upstream comparison also includes core implementation differences. Gravity must use a calibrated physical angle. Wrong sign, load mismatch, long delay and inadequate torque remain in the results.
An assumed fixed chassis tilt is known exactly; joint encoder noise/quantization is transformed into world angle. No dynamic IMU fusion, coupled yaw-pitch motion, flexible transmission, hard-stop contact, thermal or regeneration model is included.
No-feedback or out-of-range software zero is not a brake: unsupported pitch can fall. Boundary traces stop when the physical range is first crossed.
