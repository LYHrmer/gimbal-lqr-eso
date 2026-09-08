# Two motor profiles — actual C controllers and C wire quantization

SYNTHETIC, same assumed load and disturbances. Compare original/new within each motor; do not interpret motor-to-motor differences as an algorithm advantage.
The primary case was fixed before running: 2 Hz / 5 deg, development seeds 4310–4314 and held-out validation seeds 4315–4319. Every planned seed and all four trajectories are retained.
For an integral-enabled profile, Original core preserves Ki0; Original core (same Ki) runs identical integral tuning. Gains from adding integral must not be presented as a pure core-implementation advantage.

| Motor / trajectory (seed 4310) | Original Ki0 RMSE (deg) | Original same-Ki RMSE (deg) | New RMSE (deg) | Change vs Ki0 | New command cap / envelope |
| --- | ---: | ---: | ---: | ---: | ---: |
| dm4310_24v_1hz_5deg | 0.02507 | unavailable | 0.02461 | -1.83% | 0.0% / 0.0% |
| dm4310_24v_2hz_5deg | 0.02787 | unavailable | 0.02593 | -6.98% | 0.0% / 0.0% |
| dm4310_24v_3hz_5deg | 0.03357 | unavailable | 0.02613 | -22.16% | 0.0% / 0.0% |
| dm4310_24v_3hz_10deg | 0.05641 | unavailable | 0.04208 | -25.41% | 0.0% / 0.0% |
| gm6020_current_v1_4_1hz_5deg | 0.03556 | 0.02925 | 0.02728 | -23.29% | 0.0% / 0.0% |
| gm6020_current_v1_4_2hz_5deg | 0.02578 | 0.01671 | 0.01679 | -34.87% | 0.0% / 0.0% |
| gm6020_current_v1_4_3hz_5deg | 0.40381 | 0.35437 | 0.37639 | -6.79% | 42.2% / 0.0% |
| gm6020_current_v1_4_3hz_10deg | 9.08022 | 9.13016 | 9.13246 | +0.58% | 86.2% / 0.0% |

Development — preselected 2 Hz / 5 deg:

| Motor / seed | Original Ki0 / new RMSE (deg) | Original / new max error (deg) | Original / new command RMS (N.m) | Command cap: old / new | Envelope clipping: old / new | Change vs Ki0 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| dm4310_24v / 4310 | 0.02787 / 0.02593 | 0.07069 / 0.06476 | 0.50293 / 0.50247 | 0.0% / 0.0% | 0.0% / 0.0% | -6.98% |
| dm4310_24v / 4311 | 0.02749 / 0.02546 | 0.06477 / 0.05970 | 0.50299 / 0.50241 | 0.0% / 0.0% | 0.0% / 0.0% | -7.39% |
| dm4310_24v / 4312 | 0.02800 / 0.02603 | 0.06439 / 0.06013 | 0.50304 / 0.50252 | 0.0% / 0.0% | 0.0% / 0.0% | -7.04% |
| dm4310_24v / 4313 | 0.02698 / 0.02470 | 0.06292 / 0.05700 | 0.50279 / 0.50236 | 0.0% / 0.0% | 0.0% / 0.0% | -8.44% |
| dm4310_24v / 4314 | 0.02779 / 0.02566 | 0.06720 / 0.05833 | 0.50299 / 0.50248 | 0.0% / 0.0% | 0.0% / 0.0% | -7.65% |
| gm6020_current_v1_4 / 4310 | 0.02578 / 0.01679 | 0.06496 / 0.03904 | 0.50810 / 0.50800 | 0.0% / 0.0% | 0.0% / 0.0% | -34.87% |
| gm6020_current_v1_4 / 4311 | 0.02648 / 0.01606 | 0.06210 / 0.05151 | 0.50821 / 0.50760 | 0.0% / 0.0% | 0.0% / 0.0% | -39.34% |
| gm6020_current_v1_4 / 4312 | 0.02403 / 0.01579 | 0.05921 / 0.04308 | 0.50824 / 0.50785 | 0.0% / 0.0% | 0.0% / 0.0% | -34.28% |
| gm6020_current_v1_4 / 4313 | 0.02386 / 0.01672 | 0.05617 / 0.04281 | 0.50790 / 0.50759 | 0.0% / 0.0% | 0.0% / 0.0% | -29.91% |
| gm6020_current_v1_4 / 4314 | 0.02490 / 0.01624 | 0.05750 / 0.03833 | 0.50797 / 0.50750 | 0.0% / 0.0% | 0.0% / 0.0% | -34.80% |

dm4310_24v: 5/5 seeds have lower RMSE; mean paired change -7.50%, worst change -6.98%.

gm6020_current_v1_4: 5/5 seeds have lower RMSE; mean paired change -34.64%, worst change -29.91%.

Development — isolate the same-integral implementation comparison:

| Motor / seed | Same-Ki original RMSE (deg) | New RMSE (deg) | Change vs same-Ki original |
| --- | ---: | ---: | ---: |
| gm6020_current_v1_4 / 4310 | 0.01671 | 0.01679 | +0.49% |
| gm6020_current_v1_4 / 4311 | 0.01622 | 0.01606 | -0.97% |
| gm6020_current_v1_4 / 4312 | 0.01645 | 0.01579 | -3.99% |
| gm6020_current_v1_4 / 4313 | 0.01699 | 0.01672 | -1.57% |
| gm6020_current_v1_4 / 4314 | 0.01580 | 0.01624 | +2.76% |

gm6020_current_v1_4: mean change vs same-Ki original -0.65%.

Held-out validation — preselected 2 Hz / 5 deg:

| Motor / seed | Original Ki0 / new RMSE (deg) | Original / new max error (deg) | Original / new command RMS (N.m) | Command cap: old / new | Envelope clipping: old / new | Change vs Ki0 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| dm4310_24v / 4315 | 0.02776 / 0.02552 | 0.06546 / 0.05905 | 0.50295 / 0.50244 | 0.0% / 0.0% | 0.0% / 0.0% | -8.08% |
| dm4310_24v / 4316 | 0.02704 / 0.02514 | 0.06791 / 0.06292 | 0.50287 / 0.50240 | 0.0% / 0.0% | 0.0% / 0.0% | -7.05% |
| dm4310_24v / 4317 | 0.02753 / 0.02594 | 0.06290 / 0.05845 | 0.50297 / 0.50245 | 0.0% / 0.0% | 0.0% / 0.0% | -5.78% |
| dm4310_24v / 4318 | 0.02727 / 0.02508 | 0.06672 / 0.05879 | 0.50291 / 0.50243 | 0.0% / 0.0% | 0.0% / 0.0% | -8.04% |
| dm4310_24v / 4319 | 0.02711 / 0.02560 | 0.06543 / 0.05850 | 0.50293 / 0.50249 | 0.0% / 0.0% | 0.0% / 0.0% | -5.57% |
| gm6020_current_v1_4 / 4315 | 0.02580 / 0.01605 | 0.06371 / 0.04651 | 0.50817 / 0.50770 | 0.0% / 0.0% | 0.0% / 0.0% | -37.78% |
| gm6020_current_v1_4 / 4316 | 0.02484 / 0.01745 | 0.06026 / 0.04843 | 0.50833 / 0.50780 | 0.0% / 0.0% | 0.0% / 0.0% | -29.75% |
| gm6020_current_v1_4 / 4317 | 0.02662 / 0.01688 | 0.06418 / 0.05212 | 0.50829 / 0.50821 | 0.0% / 0.0% | 0.0% / 0.0% | -36.61% |
| gm6020_current_v1_4 / 4318 | 0.02435 / 0.01779 | 0.05421 / 0.04242 | 0.50836 / 0.50775 | 0.0% / 0.0% | 0.0% / 0.0% | -26.92% |
| gm6020_current_v1_4 / 4319 | 0.02530 / 0.01585 | 0.05806 / 0.03901 | 0.50823 / 0.50763 | 0.0% / 0.0% | 0.0% / 0.0% | -37.35% |

dm4310_24v: 5/5 seeds have lower RMSE; mean paired change -6.90%, worst change -5.57%.

gm6020_current_v1_4: 5/5 seeds have lower RMSE; mean paired change -33.68%, worst change -26.92%.

Held-out validation — isolate the same-integral implementation comparison:

| Motor / seed | Same-Ki original RMSE (deg) | New RMSE (deg) | Change vs same-Ki original |
| --- | ---: | ---: | ---: |
| gm6020_current_v1_4 / 4315 | 0.01421 | 0.01605 | +12.94% |
| gm6020_current_v1_4 / 4316 | 0.01666 | 0.01745 | +4.72% |
| gm6020_current_v1_4 / 4317 | 0.01755 | 0.01688 | -3.81% |
| gm6020_current_v1_4 / 4318 | 0.01703 | 0.01779 | +4.51% |
| gm6020_current_v1_4 / 4319 | 0.01560 | 0.01585 | +1.62% |

gm6020_current_v1_4: mean change vs same-Ki original +4.00%.
Negative change means lower error. Five paired synthetic seeds are a sensitivity check, not a confidence interval or hardware-performance proof.
All current/torque conversion uses this repository's real C codecs. GM6020 current mode is assumed confirmed only in this offline profile; no CAN is sent.
The 2 ms actuator lag is an explicit assumption and is not the GM6020 datasheet's 3 ms mechanical time constant.
Feedback uses an assumed calibrated, zero-centered uniform quantizer; only command conversion uses the actual C protocol. This is not bit-exact feedback CAN replay.
The torque caps do not establish all-speed continuous or stall thermal feasibility. CSVs retain the primary seed-4310 trajectories, and JSON retains every seed's error, command RMS, saturation and wire metrics.
