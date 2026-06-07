# Hold Window Reference Analysis

- input_dir: `diagnostics/research_baseline_search_post_settle_count_recenter8_gain3000_damping10_25hz_v1`
- command_source: `trajectory_commands.csv`
- tracking_source: `trajectory_controller_state_samples.csv`
- physical_clearance_m: `0.001000`
- state_loop_hz: `25.0`
- hold_like_command_count: `1`
- hold_like_best_feedback_1mm_ticks: `5`

| Cmd | Hold | Samples | Duration s | Target XY m | Target depth m | Ref mean XY m | Ref max XY m | Feedback mean XY m | Feedback max XY m | Feedback final XY m | Ref best 1mm ticks | Feedback best 1mm ticks | P95 JTC error rad |
| ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 0 | no | 10035 | 40.000 | 0.000000 | 0.000000 | 0.227174 | 0.408880 | 0.226786 | 0.411517 | 0.001173 | 5 | 1 | 0.012767 |
| 1 | no | 2940 | 15.000 | 0.000000 | 0.000000 | 0.000382 | 0.002353 | 0.001285 | 0.004260 | 0.000662 | 220 | 9 | 0.007598 |
| 2 | yes | 3020 | 2.000 | 0.000000 | 0.000000 | 0.000110 | 0.001463 | 0.001221 | 0.003679 | 0.001375 | 289 | 5 | 0.007530 |
| 3 | no | 6268 | 25.000 | 0.409676 | 0.000000 | 0.132784 | 0.409676 | 0.132844 | 0.410783 | 0.409833 | 244 | 6 | 0.011746 |

Interpretation: this is an offline passive-log diagnostic. A hold-like command is a single-point trajectory lasting at least 1 s. The analyzer compares the controller reference and feedback in each command window; it does not publish commands or alter the task safety gates.
