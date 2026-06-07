# Hold Window Reference Analysis

- input_dir: `diagnostics/research_baseline_done_shutdown_recenter8_settle9_gain3000_damping10_25hz_v1`
- command_source: `trajectory_commands.csv`
- tracking_source: `trajectory_controller_state_samples.csv`
- physical_clearance_m: `0.001000`
- state_loop_hz: `25.0`
- hold_like_command_count: `3`
- hold_like_best_feedback_1mm_ticks: `6`

| Cmd | Hold | Samples | Duration s | Target XY m | Target depth m | Ref mean XY m | Ref max XY m | Feedback mean XY m | Feedback max XY m | Feedback final XY m | Ref best 1mm ticks | Feedback best 1mm ticks | P95 JTC error rad |
| ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 0 | no | 9966 | 40.000 | 0.000000 | 0.000000 | 0.226023 | 0.409676 | 0.225627 | 0.410713 | 0.001094 | 5 | 4 | 0.013012 |
| 1 | no | 5153 | 15.000 | 0.000000 | 0.000000 | 0.000109 | 0.001090 | 0.001183 | 0.003493 | 0.001335 | 465 | 6 | 0.007539 |
| 2 | yes | 2260 | 8.000 | 0.000001 | 0.000000 | 0.000160 | 0.000233 | 0.001138 | 0.003227 | 0.000388 | 226 | 6 | 0.007543 |
| 3 | yes | 2271 | 8.000 | 0.000001 | 0.000000 | 0.000297 | 0.000673 | 0.001259 | 0.003477 | 0.001212 | 227 | 4 | 0.007572 |
| 4 | yes | 1957 | 8.000 | 0.000003 | 0.000000 | 0.000200 | 0.000392 | 0.001218 | 0.003175 | 0.001312 | 196 | 4 | 0.007683 |

Interpretation: this is an offline passive-log diagnostic. A hold-like command is a single-point trajectory lasting at least 1 s. The analyzer compares the controller reference and feedback in each command window; it does not publish commands or alter the task safety gates.
