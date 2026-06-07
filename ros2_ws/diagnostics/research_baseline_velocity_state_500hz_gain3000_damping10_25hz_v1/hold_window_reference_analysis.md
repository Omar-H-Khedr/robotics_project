# Hold Window Reference Analysis

- input_dir: `diagnostics/research_baseline_velocity_state_500hz_gain3000_damping10_25hz_v1`
- command_source: `trajectory_commands.csv`
- tracking_source: `trajectory_controller_state_samples.csv`
- physical_clearance_m: `0.001000`
- state_loop_hz: `25.0`
- hold_like_command_count: `2`
- hold_like_best_feedback_1mm_ticks: `97`

| Cmd | Hold | Samples | Duration s | Target XY m | Target depth m | Ref mean XY m | Ref max XY m | Feedback mean XY m | Feedback max XY m | Feedback final XY m | Ref best 1mm ticks | Feedback best 1mm ticks | P95 JTC error rad |
| ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 0 | no | 20016 | 40.000 | 0.000000 | 0.000000 | 0.228321 | 0.409676 | 0.228152 | 0.410517 | 0.000292 | 3 | 3 | 0.006283 |
| 1 | no | 5727 | 15.000 | 0.000000 | 0.000000 | 0.000094 | 0.000942 | 0.000497 | 0.001895 | 0.000627 | 289 | 66 | 0.003701 |
| 2 | yes | 1140 | 2.000 | 0.000000 | 0.000000 | 0.000124 | 0.000447 | 0.000542 | 0.001505 | 0.000778 | 57 | 28 | 0.003645 |
| 3 | yes | 11526 | 20.000 | 0.000000 | 0.020000 | 0.000334 | 0.000614 | 0.000559 | 0.001671 | 0.000383 | 577 | 97 | 0.003774 |
| 4 | no | 12572 | 25.000 | 0.409676 | 0.000000 | 0.101479 | 0.409676 | 0.101577 | 0.410607 | 0.410012 | 345 | 82 | 0.005268 |

Interpretation: this is an offline passive-log diagnostic. A hold-like command is a single-point trajectory lasting at least 1 s. The analyzer compares the controller reference and feedback in each command window; it does not publish commands or alter the task safety gates.
