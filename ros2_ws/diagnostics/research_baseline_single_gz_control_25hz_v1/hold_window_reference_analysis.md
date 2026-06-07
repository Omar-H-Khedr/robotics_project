# Hold Window Reference Analysis

- input_dir: `diagnostics/research_baseline_single_gz_control_25hz_v1`
- command_source: `trajectory_commands.csv`
- tracking_source: `trajectory_controller_state_samples.csv`
- physical_clearance_m: `0.001000`
- state_loop_hz: `25.0`
- hold_like_command_count: `4`
- hold_like_best_feedback_1mm_ticks: `3`

| Cmd | Hold | Samples | Duration s | Target XY m | Target depth m | Ref mean XY m | Ref max XY m | Feedback mean XY m | Feedback max XY m | Feedback final XY m | Ref best 1mm ticks | Feedback best 1mm ticks | P95 JTC error rad |
| ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 0 | no | 10983 | 40.000 | 0.000000 | 0.000000 | 0.207423 | 0.410560 | 0.207050 | 0.414759 | 0.003203 | 105 | 2 | 0.013098 |
| 1 | no | 3576 | 15.000 | 0.000000 | 0.000000 | 0.000343 | 0.003270 | 0.002178 | 0.006184 | 0.002021 | 285 | 2 | 0.008294 |
| 2 | yes | 610 | 5.000 | 0.000001 | 0.000000 | 0.001311 | 0.001592 | 0.002091 | 0.005030 | 0.000524 | 1 | 1 | 0.008390 |
| 3 | yes | 557 | 5.000 | 0.000000 | 0.000000 | 0.002157 | 0.002918 | 0.003074 | 0.007377 | 0.002466 | 0 | 1 | 0.008241 |
| 4 | yes | 610 | 5.000 | 0.000000 | 0.000000 | 0.000731 | 0.001471 | 0.002189 | 0.005497 | 0.003760 | 60 | 3 | 0.008316 |
| 5 | yes | 461 | 5.000 | 0.000000 | 0.000000 | 0.001832 | 0.002244 | 0.002682 | 0.006253 | 0.001485 | 0 | 2 | 0.008451 |

Interpretation: this is an offline passive-log diagnostic. A hold-like command is a single-point trajectory lasting at least 1 s. The analyzer compares the controller reference and feedback in each command window; it does not publish commands or alter the task safety gates.
