# Hold Window Reference Analysis

- input_dir: `diagnostics/research_baseline_search_recenter_4mm_v1`
- command_source: `trajectory_commands.csv`
- tracking_source: `trajectory_controller_state_samples.csv`
- physical_clearance_m: `0.001000`
- state_loop_hz: `10.0`
- hold_like_command_count: `1`
- hold_like_best_feedback_1mm_ticks: `3`

| Cmd | Hold | Samples | Duration s | Target XY m | Target depth m | Ref mean XY m | Ref max XY m | Feedback mean XY m | Feedback max XY m | Feedback final XY m | Ref best 1mm ticks | Feedback best 1mm ticks | P95 JTC error rad |
| ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 0 | no | 13173 | 40.000 | 0.000000 | 0.000000 | 0.174298 | 0.411877 | 0.174340 | 0.414874 | 0.001455 | 127 | 2 | 0.012778 |
| 1 | no | 3075 | 15.000 | 0.000000 | 0.000000 | 0.000223 | 0.001419 | 0.002065 | 0.005793 | 0.002157 | 98 | 3 | 0.008249 |
| 2 | yes | 101 | 2.000 | 0.000000 | 0.000000 | 0.000744 | 0.000790 | 0.001810 | 0.004100 | 0.001073 | 5 | 3 | 0.007924 |
| 3 | no | 33817 | 25.000 | 0.409676 | 0.000000 | 0.358332 | 0.409676 | 0.358041 | 0.414967 | 0.411230 | 87 | 2 | 0.011793 |

Interpretation: this is an offline passive-log diagnostic. A hold-like command is a single-point trajectory lasting at least 1 s. The analyzer compares the controller reference and feedback in each command window; it does not publish commands or alter the task safety gates.
