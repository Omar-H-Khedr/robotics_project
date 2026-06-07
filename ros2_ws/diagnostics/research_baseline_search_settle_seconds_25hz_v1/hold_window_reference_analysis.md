# Hold Window Reference Analysis

- input_dir: `diagnostics/research_baseline_search_settle_seconds_25hz_v1`
- command_source: `trajectory_commands.csv`
- tracking_source: `trajectory_controller_state_samples.csv`
- physical_clearance_m: `0.001000`
- state_loop_hz: `25.0`
- hold_like_command_count: `5`
- hold_like_best_feedback_1mm_ticks: `4`

| Cmd | Hold | Samples | Duration s | Target XY m | Target depth m | Ref mean XY m | Ref max XY m | Feedback mean XY m | Feedback max XY m | Feedback final XY m | Ref best 1mm ticks | Feedback best 1mm ticks | P95 JTC error rad |
| ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 0 | no | 10140 | 40.000 | 0.000000 | 0.000000 | 0.226565 | 0.411273 | 0.225985 | 0.415727 | 0.001073 | 15 | 3 | 0.013570 |
| 1 | no | 4403 | 15.000 | 0.000000 | 0.000000 | 0.000223 | 0.002252 | 0.002101 | 0.007126 | 0.001774 | 371 | 3 | 0.008284 |
| 2 | yes | 1509 | 5.000 | 0.000001 | 0.000000 | 0.000715 | 0.001919 | 0.002373 | 0.006493 | 0.003246 | 100 | 4 | 0.008418 |
| 3 | yes | 1510 | 5.000 | 0.000008 | 0.000000 | 0.000648 | 0.001554 | 0.002333 | 0.006214 | 0.000415 | 106 | 2 | 0.008388 |
| 4 | yes | 1510 | 5.000 | 0.000008 | 0.000000 | 0.000486 | 0.001165 | 0.002215 | 0.005894 | 0.002615 | 133 | 3 | 0.008277 |
| 5 | yes | 1510 | 5.000 | 0.000003 | 0.000000 | 0.000895 | 0.002163 | 0.002219 | 0.005787 | 0.004260 | 84 | 2 | 0.008215 |
| 6 | yes | 394 | 5.000 | 0.000006 | 0.000000 | 0.002674 | 0.003174 | 0.003553 | 0.006396 | 0.001520 | 0 | 1 | 0.008093 |

Interpretation: this is an offline passive-log diagnostic. A hold-like command is a single-point trajectory lasting at least 1 s. The analyzer compares the controller reference and feedback in each command window; it does not publish commands or alter the task safety gates.
