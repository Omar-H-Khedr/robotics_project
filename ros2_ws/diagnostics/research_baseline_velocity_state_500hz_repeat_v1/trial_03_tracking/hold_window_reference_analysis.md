# Hold Window Reference Analysis

- input_dir: `diagnostics/research_baseline_velocity_state_500hz_repeat_v1/trial_03_tracking`
- command_source: `trajectory_commands.csv`
- tracking_source: `trajectory_controller_state_samples.csv`
- physical_clearance_m: `0.001000`
- state_loop_hz: `25.0`
- hold_like_command_count: `2`
- hold_like_best_feedback_1mm_ticks: `38`

| Cmd | Hold | Samples | Duration s | Target XY m | Target depth m | Ref mean XY m | Ref max XY m | Feedback mean XY m | Feedback max XY m | Feedback final XY m | Ref best 1mm ticks | Feedback best 1mm ticks | P95 JTC error rad |
| ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 0 | no | 20027 | 40.000 | 0.000000 | 0.000000 | 0.228676 | 0.410185 | 0.228511 | 0.411455 | 0.000465 | 4 | 5 | 0.006295 |
| 1 | no | 5796 | 15.000 | 0.000000 | 0.000000 | 0.000097 | 0.000457 | 0.000497 | 0.001523 | 0.000514 | 290 | 117 | 0.003704 |
| 2 | yes | 1140 | 2.000 | 0.000000 | 0.000000 | 0.000112 | 0.000210 | 0.000481 | 0.001384 | 0.000507 | 57 | 38 | 0.003805 |
| 3 | yes | 1426 | 20.000 | 0.000000 | 0.020000 | 0.000704 | 0.000857 | 0.000759 | 0.001883 | 0.000888 | 72 | 19 | 0.003795 |
| 4 | no | 12338 | 25.000 | 0.409676 | 0.000000 | 0.129107 | 0.403429 | 0.129027 | 0.404230 | 0.403077 | 1 | 3 | 0.005681 |

Interpretation: this is an offline passive-log diagnostic. A hold-like command is a single-point trajectory lasting at least 1 s. The analyzer compares the controller reference and feedback in each command window; it does not publish commands or alter the task safety gates.
