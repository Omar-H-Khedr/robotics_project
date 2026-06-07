# Hold Window Reference Analysis

- input_dir: `diagnostics/research_baseline_velocity_state_500hz_repeat_v1/trial_02_tracking`
- command_source: `trajectory_commands.csv`
- tracking_source: `trajectory_controller_state_samples.csv`
- physical_clearance_m: `0.001000`
- state_loop_hz: `25.0`
- hold_like_command_count: `2`
- hold_like_best_feedback_1mm_ticks: `34`

| Cmd | Hold | Samples | Duration s | Target XY m | Target depth m | Ref mean XY m | Ref max XY m | Feedback mean XY m | Feedback max XY m | Feedback final XY m | Ref best 1mm ticks | Feedback best 1mm ticks | P95 JTC error rad |
| ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 0 | no | 20046 | 40.000 | 0.000000 | 0.000000 | 0.228238 | 0.409773 | 0.228077 | 0.410975 | 0.000554 | 4 | 4 | 0.006274 |
| 1 | no | 5884 | 15.000 | 0.000000 | 0.000000 | 0.000121 | 0.000651 | 0.000495 | 0.001782 | 0.000588 | 295 | 103 | 0.003750 |
| 2 | yes | 1244 | 2.000 | 0.000000 | 0.000000 | 0.000111 | 0.000413 | 0.000532 | 0.001471 | 0.000165 | 63 | 34 | 0.003728 |
| 3 | yes | 182 | 20.000 | 0.000000 | 0.020000 | 0.000640 | 0.000675 | 0.000806 | 0.001638 | 0.000963 | 10 | 5 | 0.003547 |
| 4 | no | 12466 | 25.000 | 0.409676 | 0.000000 | 0.131356 | 0.408941 | 0.131343 | 0.409514 | 0.409107 | 251 | 12 | 0.005756 |

Interpretation: this is an offline passive-log diagnostic. A hold-like command is a single-point trajectory lasting at least 1 s. The analyzer compares the controller reference and feedback in each command window; it does not publish commands or alter the task safety gates.
