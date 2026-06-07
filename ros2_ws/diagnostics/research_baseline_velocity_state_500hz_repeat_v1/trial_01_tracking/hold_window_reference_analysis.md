# Hold Window Reference Analysis

- input_dir: `diagnostics/research_baseline_velocity_state_500hz_repeat_v1/trial_01_tracking`
- command_source: `trajectory_commands.csv`
- tracking_source: `trajectory_controller_state_samples.csv`
- physical_clearance_m: `0.001000`
- state_loop_hz: `25.0`
- hold_like_command_count: `2`
- hold_like_best_feedback_1mm_ticks: `59`

| Cmd | Hold | Samples | Duration s | Target XY m | Target depth m | Ref mean XY m | Ref max XY m | Feedback mean XY m | Feedback max XY m | Feedback final XY m | Ref best 1mm ticks | Feedback best 1mm ticks | P95 JTC error rad |
| ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 0 | no | 20033 | 40.000 | 0.000000 | 0.000000 | 0.228560 | 0.410187 | 0.228394 | 0.411087 | 0.000533 | 4 | 5 | 0.006300 |
| 1 | no | 5830 | 15.000 | 0.000000 | 0.000000 | 0.000064 | 0.000467 | 0.000460 | 0.001646 | 0.000381 | 292 | 103 | 0.003729 |
| 2 | yes | 1139 | 2.000 | 0.000000 | 0.000000 | 0.000150 | 0.000491 | 0.000514 | 0.001350 | 0.000840 | 57 | 36 | 0.003824 |
| 3 | yes | 11543 | 20.000 | 0.000000 | 0.020000 | 0.000493 | 0.000829 | 0.000648 | 0.001810 | 0.000198 | 578 | 59 | 0.003782 |
| 4 | no | 12493 | 25.000 | 0.409676 | 0.000000 | 0.099636 | 0.409676 | 0.099662 | 0.410395 | 0.409752 | 345 | 33 | 0.005161 |

Interpretation: this is an offline passive-log diagnostic. A hold-like command is a single-point trajectory lasting at least 1 s. The analyzer compares the controller reference and feedback in each command window; it does not publish commands or alter the task safety gates.
