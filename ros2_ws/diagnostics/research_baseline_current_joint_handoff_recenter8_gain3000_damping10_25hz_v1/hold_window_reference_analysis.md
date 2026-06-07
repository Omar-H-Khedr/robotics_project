# Hold Window Reference Analysis

- input_dir: `diagnostics/research_baseline_current_joint_handoff_recenter8_gain3000_damping10_25hz_v1`
- command_source: `trajectory_commands.csv`
- tracking_source: `trajectory_controller_state_samples.csv`
- physical_clearance_m: `0.001000`
- state_loop_hz: `25.0`
- hold_like_command_count: `4`
- hold_like_best_feedback_1mm_ticks: `7`

| Cmd | Hold | Samples | Duration s | Target XY m | Target depth m | Ref mean XY m | Ref max XY m | Feedback mean XY m | Feedback max XY m | Feedback final XY m | Ref best 1mm ticks | Feedback best 1mm ticks | P95 JTC error rad |
| ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 0 | no | 10018 | 40.000 | 0.000000 | 0.000000 | 0.227100 | 0.409676 | 0.226732 | 0.411582 | 0.002249 | 4 | 1 | 0.013034 |
| 1 | no | 5127 | 15.000 | 0.000000 | 0.000000 | 0.000131 | 0.001443 | 0.001180 | 0.003546 | 0.001767 | 451 | 7 | 0.007560 |
| 2 | yes | 2259 | 8.000 | 0.000001 | 0.000000 | 0.000426 | 0.001134 | 0.001352 | 0.003819 | 0.001337 | 210 | 7 | 0.007722 |
| 3 | yes | 2280 | 8.000 | 0.000003 | 0.000000 | 0.000731 | 0.001664 | 0.001492 | 0.004288 | 0.000715 | 148 | 3 | 0.007521 |
| 4 | yes | 2280 | 8.000 | 0.000002 | 0.000000 | 0.000185 | 0.000424 | 0.001191 | 0.003608 | 0.001285 | 228 | 6 | 0.007564 |
| 5 | yes | 2180 | 8.000 | 0.000002 | 0.000000 | 0.000671 | 0.001460 | 0.001472 | 0.003981 | 0.000386 | 155 | 6 | 0.007603 |
| 6 | no | 1353 | 25.000 | 0.409676 | 0.000000 | 0.001511 | 0.001748 | 0.001872 | 0.004997 | 0.001185 | 23 | 3 | 0.007548 |

Interpretation: this is an offline passive-log diagnostic. A hold-like command is a single-point trajectory lasting at least 1 s. The analyzer compares the controller reference and feedback in each command window; it does not publish commands or alter the task safety gates.
