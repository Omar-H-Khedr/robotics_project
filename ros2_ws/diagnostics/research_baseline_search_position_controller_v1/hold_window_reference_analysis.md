# Hold Window Reference Analysis

- input_dir: `diagnostics/research_baseline_search_position_controller_v1`
- command_source: `trajectory_commands.csv`
- tracking_source: `trajectory_controller_state_samples.csv`
- physical_clearance_m: `0.001000`
- state_loop_hz: `10.0`
- hold_like_command_count: `7`
- hold_like_best_feedback_1mm_ticks: `0`

| Cmd | Hold | Samples | Duration s | Target XY m | Target depth m | Ref mean XY m | Ref max XY m | Feedback mean XY m | Feedback max XY m | Feedback final XY m | Ref best 1mm ticks | Feedback best 1mm ticks | P95 JTC error rad |
| ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 0 | no | 0 | 40.000 | 0.000000 | 0.000000 | none | none | none | none | none | 0 | 0 | none |
| 1 | no | 0 | 15.000 | 0.000000 | 0.000000 | none | none | none | none | none | 0 | 0 | none |
| 2 | yes | 0 | 5.000 | 0.003001 | 0.000000 | none | none | none | none | none | 0 | 0 | none |
| 3 | yes | 0 | 5.000 | 0.000009 | 0.000000 | none | none | none | none | none | 0 | 0 | none |
| 4 | yes | 0 | 5.000 | 0.000007 | 0.000000 | none | none | none | none | none | 0 | 0 | none |
| 5 | yes | 0 | 5.000 | 0.000001 | 0.000000 | none | none | none | none | none | 0 | 0 | none |
| 6 | yes | 0 | 5.000 | 0.000005 | 0.000000 | none | none | none | none | none | 0 | 0 | none |
| 7 | yes | 0 | 5.000 | 0.000002 | 0.000000 | none | none | none | none | none | 0 | 0 | none |
| 8 | yes | 0 | 5.000 | 0.000001 | 0.000000 | none | none | none | none | none | 0 | 0 | none |
| 9 | no | 0 | 25.000 | 0.409676 | 0.000000 | none | none | none | none | none | 0 | 0 | none |

Interpretation: this is an offline passive-log diagnostic. A hold-like command is a single-point trajectory lasting at least 1 s. The analyzer compares the controller reference and feedback in each command window; it does not publish commands or alter the task safety gates.
