# Hold Window Reference Analysis

- input_dir: `diagnostics/research_baseline_search_recenter_4mm_v1_repeat2`
- command_source: `trajectory_commands.csv`
- tracking_source: `trajectory_controller_state_samples.csv`
- physical_clearance_m: `0.001000`
- state_loop_hz: `10.0`
- hold_like_command_count: `7`
- hold_like_best_feedback_1mm_ticks: `2`

| Cmd | Hold | Samples | Duration s | Target XY m | Target depth m | Ref mean XY m | Ref max XY m | Feedback mean XY m | Feedback max XY m | Feedback final XY m | Ref best 1mm ticks | Feedback best 1mm ticks | P95 JTC error rad |
| ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 0 | no | 11550 | 40.000 | 0.000000 | 0.000000 | 0.197567 | 0.408350 | 0.197297 | 0.412052 | 0.002404 | 62 | 4 | 0.013038 |
| 1 | no | 4447 | 15.000 | 0.000000 | 0.000000 | 0.000222 | 0.001931 | 0.002155 | 0.005221 | 0.000799 | 150 | 2 | 0.008265 |
| 2 | yes | 1526 | 3.000 | 0.000001 | 0.000000 | 0.000795 | 0.003358 | 0.002380 | 0.006389 | 0.001181 | 40 | 2 | 0.008354 |
| 3 | yes | 1525 | 3.000 | 0.000001 | 0.000000 | 0.000306 | 0.001239 | 0.002190 | 0.005412 | 0.001405 | 55 | 2 | 0.008233 |
| 4 | yes | 1526 | 3.000 | 0.000001 | 0.000000 | 0.000652 | 0.002647 | 0.002196 | 0.006120 | 0.001225 | 42 | 1 | 0.008513 |
| 5 | yes | 1524 | 3.000 | 0.000006 | 0.000000 | 0.000472 | 0.001903 | 0.002261 | 0.006466 | 0.003126 | 46 | 1 | 0.008380 |
| 6 | yes | 1526 | 5.000 | 0.003004 | 0.000000 | 0.002253 | 0.003004 | 0.003181 | 0.007613 | 0.001264 | 0 | 1 | 0.008382 |
| 7 | yes | 1524 | 3.000 | 0.000005 | 0.000000 | 0.000589 | 0.002380 | 0.002435 | 0.005855 | 0.002527 | 43 | 2 | 0.008332 |
| 8 | yes | 601 | 3.000 | 0.000000 | 0.000000 | 0.001633 | 0.002722 | 0.002432 | 0.006494 | 0.001609 | 6 | 1 | 0.008350 |
| 9 | no | 27753 | 25.000 | 0.409676 | 0.000000 | 0.343776 | 0.409676 | 0.343523 | 0.415125 | 0.411341 | 7 | 1 | 0.011614 |

Interpretation: this is an offline passive-log diagnostic. A hold-like command is a single-point trajectory lasting at least 1 s. The analyzer compares the controller reference and feedback in each command window; it does not publish commands or alter the task safety gates.
