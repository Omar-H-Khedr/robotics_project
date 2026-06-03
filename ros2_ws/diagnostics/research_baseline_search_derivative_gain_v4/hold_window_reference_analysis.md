# Hold Window Reference Analysis

- input_dir: `diagnostics/research_baseline_search_derivative_gain_v4`
- command_source: `trajectory_commands.csv`
- tracking_source: `trajectory_controller_state_samples.csv`
- physical_clearance_m: `0.001000`
- state_loop_hz: `10.0`
- hold_like_command_count: `7`
- hold_like_best_feedback_1mm_ticks: `2`

| Cmd | Hold | Samples | Duration s | Target XY m | Target depth m | Ref mean XY m | Ref max XY m | Feedback mean XY m | Feedback max XY m | Feedback final XY m | Ref best 1mm ticks | Feedback best 1mm ticks | P95 JTC error rad |
| ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 0 | no | 10180 | 40.000 | 0.000000 | 0.000000 | 0.224053 | 0.410114 | 0.223490 | 0.413465 | 0.002247 | 8 | 0 | 0.013526 |
| 1 | no | 4446 | 15.000 | 0.000000 | 0.000000 | 0.000245 | 0.002535 | 0.002207 | 0.006700 | 0.002657 | 154 | 2 | 0.008370 |
| 2 | yes | 1526 | 5.000 | 0.000001 | 0.000000 | 0.001342 | 0.003397 | 0.002686 | 0.006891 | 0.000909 | 26 | 2 | 0.008256 |
| 3 | yes | 1524 | 5.000 | 0.000009 | 0.000000 | 0.000372 | 0.000907 | 0.002223 | 0.005533 | 0.000529 | 61 | 2 | 0.008324 |
| 4 | yes | 1526 | 5.000 | 0.000006 | 0.000000 | 0.000370 | 0.000894 | 0.002317 | 0.006377 | 0.001749 | 62 | 2 | 0.008472 |
| 5 | yes | 1524 | 5.000 | 0.000004 | 0.000000 | 0.001471 | 0.003582 | 0.002617 | 0.008397 | 0.003613 | 24 | 1 | 0.008375 |
| 6 | yes | 1550 | 5.000 | 0.000002 | 0.000000 | 0.001246 | 0.003082 | 0.002709 | 0.007564 | 0.001822 | 28 | 1 | 0.008316 |
| 7 | yes | 1550 | 5.000 | 0.000002 | 0.000000 | 0.000595 | 0.001474 | 0.002333 | 0.006050 | 0.002473 | 45 | 2 | 0.008438 |
| 8 | yes | 555 | 5.000 | 0.000003 | 0.000000 | 0.003038 | 0.003904 | 0.003818 | 0.007237 | 0.000706 | 0 | 1 | 0.008472 |
| 9 | no | 25605 | 25.000 | 0.409676 | 0.000000 | 0.338239 | 0.409676 | 0.337944 | 0.415323 | 0.409680 | 0 | 1 | 0.011697 |

Interpretation: this is an offline passive-log diagnostic. A hold-like command is a single-point trajectory lasting at least 1 s. The analyzer compares the controller reference and feedback in each command window; it does not publish commands or alter the task safety gates.
