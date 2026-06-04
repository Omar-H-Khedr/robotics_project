# Hold Window Reference Analysis

- input_dir: `diagnostics/research_baseline_search_velocity_state_v1`
- command_source: `trajectory_commands.csv`
- tracking_source: `trajectory_controller_state_samples.csv`
- physical_clearance_m: `0.001000`
- state_loop_hz: `10.0`
- hold_like_command_count: `7`
- hold_like_best_feedback_1mm_ticks: `2`

| Cmd | Hold | Samples | Duration s | Target XY m | Target depth m | Ref mean XY m | Ref max XY m | Feedback mean XY m | Feedback max XY m | Feedback final XY m | Ref best 1mm ticks | Feedback best 1mm ticks | P95 JTC error rad |
| ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 0 | no | 11400 | 40.000 | 0.000000 | 0.000000 | 0.198970 | 0.410716 | 0.198683 | 0.414732 | 0.002138 | 57 | 2 | 0.013169 |
| 1 | no | 4497 | 15.000 | 0.000000 | 0.000000 | 0.000254 | 0.002378 | 0.002145 | 0.006153 | 0.003157 | 150 | 2 | 0.008279 |
| 2 | yes | 1525 | 5.000 | 0.000001 | 0.000000 | 0.001397 | 0.003528 | 0.002812 | 0.007788 | 0.002562 | 26 | 1 | 0.008341 |
| 3 | yes | 1549 | 5.000 | 0.000005 | 0.000000 | 0.000218 | 0.000540 | 0.002192 | 0.005447 | 0.002745 | 62 | 1 | 0.008366 |
| 4 | yes | 1575 | 5.000 | 0.000007 | 0.000000 | 0.001473 | 0.003702 | 0.002909 | 0.007468 | 0.001078 | 26 | 2 | 0.008408 |
| 5 | yes | 1524 | 5.000 | 0.000004 | 0.000000 | 0.000232 | 0.000564 | 0.001952 | 0.005314 | 0.000183 | 61 | 2 | 0.008384 |
| 6 | yes | 1525 | 5.000 | 0.000005 | 0.000000 | 0.000404 | 0.000981 | 0.002018 | 0.005503 | 0.002246 | 61 | 2 | 0.008246 |
| 7 | yes | 1525 | 5.000 | 0.000003 | 0.000000 | 0.000577 | 0.001408 | 0.002170 | 0.005579 | 0.001938 | 46 | 1 | 0.008386 |
| 8 | yes | 530 | 5.000 | 0.000006 | 0.000000 | 0.001110 | 0.002436 | 0.002398 | 0.005583 | 0.002737 | 7 | 1 | 0.008394 |
| 9 | no | 26529 | 25.000 | 0.409676 | 0.000000 | 0.340754 | 0.409676 | 0.340468 | 0.415501 | 0.407842 | 0 | 1 | 0.011678 |

Interpretation: this is an offline passive-log diagnostic. A hold-like command is a single-point trajectory lasting at least 1 s. The analyzer compares the controller reference and feedback in each command window; it does not publish commands or alter the task safety gates.
