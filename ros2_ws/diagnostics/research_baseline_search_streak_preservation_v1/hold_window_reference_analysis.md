# Hold Window Reference Analysis

- input_dir: `diagnostics/research_baseline_search_streak_preservation_v1`
- command_source: `trajectory_commands.csv`
- tracking_source: `trajectory_controller_state_samples.csv`
- physical_clearance_m: `0.001000`
- state_loop_hz: `10.0`
- hold_like_command_count: `7`
- hold_like_best_feedback_1mm_ticks: `3`

| Cmd | Hold | Samples | Duration s | Target XY m | Target depth m | Ref mean XY m | Ref max XY m | Feedback mean XY m | Feedback max XY m | Feedback final XY m | Ref best 1mm ticks | Feedback best 1mm ticks | P95 JTC error rad |
| ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 0 | no | 10619 | 40.000 | 0.000000 | 0.000000 | 0.214838 | 0.409912 | 0.214382 | 0.414675 | 0.000955 | 25 | 1 | 0.013057 |
| 1 | no | 4575 | 15.000 | 0.000000 | 0.000000 | 0.000114 | 0.001342 | 0.002112 | 0.006541 | 0.001138 | 180 | 2 | 0.008386 |
| 2 | yes | 1524 | 3.000 | 0.000001 | 0.000000 | 0.000044 | 0.000148 | 0.002224 | 0.005810 | 0.003657 | 61 | 2 | 0.008310 |
| 3 | yes | 1526 | 3.000 | 0.000000 | 0.000000 | 0.000534 | 0.002180 | 0.002221 | 0.007347 | 0.003496 | 44 | 1 | 0.008388 |
| 4 | yes | 1525 | 3.000 | 0.000002 | 0.000000 | 0.000934 | 0.003789 | 0.002254 | 0.006578 | 0.002575 | 38 | 2 | 0.008341 |
| 5 | yes | 1525 | 3.000 | 0.000000 | 0.000000 | 0.000708 | 0.002876 | 0.002498 | 0.006573 | 0.002149 | 41 | 2 | 0.008391 |
| 6 | yes | 1551 | 3.000 | 0.000003 | 0.000000 | 0.000523 | 0.002156 | 0.002245 | 0.006633 | 0.001764 | 46 | 2 | 0.008404 |
| 7 | yes | 1524 | 3.000 | 0.000000 | 0.000000 | 0.000682 | 0.002767 | 0.002363 | 0.007118 | 0.001968 | 41 | 3 | 0.008347 |
| 8 | yes | 579 | 3.000 | 0.000002 | 0.000000 | 0.001162 | 0.001889 | 0.002571 | 0.005658 | 0.000904 | 9 | 1 | 0.008169 |
| 9 | no | 24723 | 25.000 | 0.409676 | 0.000000 | 0.337424 | 0.409676 | 0.337152 | 0.415040 | 0.409684 | 1 | 2 | 0.011657 |

Interpretation: this is an offline passive-log diagnostic. A hold-like command is a single-point trajectory lasting at least 1 s. The analyzer compares the controller reference and feedback in each command window; it does not publish commands or alter the task safety gates.
