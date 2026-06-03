# Hold Window Reference Analysis

- input_dir: `diagnostics/research_baseline_search_gain3000_v1`
- command_source: `trajectory_commands.csv`
- tracking_source: `trajectory_controller_state_samples.csv`
- physical_clearance_m: `0.001000`
- state_loop_hz: `10.0`
- hold_like_command_count: `7`
- hold_like_best_feedback_1mm_ticks: `3`

| Cmd | Hold | Samples | Duration s | Target XY m | Target depth m | Ref mean XY m | Ref max XY m | Feedback mean XY m | Feedback max XY m | Feedback final XY m | Ref best 1mm ticks | Feedback best 1mm ticks | P95 JTC error rad |
| ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 0 | no | 11140 | 40.000 | 0.000000 | 0.000000 | 0.204477 | 0.411126 | 0.204158 | 0.414278 | 0.002135 | 46 | 1 | 0.013334 |
| 1 | no | 4371 | 15.000 | 0.000000 | 0.000000 | 0.000169 | 0.001446 | 0.002121 | 0.005814 | 0.003289 | 150 | 2 | 0.008261 |
| 2 | yes | 1525 | 5.000 | 0.000001 | 0.000000 | 0.001401 | 0.003192 | 0.002506 | 0.007194 | 0.002659 | 24 | 3 | 0.008210 |
| 3 | yes | 1524 | 5.000 | 0.000000 | 0.000000 | 0.000432 | 0.001053 | 0.002243 | 0.005778 | 0.004012 | 58 | 1 | 0.008184 |
| 4 | yes | 1525 | 5.000 | 0.003001 | 0.000000 | 0.003216 | 0.003535 | 0.003822 | 0.008668 | 0.004911 | 1 | 1 | 0.008449 |
| 5 | yes | 1525 | 5.000 | 0.002997 | 0.000000 | 0.004227 | 0.006458 | 0.004589 | 0.010026 | 0.004518 | 0 | 1 | 0.008444 |
| 6 | yes | 1526 | 5.000 | 0.003000 | 0.000000 | 0.002674 | 0.003000 | 0.002970 | 0.006816 | 0.004781 | 0 | 1 | 0.008211 |
| 7 | yes | 1524 | 5.000 | 0.003000 | 0.000000 | 0.002844 | 0.003446 | 0.003083 | 0.006896 | 0.004472 | 0 | 1 | 0.008386 |
| 8 | yes | 580 | 5.000 | 0.000004 | 0.000000 | 0.004587 | 0.005967 | 0.004653 | 0.009684 | 0.000884 | 0 | 0 | 0.008314 |
| 9 | no | 26532 | 25.000 | 0.409676 | 0.000000 | 0.341007 | 0.409676 | 0.340618 | 0.415061 | 0.408572 | 0 | 1 | 0.011611 |

Interpretation: this is an offline passive-log diagnostic. A hold-like command is a single-point trajectory lasting at least 1 s. The analyzer compares the controller reference and feedback in each command window; it does not publish commands or alter the task safety gates.
