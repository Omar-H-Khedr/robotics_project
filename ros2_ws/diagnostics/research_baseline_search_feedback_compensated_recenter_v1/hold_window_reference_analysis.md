# Hold Window Reference Analysis

- input_dir: `diagnostics/research_baseline_search_feedback_compensated_recenter_v1`
- command_source: `trajectory_commands.csv`
- tracking_source: `trajectory_controller_state_samples.csv`
- physical_clearance_m: `0.001000`
- state_loop_hz: `10.0`
- hold_like_command_count: `7`
- hold_like_best_feedback_1mm_ticks: `4`

| Cmd | Hold | Samples | Duration s | Target XY m | Target depth m | Ref mean XY m | Ref max XY m | Feedback mean XY m | Feedback max XY m | Feedback final XY m | Ref best 1mm ticks | Feedback best 1mm ticks | P95 JTC error rad |
| ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 0 | no | 10468 | 40.000 | 0.000000 | 0.000000 | 0.218378 | 0.409659 | 0.217886 | 0.414152 | 0.000258 | 19 | 2 | 0.013142 |
| 1 | no | 4329 | 15.000 | 0.000000 | 0.000000 | 0.000183 | 0.001613 | 0.002104 | 0.005886 | 0.000865 | 148 | 3 | 0.008434 |
| 2 | yes | 1527 | 3.000 | 0.000864 | 0.000000 | 0.001011 | 0.002202 | 0.001878 | 0.005914 | 0.002202 | 45 | 2 | 0.008287 |
| 3 | yes | 1524 | 3.000 | 0.001525 | 0.000000 | 0.001483 | 0.002199 | 0.002280 | 0.006466 | 0.002135 | 0 | 1 | 0.008287 |
| 4 | yes | 1524 | 3.000 | 0.002000 | 0.000000 | 0.001541 | 0.002088 | 0.002477 | 0.007239 | 0.001424 | 14 | 2 | 0.008397 |
| 5 | yes | 1525 | 3.000 | 0.001957 | 0.000000 | 0.001406 | 0.001957 | 0.002713 | 0.006797 | 0.003001 | 20 | 4 | 0.008365 |
| 6 | yes | 1524 | 3.000 | 0.002000 | 0.000000 | 0.001426 | 0.002000 | 0.002389 | 0.006151 | 0.004757 | 19 | 2 | 0.008316 |
| 7 | yes | 1527 | 5.000 | 0.003000 | 0.000000 | 0.002008 | 0.003147 | 0.002759 | 0.007502 | 0.003807 | 8 | 1 | 0.008440 |
| 8 | yes | 601 | 5.000 | 0.003001 | 0.000000 | 0.003460 | 0.003948 | 0.004075 | 0.008310 | 0.004491 | 0 | 0 | 0.008394 |
| 9 | no | 12762 | 25.000 | 0.409676 | 0.000000 | 0.266773 | 0.409676 | 0.266503 | 0.414222 | 0.410737 | 0 | 1 | 0.011736 |

Interpretation: this is an offline passive-log diagnostic. A hold-like command is a single-point trajectory lasting at least 1 s. The analyzer compares the controller reference and feedback in each command window; it does not publish commands or alter the task safety gates.
