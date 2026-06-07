# Hold Window Reference Analysis

- input_dir: `diagnostics/research_baseline_insert_predepth_recenter_500hz_v1`
- command_source: `trajectory_commands.csv`
- tracking_source: `trajectory_controller_state_samples.csv`
- physical_clearance_m: `0.001000`
- state_loop_hz: `25.0`
- hold_like_command_count: `4`
- hold_like_best_feedback_1mm_ticks: `46`

| Cmd | Hold | Samples | Duration s | Target XY m | Target depth m | Ref mean XY m | Ref max XY m | Feedback mean XY m | Feedback max XY m | Feedback final XY m | Ref best 1mm ticks | Feedback best 1mm ticks | P95 JTC error rad |
| ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 0 | no | 20009 | 40.000 | 0.000000 | 0.000000 | 0.228406 | 0.410153 | 0.228244 | 0.411426 | 0.000580 | 4 | 3 | 0.006323 |
| 1 | no | 5848 | 15.000 | 0.000000 | 0.000000 | 0.000101 | 0.000664 | 0.000487 | 0.001642 | 0.000273 | 293 | 81 | 0.003708 |
| 2 | yes | 1221 | 2.000 | 0.000000 | 0.000000 | 0.000107 | 0.000415 | 0.000513 | 0.001580 | 0.000593 | 62 | 43 | 0.003692 |
| 3 | yes | 1319 | 20.000 | 0.000000 | 0.020000 | 0.000710 | 0.000778 | 0.000799 | 0.002130 | 0.000287 | 66 | 8 | 0.003707 |
| 4 | yes | 1138 | 2.000 | 0.000000 | 0.000000 | 0.000247 | 0.000679 | 0.000545 | 0.001710 | 0.000167 | 57 | 34 | 0.003762 |
| 5 | yes | 11508 | 20.000 | 0.000000 | 0.019993 | 0.000353 | 0.000564 | 0.000565 | 0.001853 | 0.000517 | 577 | 46 | 0.003784 |
| 6 | no | 12551 | 25.000 | 0.409676 | 0.000000 | 0.101226 | 0.409676 | 0.101295 | 0.410364 | 0.409863 | 345 | 21 | 0.005332 |

Interpretation: this is an offline passive-log diagnostic. A hold-like command is a single-point trajectory lasting at least 1 s. The analyzer compares the controller reference and feedback in each command window; it does not publish commands or alter the task safety gates.
