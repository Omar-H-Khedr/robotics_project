# Hold Window Reference Analysis

- input_dir: `diagnostics/research_baseline_insert_handoff_gate_order_v1`
- command_source: `trajectory_commands.csv`
- tracking_source: `trajectory_controller_state_samples.csv`
- physical_clearance_m: `0.001000`
- state_loop_hz: `25.0`
- hold_like_command_count: `5`
- hold_like_best_feedback_1mm_ticks: `4`

| Cmd | Hold | Samples | Duration s | Target XY m | Target depth m | Ref mean XY m | Ref max XY m | Feedback mean XY m | Feedback max XY m | Feedback final XY m | Ref best 1mm ticks | Feedback best 1mm ticks | P95 JTC error rad |
| ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 0 | no | 10169 | 40.000 | 0.000000 | 0.000000 | 0.224551 | 0.412837 | 0.223979 | 0.416875 | 0.001142 | 18 | 4 | 0.013390 |
| 1 | no | 4417 | 15.000 | 0.000000 | 0.000000 | 0.000093 | 0.000864 | 0.002135 | 0.005799 | 0.001663 | 442 | 6 | 0.008339 |
| 2 | yes | 1510 | 5.000 | 0.000001 | 0.000000 | 0.000656 | 0.001309 | 0.002093 | 0.005316 | 0.003393 | 102 | 2 | 0.008519 |
| 3 | yes | 1509 | 5.000 | 0.000000 | 0.000000 | 0.001040 | 0.002514 | 0.002560 | 0.006756 | 0.001063 | 75 | 3 | 0.008440 |
| 4 | yes | 1510 | 5.000 | 0.000000 | 0.000000 | 0.000587 | 0.001417 | 0.002098 | 0.006042 | 0.000747 | 114 | 4 | 0.008394 |
| 5 | yes | 1520 | 5.000 | 0.000006 | 0.000000 | 0.000392 | 0.000948 | 0.002278 | 0.006546 | 0.001994 | 152 | 1 | 0.008555 |
| 6 | yes | 949 | 5.000 | 0.000007 | 0.000000 | 0.001412 | 0.002270 | 0.002571 | 0.005987 | 0.002341 | 24 | 1 | 0.008577 |

Interpretation: this is an offline passive-log diagnostic. A hold-like command is a single-point trajectory lasting at least 1 s. The analyzer compares the controller reference and feedback in each command window; it does not publish commands or alter the task safety gates.
