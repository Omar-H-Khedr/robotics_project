# Hold Window Reference Analysis

- input_dir: `diagnostics/research_baseline_search_derivative_gain_v1`
- command_source: `trajectory_commands.csv`
- tracking_source: `trajectory_controller_state_samples.csv`
- physical_clearance_m: `0.001000`
- state_loop_hz: `10.0`
- hold_like_command_count: `7`
- hold_like_best_feedback_1mm_ticks: `3`

| Cmd | Hold | Samples | Duration s | Target XY m | Target depth m | Ref mean XY m | Ref max XY m | Feedback mean XY m | Feedback max XY m | Feedback final XY m | Ref best 1mm ticks | Feedback best 1mm ticks | P95 JTC error rad |
| ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 0 | no | 11057 | 40.000 | 0.000000 | 0.000000 | 0.207524 | 0.411415 | 0.207158 | 0.415565 | 0.001005 | 43 | 2 | 0.013200 |
| 1 | no | 4470 | 15.000 | 0.000000 | 0.000000 | 0.000271 | 0.002624 | 0.002159 | 0.007075 | 0.001887 | 148 | 3 | 0.008337 |
| 2 | yes | 1524 | 5.000 | 0.000001 | 0.000000 | 0.001168 | 0.002990 | 0.002387 | 0.006647 | 0.000434 | 29 | 3 | 0.008330 |
| 3 | yes | 1525 | 5.000 | 0.000002 | 0.000000 | 0.001247 | 0.003039 | 0.002460 | 0.006872 | 0.001782 | 27 | 1 | 0.008367 |
| 4 | yes | 1526 | 5.000 | 0.000002 | 0.000000 | 0.001126 | 0.002747 | 0.002311 | 0.006758 | 0.003607 | 30 | 2 | 0.008220 |
| 5 | yes | 1524 | 5.000 | 0.000000 | 0.000000 | 0.001164 | 0.002836 | 0.002638 | 0.007156 | 0.002122 | 28 | 1 | 0.008453 |
| 6 | yes | 1525 | 5.000 | 0.000000 | 0.000000 | 0.000680 | 0.001659 | 0.002396 | 0.006428 | 0.002277 | 41 | 1 | 0.008392 |
| 7 | yes | 1525 | 5.000 | 0.000005 | 0.000000 | 0.000666 | 0.001622 | 0.002196 | 0.005631 | 0.002036 | 41 | 2 | 0.008423 |
| 8 | yes | 605 | 5.000 | 0.000004 | 0.000000 | 0.001788 | 0.002357 | 0.002692 | 0.006385 | 0.002532 | 0 | 0 | 0.008617 |
| 9 | no | 23842 | 25.000 | 0.409676 | 0.000000 | 0.331323 | 0.409676 | 0.331064 | 0.415047 | 0.408603 | 0 | 1 | 0.011644 |

Interpretation: this is an offline passive-log diagnostic. A hold-like command is a single-point trajectory lasting at least 1 s. The analyzer compares the controller reference and feedback in each command window; it does not publish commands or alter the task safety gates.
