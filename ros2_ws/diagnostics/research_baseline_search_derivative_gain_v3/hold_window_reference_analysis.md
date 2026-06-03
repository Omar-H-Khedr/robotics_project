# Hold Window Reference Analysis

- input_dir: `diagnostics/research_baseline_search_derivative_gain_v3`
- command_source: `trajectory_commands.csv`
- tracking_source: `trajectory_controller_state_samples.csv`
- physical_clearance_m: `0.001000`
- state_loop_hz: `10.0`
- hold_like_command_count: `7`
- hold_like_best_feedback_1mm_ticks: `3`

| Cmd | Hold | Samples | Duration s | Target XY m | Target depth m | Ref mean XY m | Ref max XY m | Feedback mean XY m | Feedback max XY m | Feedback final XY m | Ref best 1mm ticks | Feedback best 1mm ticks | P95 JTC error rad |
| ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 0 | no | 10100 | 40.000 | 0.000000 | 0.000000 | 0.227789 | 0.412427 | 0.227235 | 0.415312 | 0.001973 | 4 | 2 | 0.013323 |
| 1 | no | 4525 | 15.000 | 0.000000 | 0.000000 | 0.000478 | 0.003474 | 0.002310 | 0.008036 | 0.002517 | 149 | 3 | 0.008417 |
| 2 | yes | 1524 | 5.000 | 0.000001 | 0.000000 | 0.001097 | 0.002515 | 0.002387 | 0.006473 | 0.002046 | 28 | 2 | 0.008259 |
| 3 | yes | 1526 | 5.000 | 0.000000 | 0.000000 | 0.001188 | 0.002898 | 0.002450 | 0.007452 | 0.002663 | 29 | 2 | 0.008306 |
| 4 | yes | 1524 | 5.000 | 0.000000 | 0.000000 | 0.001631 | 0.003973 | 0.002708 | 0.007754 | 0.002736 | 23 | 3 | 0.008299 |
| 5 | yes | 1550 | 5.000 | 0.000000 | 0.000000 | 0.001249 | 0.003097 | 0.002512 | 0.006840 | 0.003129 | 28 | 2 | 0.008481 |
| 6 | yes | 1526 | 5.000 | 0.000009 | 0.000000 | 0.000464 | 0.001125 | 0.002220 | 0.006399 | 0.001707 | 56 | 2 | 0.008282 |
| 7 | yes | 1524 | 5.000 | 0.000004 | 0.000000 | 0.000814 | 0.001982 | 0.002481 | 0.006472 | 0.001535 | 36 | 1 | 0.008237 |
| 8 | yes | 581 | 5.000 | 0.000007 | 0.000000 | 0.000711 | 0.000924 | 0.002221 | 0.005466 | 0.003057 | 24 | 2 | 0.008382 |
| 9 | no | 25758 | 25.000 | 0.409676 | 0.000000 | 0.338921 | 0.409676 | 0.338583 | 0.414704 | 0.406942 | 0 | 1 | 0.011757 |

Interpretation: this is an offline passive-log diagnostic. A hold-like command is a single-point trajectory lasting at least 1 s. The analyzer compares the controller reference and feedback in each command window; it does not publish commands or alter the task safety gates.
