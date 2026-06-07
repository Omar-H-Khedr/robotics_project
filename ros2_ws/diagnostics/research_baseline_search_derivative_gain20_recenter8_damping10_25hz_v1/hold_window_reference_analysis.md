# Hold Window Reference Analysis

- input_dir: `diagnostics/research_baseline_search_derivative_gain20_recenter8_damping10_25hz_v1`
- command_source: `trajectory_commands.csv`
- tracking_source: `trajectory_controller_state_samples.csv`
- physical_clearance_m: `0.001000`
- state_loop_hz: `25.0`
- hold_like_command_count: `4`
- hold_like_best_feedback_1mm_ticks: `9`

| Cmd | Hold | Samples | Duration s | Target XY m | Target depth m | Ref mean XY m | Ref max XY m | Feedback mean XY m | Feedback max XY m | Feedback final XY m | Ref best 1mm ticks | Feedback best 1mm ticks | P95 JTC error rad |
| ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 0 | no | 10038 | 40.000 | 0.000000 | 0.000000 | 0.227216 | 0.408866 | 0.226822 | 0.412106 | 0.000963 | 5 | 1 | 0.013056 |
| 1 | no | 5167 | 15.000 | 0.000000 | 0.000000 | 0.000106 | 0.001369 | 0.001219 | 0.003731 | 0.000522 | 506 | 7 | 0.007564 |
| 2 | yes | 2259 | 8.000 | 0.000001 | 0.000000 | 0.000570 | 0.001381 | 0.001391 | 0.004056 | 0.000348 | 179 | 5 | 0.007615 |
| 3 | yes | 2260 | 8.000 | 0.000002 | 0.000000 | 0.000161 | 0.000363 | 0.001187 | 0.003271 | 0.000488 | 226 | 9 | 0.007551 |
| 4 | yes | 2300 | 8.000 | 0.000002 | 0.000000 | 0.000206 | 0.000473 | 0.001238 | 0.003810 | 0.001854 | 230 | 8 | 0.007646 |
| 5 | yes | 2174 | 8.000 | 0.000003 | 0.000000 | 0.000560 | 0.001219 | 0.001244 | 0.003848 | 0.000586 | 182 | 8 | 0.007632 |
| 6 | no | 6273 | 25.000 | 0.409676 | 0.000000 | 0.125249 | 0.409676 | 0.125139 | 0.411598 | 0.409802 | 23 | 2 | 0.011833 |

Interpretation: this is an offline passive-log diagnostic. A hold-like command is a single-point trajectory lasting at least 1 s. The analyzer compares the controller reference and feedback in each command window; it does not publish commands or alter the task safety gates.
