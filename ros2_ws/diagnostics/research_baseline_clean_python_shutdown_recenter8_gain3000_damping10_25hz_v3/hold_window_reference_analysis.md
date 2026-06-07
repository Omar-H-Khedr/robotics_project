# Hold Window Reference Analysis

- input_dir: `diagnostics/research_baseline_clean_python_shutdown_recenter8_gain3000_damping10_25hz_v3`
- command_source: `trajectory_commands.csv`
- tracking_source: `trajectory_controller_state_samples.csv`
- physical_clearance_m: `0.001000`
- state_loop_hz: `25.0`
- hold_like_command_count: `4`
- hold_like_best_feedback_1mm_ticks: `6`

| Cmd | Hold | Samples | Duration s | Target XY m | Target depth m | Ref mean XY m | Ref max XY m | Feedback mean XY m | Feedback max XY m | Feedback final XY m | Ref best 1mm ticks | Feedback best 1mm ticks | P95 JTC error rad |
| ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 0 | no | 10015 | 40.000 | 0.000000 | 0.000000 | 0.228057 | 0.408707 | 0.227683 | 0.411043 | 0.000274 | 3 | 2 | 0.013156 |
| 1 | no | 5118 | 15.000 | 0.000000 | 0.000000 | 0.000114 | 0.001506 | 0.001195 | 0.004592 | 0.000471 | 499 | 8 | 0.007561 |
| 2 | yes | 2299 | 8.000 | 0.000001 | 0.000000 | 0.000259 | 0.000807 | 0.001261 | 0.003398 | 0.002012 | 230 | 5 | 0.007565 |
| 3 | yes | 2260 | 8.000 | 0.000002 | 0.000000 | 0.000307 | 0.000692 | 0.001248 | 0.003857 | 0.000448 | 226 | 5 | 0.007629 |
| 4 | yes | 2269 | 8.000 | 0.000001 | 0.000000 | 0.000511 | 0.001159 | 0.001380 | 0.003987 | 0.001505 | 199 | 6 | 0.007589 |
| 5 | yes | 2154 | 8.000 | 0.000001 | 0.000000 | 0.000523 | 0.001125 | 0.001366 | 0.003938 | 0.000851 | 193 | 4 | 0.007620 |
| 6 | no | 6286 | 25.000 | 0.409676 | 0.000000 | 0.125647 | 0.409676 | 0.125675 | 0.411116 | 0.408293 | 274 | 5 | 0.011190 |

Interpretation: this is an offline passive-log diagnostic. A hold-like command is a single-point trajectory lasting at least 1 s. The analyzer compares the controller reference and feedback in each command window; it does not publish commands or alter the task safety gates.
