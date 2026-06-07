# Hold Window Reference Analysis

- input_dir: `diagnostics/research_baseline_search_damping10_gain3000_25hz_v1`
- command_source: `trajectory_commands.csv`
- tracking_source: `trajectory_controller_state_samples.csv`
- physical_clearance_m: `0.001000`
- state_loop_hz: `25.0`
- hold_like_command_count: `1`
- hold_like_best_feedback_1mm_ticks: `4`

| Cmd | Hold | Samples | Duration s | Target XY m | Target depth m | Ref mean XY m | Ref max XY m | Feedback mean XY m | Feedback max XY m | Feedback final XY m | Ref best 1mm ticks | Feedback best 1mm ticks | P95 JTC error rad |
| ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 0 | no | 9950 | 40.000 | 0.000000 | 0.000000 | 0.229557 | 0.409521 | 0.229172 | 0.411626 | 0.000562 | 2 | 0 | 0.012933 |
| 1 | no | 2932 | 15.000 | 0.000000 | 0.000000 | 0.000259 | 0.001077 | 0.001283 | 0.004059 | 0.000417 | 245 | 6 | 0.007659 |
| 2 | yes | 1511 | 2.000 | 0.000000 | 0.000000 | 0.000116 | 0.000887 | 0.001176 | 0.003123 | 0.000416 | 152 | 4 | 0.007546 |
| 3 | no | 5691 | 25.000 | 0.409676 | 0.000000 | 0.106817 | 0.359304 | 0.106945 | 0.361192 | 0.356302 | 251 | 3 | 0.011924 |

Interpretation: this is an offline passive-log diagnostic. A hold-like command is a single-point trajectory lasting at least 1 s. The analyzer compares the controller reference and feedback in each command window; it does not publish commands or alter the task safety gates.
