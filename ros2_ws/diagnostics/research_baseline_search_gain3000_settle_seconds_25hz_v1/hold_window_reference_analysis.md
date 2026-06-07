# Hold Window Reference Analysis

- input_dir: `diagnostics/research_baseline_search_gain3000_settle_seconds_25hz_v1`
- command_source: `trajectory_commands.csv`
- tracking_source: `trajectory_controller_state_samples.csv`
- physical_clearance_m: `0.001000`
- state_loop_hz: `25.0`
- hold_like_command_count: `1`
- hold_like_best_feedback_1mm_ticks: `1`

| Cmd | Hold | Samples | Duration s | Target XY m | Target depth m | Ref mean XY m | Ref max XY m | Feedback mean XY m | Feedback max XY m | Feedback final XY m | Ref best 1mm ticks | Feedback best 1mm ticks | P95 JTC error rad |
| ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 0 | no | 10012 | 40.000 | 0.000000 | 0.000000 | 0.229058 | 0.411996 | 0.228441 | 0.415290 | 0.002809 | 4 | 2 | 0.013408 |
| 1 | no | 2858 | 15.000 | 0.000000 | 0.000000 | 0.000398 | 0.002526 | 0.002212 | 0.006901 | 0.001810 | 215 | 2 | 0.008381 |
| 2 | yes | 30 | 2.000 | 0.000000 | 0.000000 | 0.001562 | 0.001609 | 0.001814 | 0.003828 | 0.002227 | 0 | 1 | 0.007132 |
| 3 | no | 7831 | 25.000 | 0.409676 | 0.000000 | 0.187921 | 0.409676 | 0.188055 | 0.414348 | 0.407716 | 219 | 3 | 0.011889 |

Interpretation: this is an offline passive-log diagnostic. A hold-like command is a single-point trajectory lasting at least 1 s. The analyzer compares the controller reference and feedback in each command window; it does not publish commands or alter the task safety gates.
