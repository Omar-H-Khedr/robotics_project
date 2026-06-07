# Hold Window Reference Analysis

- input_dir: `diagnostics/research_baseline_search_gate_trace_recenter8_gain3000_damping10_25hz_v1`
- command_source: `trajectory_commands.csv`
- tracking_source: `trajectory_controller_state_samples.csv`
- physical_clearance_m: `0.001000`
- state_loop_hz: `25.0`
- hold_like_command_count: `1`
- hold_like_best_feedback_1mm_ticks: `5`

| Cmd | Hold | Samples | Duration s | Target XY m | Target depth m | Ref mean XY m | Ref max XY m | Feedback mean XY m | Feedback max XY m | Feedback final XY m | Ref best 1mm ticks | Feedback best 1mm ticks | P95 JTC error rad |
| ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 0 | no | 10057 | 40.000 | 0.000000 | 0.000000 | 0.226487 | 0.411142 | 0.226112 | 0.413361 | 0.000788 | 7 | 1 | 0.013043 |
| 1 | no | 2917 | 15.000 | 0.000000 | 0.000000 | 0.000124 | 0.001006 | 0.001137 | 0.003237 | 0.001639 | 291 | 5 | 0.007648 |
| 2 | yes | 3013 | 2.000 | 0.000000 | 0.000000 | 0.000032 | 0.000258 | 0.001174 | 0.003678 | 0.001625 | 302 | 5 | 0.007544 |
| 3 | no | 6264 | 25.000 | 0.409676 | 0.000000 | 0.141988 | 0.409676 | 0.141810 | 0.411987 | 0.408636 | 0 | 2 | 0.012068 |

Interpretation: this is an offline passive-log diagnostic. A hold-like command is a single-point trajectory lasting at least 1 s. The analyzer compares the controller reference and feedback in each command window; it does not publish commands or alter the task safety gates.
