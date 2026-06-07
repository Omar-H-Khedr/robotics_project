# Hold Window Reference Analysis

- input_dir: `diagnostics/research_baseline_search_recenter8_settle9_gain3000_damping10_25hz_v1`
- command_source: `trajectory_commands.csv`
- tracking_source: `trajectory_controller_state_samples.csv`
- physical_clearance_m: `0.001000`
- state_loop_hz: `25.0`
- hold_like_command_count: `1`
- hold_like_best_feedback_1mm_ticks: `4`

| Cmd | Hold | Samples | Duration s | Target XY m | Target depth m | Ref mean XY m | Ref max XY m | Feedback mean XY m | Feedback max XY m | Feedback final XY m | Ref best 1mm ticks | Feedback best 1mm ticks | P95 JTC error rad |
| ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 0 | no | 10025 | 40.000 | 0.000000 | 0.000000 | 0.228722 | 0.410506 | 0.228330 | 0.412964 | 0.001749 | 4 | 2 | 0.012922 |
| 1 | no | 3607 | 15.000 | 0.000000 | 0.000000 | 0.000197 | 0.001417 | 0.001239 | 0.003644 | 0.001537 | 300 | 7 | 0.007556 |
| 2 | yes | 3024 | 2.000 | 0.000001 | 0.000000 | 0.000218 | 0.002768 | 0.001276 | 0.004509 | 0.000494 | 272 | 4 | 0.007669 |
| 3 | no | 10752 | 25.000 | 0.409676 | 0.000000 | 0.243575 | 0.409676 | 0.243487 | 0.412989 | 0.408838 | 1 | 3 | 0.011225 |

Interpretation: this is an offline passive-log diagnostic. A hold-like command is a single-point trajectory lasting at least 1 s. The analyzer compares the controller reference and feedback in each command window; it does not publish commands or alter the task safety gates.
