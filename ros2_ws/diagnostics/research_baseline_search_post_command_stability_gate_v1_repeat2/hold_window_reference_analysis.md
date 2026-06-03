# Hold Window Reference Analysis

- input_dir: `diagnostics/research_baseline_search_post_command_stability_gate_v1_repeat2`
- command_source: `trajectory_commands.csv`
- tracking_source: `trajectory_controller_state_samples.csv`
- physical_clearance_m: `0.001000`
- state_loop_hz: `10.0`
- hold_like_command_count: `7`
- hold_like_best_feedback_1mm_ticks: `2`

| Cmd | Hold | Samples | Duration s | Target XY m | Target depth m | Ref mean XY m | Ref max XY m | Feedback mean XY m | Feedback max XY m | Feedback final XY m | Ref best 1mm ticks | Feedback best 1mm ticks | P95 JTC error rad |
| ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 0 | no | 12673 | 40.000 | 0.000000 | 0.000000 | 0.180514 | 0.410266 | 0.180451 | 0.414065 | 0.004076 | 107 | 2 | 0.012759 |
| 1 | no | 4476 | 15.000 | 0.000000 | 0.000000 | 0.000575 | 0.004073 | 0.002335 | 0.007753 | 0.002599 | 146 | 2 | 0.008352 |
| 2 | yes | 1524 | 3.000 | 0.000001 | 0.000000 | 0.000559 | 0.002562 | 0.002192 | 0.006285 | 0.002802 | 45 | 1 | 0.008377 |
| 3 | yes | 1525 | 5.000 | 0.003001 | 0.000000 | 0.002770 | 0.003001 | 0.003303 | 0.007882 | 0.002959 | 0 | 1 | 0.008481 |
| 4 | yes | 1525 | 3.000 | 0.000009 | 0.000000 | 0.000449 | 0.001804 | 0.002215 | 0.005895 | 0.002256 | 47 | 2 | 0.008442 |
| 5 | yes | 1525 | 3.000 | 0.000006 | 0.000000 | 0.000389 | 0.001565 | 0.002253 | 0.006259 | 0.001728 | 50 | 1 | 0.008333 |
| 6 | yes | 1525 | 3.000 | 0.000003 | 0.000000 | 0.000502 | 0.002030 | 0.002289 | 0.006330 | 0.001975 | 45 | 2 | 0.008457 |
| 7 | yes | 1525 | 3.000 | 0.000000 | 0.000000 | 0.000687 | 0.002790 | 0.002304 | 0.007203 | 0.001676 | 41 | 2 | 0.008262 |
| 8 | yes | 604 | 3.000 | 0.000001 | 0.000000 | 0.001575 | 0.002634 | 0.002466 | 0.006317 | 0.003567 | 6 | 2 | 0.008488 |
| 9 | no | 27202 | 25.000 | 0.409676 | 0.000000 | 0.342275 | 0.409676 | 0.342059 | 0.415433 | 0.410019 | 105 | 2 | 0.011699 |

Interpretation: this is an offline passive-log diagnostic. A hold-like command is a single-point trajectory lasting at least 1 s. The analyzer compares the controller reference and feedback in each command window; it does not publish commands or alter the task safety gates.
