# Hold Window Reference Analysis

- input_dir: `diagnostics/research_baseline_search_derivative_gain_v2`
- command_source: `trajectory_commands.csv`
- tracking_source: `trajectory_controller_state_samples.csv`
- physical_clearance_m: `0.001000`
- state_loop_hz: `10.0`
- hold_like_command_count: `7`
- hold_like_best_feedback_1mm_ticks: `2`

| Cmd | Hold | Samples | Duration s | Target XY m | Target depth m | Ref mean XY m | Ref max XY m | Feedback mean XY m | Feedback max XY m | Feedback final XY m | Ref best 1mm ticks | Feedback best 1mm ticks | P95 JTC error rad |
| ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 0 | no | 11937 | 40.000 | 0.000000 | 0.000000 | 0.191073 | 0.408480 | 0.190879 | 0.412501 | 0.003013 | 78 | 3 | 0.013025 |
| 1 | no | 4445 | 15.000 | 0.000000 | 0.000000 | 0.000223 | 0.001566 | 0.002147 | 0.006853 | 0.003254 | 152 | 2 | 0.008421 |
| 2 | yes | 1525 | 5.000 | 0.000001 | 0.000000 | 0.001008 | 0.002782 | 0.002742 | 0.007183 | 0.003425 | 33 | 1 | 0.008430 |
| 3 | yes | 1525 | 5.000 | 0.000003 | 0.000000 | 0.000741 | 0.001806 | 0.002077 | 0.005898 | 0.001274 | 38 | 1 | 0.008311 |
| 4 | yes | 1526 | 5.000 | 0.000002 | 0.000000 | 0.000698 | 0.001703 | 0.002407 | 0.007050 | 0.001000 | 41 | 1 | 0.008393 |
| 5 | yes | 1524 | 5.000 | 0.000000 | 0.000000 | 0.000870 | 0.002122 | 0.002432 | 0.006856 | 0.001540 | 34 | 2 | 0.008376 |
| 6 | yes | 1525 | 5.000 | 0.000009 | 0.000000 | 0.000589 | 0.001432 | 0.002110 | 0.005983 | 0.001256 | 45 | 2 | 0.008434 |
| 7 | yes | 1525 | 5.000 | 0.000009 | 0.000000 | 0.000492 | 0.001200 | 0.002294 | 0.005706 | 0.002779 | 52 | 1 | 0.008345 |
| 8 | yes | 603 | 5.000 | 0.000002 | 0.000000 | 0.001510 | 0.001994 | 0.002415 | 0.005984 | 0.003137 | 1 | 2 | 0.008206 |
| 9 | no | 23752 | 25.000 | 0.409676 | 0.000000 | 0.332838 | 0.409676 | 0.332560 | 0.414978 | 0.407355 | 8 | 2 | 0.011678 |

Interpretation: this is an offline passive-log diagnostic. A hold-like command is a single-point trajectory lasting at least 1 s. The analyzer compares the controller reference and feedback in each command window; it does not publish commands or alter the task safety gates.
