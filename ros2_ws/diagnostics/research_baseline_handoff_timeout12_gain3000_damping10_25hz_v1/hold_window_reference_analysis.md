# Hold Window Reference Analysis

- input_dir: `diagnostics/research_baseline_handoff_timeout12_gain3000_damping10_25hz_v1`
- command_source: `trajectory_commands.csv`
- tracking_source: `trajectory_controller_state_samples.csv`
- physical_clearance_m: `0.001000`
- state_loop_hz: `25.0`
- hold_like_command_count: `7`
- hold_like_best_feedback_1mm_ticks: `6`

| Cmd | Hold | Samples | Duration s | Target XY m | Target depth m | Ref mean XY m | Ref max XY m | Feedback mean XY m | Feedback max XY m | Feedback final XY m | Ref best 1mm ticks | Feedback best 1mm ticks | P95 JTC error rad |
| ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 0 | no | 10049 | 40.000 | 0.000000 | 0.000000 | 0.226955 | 0.410459 | 0.226570 | 0.412484 | 0.000851 | 6 | 1 | 0.012879 |
| 1 | no | 4428 | 15.000 | 0.000000 | 0.000000 | 0.000128 | 0.002009 | 0.001172 | 0.004238 | 0.001609 | 418 | 8 | 0.007532 |
| 2 | yes | 1519 | 5.000 | 0.000001 | 0.000000 | 0.001027 | 0.002595 | 0.001673 | 0.005388 | 0.001297 | 78 | 4 | 0.007709 |
| 3 | yes | 1510 | 5.000 | 0.000006 | 0.000000 | 0.000658 | 0.001581 | 0.001519 | 0.004172 | 0.001135 | 104 | 5 | 0.007493 |
| 4 | yes | 1509 | 5.000 | 0.000003 | 0.000000 | 0.000597 | 0.001440 | 0.001378 | 0.003983 | 0.000981 | 112 | 3 | 0.007580 |
| 5 | yes | 1521 | 5.000 | 0.000005 | 0.000000 | 0.000408 | 0.000993 | 0.001203 | 0.003295 | 0.001880 | 152 | 6 | 0.007664 |
| 6 | yes | 1509 | 5.000 | 0.000002 | 0.000000 | 0.000805 | 0.001942 | 0.001597 | 0.004174 | 0.001342 | 90 | 3 | 0.007614 |
| 7 | yes | 1530 | 5.000 | 0.000001 | 0.000000 | 0.000852 | 0.002084 | 0.001482 | 0.004454 | 0.000711 | 88 | 6 | 0.007564 |
| 8 | yes | 653 | 5.000 | 0.000002 | 0.000000 | 0.001308 | 0.001772 | 0.001855 | 0.004569 | 0.000681 | 11 | 2 | 0.007550 |
| 9 | no | 1354 | 25.000 | 0.409676 | 0.000000 | 0.001326 | 0.001496 | 0.001910 | 0.004894 | 0.003261 | 23 | 4 | 0.007619 |

Interpretation: this is an offline passive-log diagnostic. A hold-like command is a single-point trajectory lasting at least 1 s. The analyzer compares the controller reference and feedback in each command window; it does not publish commands or alter the task safety gates.
