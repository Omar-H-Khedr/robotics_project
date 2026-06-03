# Search Tracking Sensitivity Analysis

- input_dir: `diagnostics/research_baseline_search_streak_preservation_v1`
- command_source: `trajectory_commands.csv`
- tracking_source: `trajectory_controller_state_samples.csv`
- physical_clearance_m: `0.001000`
- hold_like_command_count: `7`
- centered_hold_like_command_count: `7`
- max_centered_hold_p95_actual_xy_drift_m: `0.003981`
- max_centered_hold_p95_joint_error_rad: `0.008404`

| Cmd | Hold | Centered target | Samples | Target XY m | Feedback mean XY m | Actual XY drift p95 m | Linearized XY drift p95 m | Residual p95 m | JTC p95 rad | Max sens p95 m/rad | Cond p95 | Dominant joint p95 m |
| ---: | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 0 | no | yes | 10619 | 0.000000 | 0.214382 | 0.004633 | 0.004625 | 0.000021 | 0.013057 | 0.849951 | 1.883 | joint_1 0.003595 |
| 1 | no | yes | 4575 | 0.000000 | 0.002112 | 0.003790 | 0.003786 | 0.000018 | 0.008386 | 0.664929 | 1.105 | joint_1 0.002682 |
| 2 | yes | yes | 1524 | 0.000001 | 0.002224 | 0.003981 | 0.003974 | 0.000020 | 0.008310 | 0.664467 | 1.104 | joint_1 0.002834 |
| 3 | yes | yes | 1526 | 0.000000 | 0.002221 | 0.003876 | 0.003869 | 0.000018 | 0.008388 | 0.655556 | 1.086 | joint_1 0.002645 |
| 4 | yes | yes | 1525 | 0.000002 | 0.002254 | 0.003720 | 0.003709 | 0.000018 | 0.008341 | 0.653461 | 1.079 | joint_1 0.002477 |
| 5 | yes | yes | 1525 | 0.000000 | 0.002498 | 0.003812 | 0.003811 | 0.000017 | 0.008391 | 0.655630 | 1.087 | joint_1 0.002839 |
| 6 | yes | yes | 1551 | 0.000003 | 0.002245 | 0.003763 | 0.003759 | 0.000019 | 0.008404 | 0.654862 | 1.082 | joint_1 0.002680 |
| 7 | yes | yes | 1524 | 0.000000 | 0.002363 | 0.003728 | 0.003726 | 0.000018 | 0.008347 | 0.654939 | 1.085 | joint_1 0.002587 |
| 8 | yes | yes | 579 | 0.000002 | 0.002571 | 0.003680 | 0.003677 | 0.000019 | 0.008169 | 0.654815 | 1.085 | joint_1 0.002703 |
| 9 | no | no | 24723 | 0.409676 | 0.337152 | 0.004300 | 0.004298 | 0.000018 | 0.011657 | 0.849056 | 1.888 | joint_1 0.003357 |

Interpretation: this passive diagnostic uses the current finite-difference peg-tip Jacobian at each controller reference. The linearized XY drift is `J_xy * (feedback - reference)`. Close agreement between actual and linearized drift means controller tracking error is sufficient to explain the Cartesian offset; a large residual points to model, frame, or log consistency issues.
