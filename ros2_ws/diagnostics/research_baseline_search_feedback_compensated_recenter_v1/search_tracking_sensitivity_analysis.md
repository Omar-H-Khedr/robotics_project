# Search Tracking Sensitivity Analysis

- input_dir: `diagnostics/research_baseline_search_feedback_compensated_recenter_v1`
- command_source: `trajectory_commands.csv`
- tracking_source: `trajectory_controller_state_samples.csv`
- physical_clearance_m: `0.001000`
- hold_like_command_count: `7`
- centered_hold_like_command_count: `1`
- max_centered_hold_p95_actual_xy_drift_m: `0.003483`
- max_centered_hold_p95_joint_error_rad: `0.008287`

| Cmd | Hold | Centered target | Samples | Target XY m | Feedback mean XY m | Actual XY drift p95 m | Linearized XY drift p95 m | Residual p95 m | JTC p95 rad | Max sens p95 m/rad | Cond p95 | Dominant joint p95 m |
| ---: | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 0 | no | yes | 10468 | 0.000000 | 0.217886 | 0.004590 | 0.004588 | 0.000020 | 0.013142 | 0.847704 | 1.874 | joint_1 0.003504 |
| 1 | no | yes | 4329 | 0.000000 | 0.002104 | 0.003832 | 0.003830 | 0.000019 | 0.008434 | 0.665101 | 1.106 | joint_1 0.002701 |
| 2 | yes | yes | 1527 | 0.000864 | 0.001878 | 0.003483 | 0.003480 | 0.000016 | 0.008287 | 0.664659 | 1.104 | joint_1 0.002385 |
| 3 | yes | no | 1524 | 0.001525 | 0.002280 | 0.003727 | 0.003727 | 0.000018 | 0.008287 | 0.655085 | 1.086 | joint_1 0.002530 |
| 4 | yes | no | 1524 | 0.002000 | 0.002477 | 0.003759 | 0.003756 | 0.000017 | 0.008397 | 0.655798 | 1.088 | joint_1 0.002693 |
| 5 | yes | no | 1525 | 0.001957 | 0.002713 | 0.003915 | 0.003907 | 0.000020 | 0.008365 | 0.654723 | 1.088 | joint_1 0.002672 |
| 6 | yes | no | 1524 | 0.002000 | 0.002389 | 0.003820 | 0.003804 | 0.000019 | 0.008316 | 0.654799 | 1.087 | joint_1 0.002598 |
| 7 | yes | no | 1527 | 0.003000 | 0.002759 | 0.003881 | 0.003878 | 0.000018 | 0.008440 | 0.654517 | 1.087 | joint_1 0.002760 |
| 8 | yes | no | 601 | 0.003001 | 0.004075 | 0.003713 | 0.003712 | 0.000019 | 0.008394 | 0.655758 | 1.092 | joint_1 0.002590 |
| 9 | no | no | 12762 | 0.409676 | 0.266503 | 0.004449 | 0.004450 | 0.000019 | 0.011736 | 0.849056 | 1.888 | joint_1 0.003460 |

Interpretation: this passive diagnostic uses the current finite-difference peg-tip Jacobian at each controller reference. The linearized XY drift is `J_xy * (feedback - reference)`. Close agreement between actual and linearized drift means controller tracking error is sufficient to explain the Cartesian offset; a large residual points to model, frame, or log consistency issues.
