# Search Tracking Sensitivity Analysis

- input_dir: `diagnostics/research_baseline_search_damping10_gain3000_25hz_v1`
- command_source: `trajectory_commands.csv`
- tracking_source: `trajectory_controller_state_samples.csv`
- physical_clearance_m: `0.001000`
- hold_like_command_count: `1`
- centered_hold_like_command_count: `1`
- max_centered_hold_p95_actual_xy_drift_m: `0.002130`
- max_centered_hold_p95_joint_error_rad: `0.007546`

| Cmd | Hold | Centered target | Samples | Target XY m | Feedback mean XY m | Actual XY drift p95 m | Linearized XY drift p95 m | Residual p95 m | JTC p95 rad | Max sens p95 m/rad | Cond p95 | Dominant joint p95 m |
| ---: | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 0 | no | yes | 9950 | 0.000000 | 0.229172 | 0.002726 | 0.002726 | 0.000010 | 0.012933 | 0.847856 | 1.883 | joint_5 0.001788 |
| 1 | no | yes | 2932 | 0.000000 | 0.001283 | 0.002213 | 0.002209 | 0.000008 | 0.007659 | 0.659954 | 1.091 | joint_1 0.001416 |
| 2 | yes | yes | 1511 | 0.000000 | 0.001176 | 0.002130 | 0.002132 | 0.000007 | 0.007546 | 0.659929 | 1.092 | joint_1 0.001306 |
| 3 | no | no | 5691 | 0.409676 | 0.106945 | 0.002655 | 0.002655 | 0.000010 | 0.011924 | 0.833598 | 1.502 | joint_1 0.001747 |

Interpretation: this passive diagnostic uses the current finite-difference peg-tip Jacobian at each controller reference. The linearized XY drift is `J_xy * (feedback - reference)`. Close agreement between actual and linearized drift means controller tracking error is sufficient to explain the Cartesian offset; a large residual points to model, frame, or log consistency issues.
