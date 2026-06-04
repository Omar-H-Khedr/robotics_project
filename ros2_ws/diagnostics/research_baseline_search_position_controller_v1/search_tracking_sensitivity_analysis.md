# Search Tracking Sensitivity Analysis

- input_dir: `diagnostics/research_baseline_search_position_controller_v1`
- command_source: `trajectory_commands.csv`
- tracking_source: `trajectory_controller_state_samples.csv`
- physical_clearance_m: `0.001000`
- hold_like_command_count: `7`
- centered_hold_like_command_count: `0`
- max_centered_hold_p95_actual_xy_drift_m: `0.000000`
- max_centered_hold_p95_joint_error_rad: `0.000000`

| Cmd | Hold | Centered target | Samples | Target XY m | Feedback mean XY m | Actual XY drift p95 m | Linearized XY drift p95 m | Residual p95 m | JTC p95 rad | Max sens p95 m/rad | Cond p95 | Dominant joint p95 m |
| ---: | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 0 | no | yes | 0 | 0.000000 | none | none | none | none | none | none | none | joint_1 none |
| 1 | no | yes | 0 | 0.000000 | none | none | none | none | none | none | none | joint_1 none |
| 2 | yes | no | 0 | 0.003001 | none | none | none | none | none | none | none | joint_1 none |
| 3 | yes | yes | 0 | 0.000009 | none | none | none | none | none | none | none | joint_1 none |
| 4 | yes | yes | 0 | 0.000007 | none | none | none | none | none | none | none | joint_1 none |
| 5 | yes | yes | 0 | 0.000001 | none | none | none | none | none | none | none | joint_1 none |
| 6 | yes | yes | 0 | 0.000005 | none | none | none | none | none | none | none | joint_1 none |
| 7 | yes | yes | 0 | 0.000002 | none | none | none | none | none | none | none | joint_1 none |
| 8 | yes | yes | 0 | 0.000001 | none | none | none | none | none | none | none | joint_1 none |
| 9 | no | no | 0 | 0.409676 | none | none | none | none | none | none | none | joint_1 none |

Interpretation: this passive diagnostic uses the current finite-difference peg-tip Jacobian at each controller reference. The linearized XY drift is `J_xy * (feedback - reference)`. Close agreement between actual and linearized drift means controller tracking error is sufficient to explain the Cartesian offset; a large residual points to model, frame, or log consistency issues.
