# Search Tracking Sensitivity Analysis

- input_dir: `diagnostics/research_baseline_velocity_state_500hz_gain3000_damping10_25hz_v1`
- command_source: `trajectory_commands.csv`
- tracking_source: `trajectory_controller_state_samples.csv`
- physical_clearance_m: `0.001000`
- hold_like_command_count: `2`
- centered_hold_like_command_count: `2`
- max_centered_hold_p95_actual_xy_drift_m: `0.000961`
- max_centered_hold_p95_joint_error_rad: `0.003774`

| Cmd | Hold | Centered target | Samples | Target XY m | Feedback mean XY m | Actual XY drift p95 m | Linearized XY drift p95 m | Residual p95 m | JTC p95 rad | Max sens p95 m/rad | Cond p95 | Dominant joint p95 m |
| ---: | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 0 | no | yes | 20016 | 0.000000 | 0.228152 | 0.001168 | 0.001168 | 0.000002 | 0.006283 | 0.848862 | 1.885 | joint_5 0.000827 |
| 1 | no | yes | 5727 | 0.000000 | 0.000497 | 0.000892 | 0.000892 | 0.000001 | 0.003701 | 0.659330 | 1.089 | joint_3 0.000475 |
| 2 | yes | yes | 1140 | 0.000000 | 0.000542 | 0.000961 | 0.000960 | 0.000001 | 0.003645 | 0.660104 | 1.092 | joint_1 0.000504 |
| 3 | yes | yes | 11526 | 0.000000 | 0.000559 | 0.000927 | 0.000927 | 0.000001 | 0.003774 | 0.656715 | 1.099 | joint_1 0.000501 |
| 4 | no | no | 12572 | 0.409676 | 0.101577 | 0.001063 | 0.001061 | 0.000002 | 0.005268 | 0.845163 | 1.709 | joint_5 0.000671 |

Interpretation: this passive diagnostic uses the current finite-difference peg-tip Jacobian at each controller reference. The linearized XY drift is `J_xy * (feedback - reference)`. Close agreement between actual and linearized drift means controller tracking error is sufficient to explain the Cartesian offset; a large residual points to model, frame, or log consistency issues.
