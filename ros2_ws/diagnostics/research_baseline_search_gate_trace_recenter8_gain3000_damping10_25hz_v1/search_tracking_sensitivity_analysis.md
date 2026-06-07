# Search Tracking Sensitivity Analysis

- input_dir: `diagnostics/research_baseline_search_gate_trace_recenter8_gain3000_damping10_25hz_v1`
- command_source: `trajectory_commands.csv`
- tracking_source: `trajectory_controller_state_samples.csv`
- physical_clearance_m: `0.001000`
- hold_like_command_count: `1`
- centered_hold_like_command_count: `1`
- max_centered_hold_p95_actual_xy_drift_m: `0.002246`
- max_centered_hold_p95_joint_error_rad: `0.007544`

| Cmd | Hold | Centered target | Samples | Target XY m | Feedback mean XY m | Actual XY drift p95 m | Linearized XY drift p95 m | Residual p95 m | JTC p95 rad | Max sens p95 m/rad | Cond p95 | Dominant joint p95 m |
| ---: | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 0 | no | yes | 10057 | 0.000000 | 0.226112 | 0.002769 | 0.002766 | 0.000010 | 0.013043 | 0.847925 | 1.887 | joint_1 0.001772 |
| 1 | no | yes | 2917 | 0.000000 | 0.001137 | 0.002191 | 0.002190 | 0.000007 | 0.007648 | 0.659331 | 1.089 | joint_1 0.001310 |
| 2 | yes | yes | 3013 | 0.000000 | 0.001174 | 0.002246 | 0.002246 | 0.000008 | 0.007544 | 0.657683 | 1.086 | joint_1 0.001364 |
| 3 | no | no | 6264 | 0.409676 | 0.141810 | 0.002690 | 0.002691 | 0.000009 | 0.012068 | 0.846212 | 1.742 | joint_1 0.001821 |

Interpretation: this passive diagnostic uses the current finite-difference peg-tip Jacobian at each controller reference. The linearized XY drift is `J_xy * (feedback - reference)`. Close agreement between actual and linearized drift means controller tracking error is sufficient to explain the Cartesian offset; a large residual points to model, frame, or log consistency issues.
