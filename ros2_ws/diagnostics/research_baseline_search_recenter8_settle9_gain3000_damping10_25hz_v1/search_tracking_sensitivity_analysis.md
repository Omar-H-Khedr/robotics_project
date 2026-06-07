# Search Tracking Sensitivity Analysis

- input_dir: `diagnostics/research_baseline_search_recenter8_settle9_gain3000_damping10_25hz_v1`
- command_source: `trajectory_commands.csv`
- tracking_source: `trajectory_controller_state_samples.csv`
- physical_clearance_m: `0.001000`
- hold_like_command_count: `1`
- centered_hold_like_command_count: `1`
- max_centered_hold_p95_actual_xy_drift_m: `0.002232`
- max_centered_hold_p95_joint_error_rad: `0.007669`

| Cmd | Hold | Centered target | Samples | Target XY m | Feedback mean XY m | Actual XY drift p95 m | Linearized XY drift p95 m | Residual p95 m | JTC p95 rad | Max sens p95 m/rad | Cond p95 | Dominant joint p95 m |
| ---: | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 0 | no | yes | 10025 | 0.000000 | 0.228330 | 0.002738 | 0.002738 | 0.000010 | 0.012922 | 0.849663 | 1.884 | joint_1 0.001780 |
| 1 | no | yes | 3607 | 0.000000 | 0.001239 | 0.002218 | 0.002217 | 0.000008 | 0.007556 | 0.663694 | 1.101 | joint_1 0.001320 |
| 2 | yes | yes | 3024 | 0.000001 | 0.001276 | 0.002232 | 0.002231 | 0.000007 | 0.007669 | 0.662390 | 1.100 | joint_1 0.001381 |
| 3 | no | no | 10752 | 0.409676 | 0.243487 | 0.002621 | 0.002619 | 0.000008 | 0.011225 | 0.849056 | 1.888 | joint_1 0.001782 |

Interpretation: this passive diagnostic uses the current finite-difference peg-tip Jacobian at each controller reference. The linearized XY drift is `J_xy * (feedback - reference)`. Close agreement between actual and linearized drift means controller tracking error is sufficient to explain the Cartesian offset; a large residual points to model, frame, or log consistency issues.
