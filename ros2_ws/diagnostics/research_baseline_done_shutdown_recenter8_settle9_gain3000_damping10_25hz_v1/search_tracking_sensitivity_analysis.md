# Search Tracking Sensitivity Analysis

- input_dir: `diagnostics/research_baseline_done_shutdown_recenter8_settle9_gain3000_damping10_25hz_v1`
- command_source: `trajectory_commands.csv`
- tracking_source: `trajectory_controller_state_samples.csv`
- physical_clearance_m: `0.001000`
- hold_like_command_count: `3`
- centered_hold_like_command_count: `3`
- max_centered_hold_p95_actual_xy_drift_m: `0.002263`
- max_centered_hold_p95_joint_error_rad: `0.007683`

| Cmd | Hold | Centered target | Samples | Target XY m | Feedback mean XY m | Actual XY drift p95 m | Linearized XY drift p95 m | Residual p95 m | JTC p95 rad | Max sens p95 m/rad | Cond p95 | Dominant joint p95 m |
| ---: | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 0 | no | yes | 9966 | 0.000000 | 0.225627 | 0.002800 | 0.002799 | 0.000010 | 0.013012 | 0.848387 | 1.882 | joint_1 0.001783 |
| 1 | no | yes | 5153 | 0.000000 | 0.001183 | 0.002247 | 0.002244 | 0.000007 | 0.007539 | 0.665071 | 1.106 | joint_1 0.001366 |
| 2 | yes | yes | 2260 | 0.000001 | 0.001138 | 0.002190 | 0.002190 | 0.000007 | 0.007543 | 0.664631 | 1.105 | joint_1 0.001327 |
| 3 | yes | yes | 2271 | 0.000001 | 0.001259 | 0.002247 | 0.002246 | 0.000007 | 0.007572 | 0.654304 | 1.085 | joint_1 0.001364 |
| 4 | yes | yes | 1957 | 0.000003 | 0.001218 | 0.002263 | 0.002262 | 0.000007 | 0.007683 | 0.654756 | 1.085 | joint_1 0.001341 |

Interpretation: this passive diagnostic uses the current finite-difference peg-tip Jacobian at each controller reference. The linearized XY drift is `J_xy * (feedback - reference)`. Close agreement between actual and linearized drift means controller tracking error is sufficient to explain the Cartesian offset; a large residual points to model, frame, or log consistency issues.
