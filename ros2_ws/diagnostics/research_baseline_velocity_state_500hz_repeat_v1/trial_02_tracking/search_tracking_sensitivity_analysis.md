# Search Tracking Sensitivity Analysis

- input_dir: `diagnostics/research_baseline_velocity_state_500hz_repeat_v1/trial_02_tracking`
- command_source: `trajectory_commands.csv`
- tracking_source: `trajectory_controller_state_samples.csv`
- physical_clearance_m: `0.001000`
- hold_like_command_count: `2`
- centered_hold_like_command_count: `2`
- max_centered_hold_p95_actual_xy_drift_m: `0.000958`
- max_centered_hold_p95_joint_error_rad: `0.003728`

| Cmd | Hold | Centered target | Samples | Target XY m | Feedback mean XY m | Actual XY drift p95 m | Linearized XY drift p95 m | Residual p95 m | JTC p95 rad | Max sens p95 m/rad | Cond p95 | Dominant joint p95 m |
| ---: | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 0 | no | yes | 20046 | 0.000000 | 0.228077 | 0.001171 | 0.001171 | 0.000002 | 0.006274 | 0.848575 | 1.886 | joint_5 0.000838 |
| 1 | no | yes | 5884 | 0.000000 | 0.000495 | 0.000908 | 0.000908 | 0.000001 | 0.003750 | 0.659376 | 1.090 | joint_3 0.000505 |
| 2 | yes | yes | 1244 | 0.000000 | 0.000532 | 0.000958 | 0.000958 | 0.000001 | 0.003728 | 0.660012 | 1.092 | joint_1 0.000525 |
| 3 | yes | yes | 182 | 0.000000 | 0.000806 | 0.000845 | 0.000845 | 0.000001 | 0.003547 | 0.652598 | 1.077 | joint_6 0.000443 |
| 4 | no | no | 12466 | 0.409676 | 0.131343 | 0.001085 | 0.001085 | 0.000002 | 0.005756 | 0.845562 | 1.722 | joint_5 0.000725 |

Interpretation: this passive diagnostic uses the current finite-difference peg-tip Jacobian at each controller reference. The linearized XY drift is `J_xy * (feedback - reference)`. Close agreement between actual and linearized drift means controller tracking error is sufficient to explain the Cartesian offset; a large residual points to model, frame, or log consistency issues.
