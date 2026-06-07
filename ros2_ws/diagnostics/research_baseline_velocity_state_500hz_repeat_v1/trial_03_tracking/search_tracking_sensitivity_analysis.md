# Search Tracking Sensitivity Analysis

- input_dir: `diagnostics/research_baseline_velocity_state_500hz_repeat_v1/trial_03_tracking`
- command_source: `trajectory_commands.csv`
- tracking_source: `trajectory_controller_state_samples.csv`
- physical_clearance_m: `0.001000`
- hold_like_command_count: `2`
- centered_hold_like_command_count: `2`
- max_centered_hold_p95_actual_xy_drift_m: `0.000928`
- max_centered_hold_p95_joint_error_rad: `0.003805`

| Cmd | Hold | Centered target | Samples | Target XY m | Feedback mean XY m | Actual XY drift p95 m | Linearized XY drift p95 m | Residual p95 m | JTC p95 rad | Max sens p95 m/rad | Cond p95 | Dominant joint p95 m |
| ---: | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 0 | no | yes | 20027 | 0.000000 | 0.228511 | 0.001167 | 0.001167 | 0.000002 | 0.006295 | 0.849231 | 1.886 | joint_5 0.000837 |
| 1 | no | yes | 5796 | 0.000000 | 0.000497 | 0.000904 | 0.000904 | 0.000001 | 0.003704 | 0.659320 | 1.090 | joint_3 0.000503 |
| 2 | yes | yes | 1140 | 0.000000 | 0.000481 | 0.000923 | 0.000923 | 0.000001 | 0.003805 | 0.659851 | 1.091 | joint_1 0.000484 |
| 3 | yes | yes | 1426 | 0.000000 | 0.000759 | 0.000928 | 0.000928 | 0.000001 | 0.003795 | 0.652823 | 1.076 | joint_3 0.000499 |
| 4 | no | no | 12338 | 0.409676 | 0.129027 | 0.001094 | 0.001094 | 0.000002 | 0.005681 | 0.844697 | 1.694 | joint_5 0.000726 |

Interpretation: this passive diagnostic uses the current finite-difference peg-tip Jacobian at each controller reference. The linearized XY drift is `J_xy * (feedback - reference)`. Close agreement between actual and linearized drift means controller tracking error is sufficient to explain the Cartesian offset; a large residual points to model, frame, or log consistency issues.
