# Search Tracking Sensitivity Analysis

- input_dir: `diagnostics/research_baseline_velocity_state_500hz_repeat_v1/trial_01_tracking`
- command_source: `trajectory_commands.csv`
- tracking_source: `trajectory_controller_state_samples.csv`
- physical_clearance_m: `0.001000`
- hold_like_command_count: `2`
- centered_hold_like_command_count: `2`
- max_centered_hold_p95_actual_xy_drift_m: `0.000912`
- max_centered_hold_p95_joint_error_rad: `0.003824`

| Cmd | Hold | Centered target | Samples | Target XY m | Feedback mean XY m | Actual XY drift p95 m | Linearized XY drift p95 m | Residual p95 m | JTC p95 rad | Max sens p95 m/rad | Cond p95 | Dominant joint p95 m |
| ---: | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 0 | no | yes | 20033 | 0.000000 | 0.228394 | 0.001197 | 0.001197 | 0.000002 | 0.006300 | 0.848955 | 1.886 | joint_5 0.000833 |
| 1 | no | yes | 5830 | 0.000000 | 0.000460 | 0.000897 | 0.000897 | 0.000001 | 0.003729 | 0.659329 | 1.089 | joint_3 0.000494 |
| 2 | yes | yes | 1139 | 0.000000 | 0.000514 | 0.000897 | 0.000897 | 0.000001 | 0.003824 | 0.660129 | 1.092 | joint_6 0.000478 |
| 3 | yes | yes | 11543 | 0.000000 | 0.000648 | 0.000912 | 0.000912 | 0.000001 | 0.003782 | 0.656716 | 1.099 | joint_1 0.000484 |
| 4 | no | no | 12493 | 0.409676 | 0.099662 | 0.001059 | 0.001059 | 0.000002 | 0.005161 | 0.844291 | 1.685 | joint_5 0.000664 |

Interpretation: this passive diagnostic uses the current finite-difference peg-tip Jacobian at each controller reference. The linearized XY drift is `J_xy * (feedback - reference)`. Close agreement between actual and linearized drift means controller tracking error is sufficient to explain the Cartesian offset; a large residual points to model, frame, or log consistency issues.
