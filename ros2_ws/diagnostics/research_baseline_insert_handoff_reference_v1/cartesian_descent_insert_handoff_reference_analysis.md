# Insert Handoff Reference Analysis

- input_dir: `ros2_ws/diagnostics/research_baseline_insert_cartesian_descent_v1`
- tracking_source: `trajectory_controller_state_samples.csv`
- command_index: `2`
- command_point_count: `6`
- command_receipt_stamp_s: `61.711`
- command_duration_s: `20.000`
- post_command_window_s: `0.500`
- samples: `75`
- physical_clearance_m: `0.001000`
- command_target_xyz_m: `0.520000, -0.200000, 0.790000`

## Boundary

- pre_command_final_reference_xy_error_m: `0.000000`
- pre_command_final_feedback_xy_error_m: `0.001660`
- initial_reference_xy_error_m: `0.000458`
- initial_feedback_xy_error_m: `0.000458`
- final_reference_xy_error_m: `0.000510`
- final_feedback_xy_error_m: `0.002675`

## Reference vs Feedback

- max_reference_xy_error_m: `0.000510`
- max_feedback_xy_error_m: `0.004264`
- p95_reference_xy_error_m: `0.000507`
- p95_feedback_xy_error_m: `0.003602`
- max_cartesian_reference_feedback_error_m: `0.004571`
- p95_cartesian_reference_feedback_error_m: `0.003842`
- max_abs_joint_error_rad: `0.009622`
- p95_max_abs_joint_error_rad: `0.007322`

## First Clearance Violations

| signal | elapsed_s | stamp_s | xy_error_m | z_m | depth_m | worst_joint | worst_joint_error_rad |
| --- | ---: | ---: | ---: | ---: | ---: | --- | ---: |
| reference | none | none | none | none | none | none | none |
| feedback | 0.005 | 61.716 | 0.001725 | 0.829307 | 0.000000 | joint_4 | 0.002697 |

Interpretation: this offline diagnostic compares the JTC reference and feedback at the INSERT handoff. It does not publish commands or alter safety gates.
