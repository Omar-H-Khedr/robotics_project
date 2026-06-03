# Insert Handoff Reference Analysis

- input_dir: `ros2_ws/diagnostics/research_baseline_insert_handoff_settle_v1`
- tracking_source: `trajectory_controller_state_samples.csv`
- command_index: `2`
- command_point_count: `1`
- command_receipt_stamp_s: `55.703`
- command_duration_s: `2.000`
- post_command_window_s: `0.500`
- samples: `74`
- physical_clearance_m: `0.001000`
- command_target_xyz_m: `0.520000, -0.200000, 0.840697`

## Boundary

- pre_command_final_reference_xy_error_m: `0.000010`
- pre_command_final_feedback_xy_error_m: `0.002773`
- initial_reference_xy_error_m: `0.001747`
- initial_feedback_xy_error_m: `0.001747`
- final_reference_xy_error_m: `0.001546`
- final_feedback_xy_error_m: `0.003138`

## Reference vs Feedback

- max_reference_xy_error_m: `0.001747`
- max_feedback_xy_error_m: `0.004553`
- p95_reference_xy_error_m: `0.001737`
- p95_feedback_xy_error_m: `0.003678`
- max_cartesian_reference_feedback_error_m: `0.004938`
- p95_cartesian_reference_feedback_error_m: `0.004392`
- max_abs_joint_error_rad: `0.009807`
- p95_max_abs_joint_error_rad: `0.008186`

## First Clearance Violations

| signal | elapsed_s | stamp_s | xy_error_m | z_m | depth_m | worst_joint | worst_joint_error_rad |
| --- | ---: | ---: | ---: | ---: | ---: | --- | ---: |
| reference | 0.000 | 55.703 | 0.001747 | 0.838968 | 0.000000 | joint_1 | 0.000000 |
| feedback | 0.000 | 55.703 | 0.001747 | 0.838968 | 0.000000 | joint_1 | 0.000000 |

Interpretation: this offline diagnostic compares the JTC reference and feedback at the INSERT handoff. It does not publish commands or alter safety gates.
