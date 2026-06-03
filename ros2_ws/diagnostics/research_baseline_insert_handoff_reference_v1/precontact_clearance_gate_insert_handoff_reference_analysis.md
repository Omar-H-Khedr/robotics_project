# Insert Handoff Reference Analysis

- input_dir: `ros2_ws/diagnostics/research_baseline_insert_precontact_clearance_gate_v1`
- tracking_source: `trajectory_controller_state_samples.csv`
- command_index: `2`
- command_point_count: `1`
- command_receipt_stamp_s: `63.203`
- command_duration_s: `20.000`
- post_command_window_s: `0.500`
- samples: `124`
- physical_clearance_m: `0.001000`
- command_target_xyz_m: `0.520004, -0.200001, 0.790004`

## Boundary

- pre_command_final_reference_xy_error_m: `0.000011`
- pre_command_final_feedback_xy_error_m: `0.002101`
- initial_reference_xy_error_m: `0.001125`
- initial_feedback_xy_error_m: `0.001125`
- final_reference_xy_error_m: `0.001149`
- final_feedback_xy_error_m: `0.005312`

## Reference vs Feedback

- max_reference_xy_error_m: `0.001149`
- max_feedback_xy_error_m: `0.005525`
- p95_reference_xy_error_m: `0.001148`
- p95_feedback_xy_error_m: `0.004541`
- max_cartesian_reference_feedback_error_m: `0.005848`
- p95_cartesian_reference_feedback_error_m: `0.004938`
- max_abs_joint_error_rad: `0.009251`
- p95_max_abs_joint_error_rad: `0.007998`

## First Clearance Violations

| signal | elapsed_s | stamp_s | xy_error_m | z_m | depth_m | worst_joint | worst_joint_error_rad |
| --- | ---: | ---: | ---: | ---: | ---: | --- | ---: |
| reference | 0.005 | 63.208 | 0.001125 | 0.830917 | 0.000000 | joint_1 | 0.000000 |
| feedback | 0.005 | 63.208 | 0.001125 | 0.830917 | 0.000000 | joint_1 | 0.000000 |

Interpretation: this offline diagnostic compares the JTC reference and feedback at the INSERT handoff. It does not publish commands or alter safety gates.
