# Insert XY Drift Analysis

- input_dir: `ros2_ws/diagnostics/research_baseline_insert_handoff_settle_v1`
- tracking_source: `trajectory_controller_state_samples.csv`
- command_index: `2`
- command_point_count: `1`
- command_receipt_stamp_s: `55.703`
- next_command_stamp_s: `56.002`
- observed_window_s: `0.299`
- samples: `74`
- hole_centre_xy_m: `0.520000, -0.200000`
- target_xyz_m: `0.520000, -0.200000, 0.840697`
- physical_clearance_m: `0.001000`
- meaningful_depth_m: `0.001000`
- pre_command_window_s: `1.000`
- pre_command_samples: `250`

## Pre-Command Boundary

- pre_command_final_xy_error_m: `0.002773`
- pre_command_final_depth_m: `0.000000`
- pre_command_max_xy_error_m: `0.004337`

## Drift Summary

- initial_xy_error_m: `0.001747`
- final_xy_error_m: `0.003138`
- max_xy_error_m: `0.004553`
- p95_xy_error_m: `0.003678`
- final_depth_m: `0.000000`
- max_depth_m: `0.000000`
- max_abs_joint_error_rad: `0.009807`
- p95_max_abs_joint_error_rad: `0.008186`

## First Events

| event | elapsed_s | stamp_s | xy_error_m | depth_m | z_m | worst_joint | worst_joint_error_rad |
| --- | ---: | ---: | ---: | ---: | ---: | --- | ---: |
| clearance_violation | 0.000 | 55.703 | 0.001747 | 0.000000 | 0.838968 | joint_1 | 0.000000 |
| meaningful_depth | none | none | none | none | none | none | none |
| sideload | none | none | none | none | none | none | none |

## Correlated Sensor Values

- nearest_fz_at_first_clearance_violation_n: `10.891801`
- nearest_contact_at_first_clearance_violation_n: `0.000000`

Interpretation: this is an offline diagnostic over passive observer CSVs. It does not publish commands or alter safety gates.
