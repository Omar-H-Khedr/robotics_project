# Insert XY Drift Analysis

- input_dir: `diagnostics/research_baseline_insert_physical_xy_gate_v1`
- tracking_source: `trajectory_controller_state_samples.csv`
- command_index: `2`
- command_point_count: `1`
- command_receipt_stamp_s: `62.602`
- next_command_stamp_s: `85.702`
- observed_window_s: `23.100`
- samples: `5775`
- hole_centre_xy_m: `0.520000, -0.200000`
- target_xyz_m: `0.520000, -0.200000, 0.790000`
- physical_clearance_m: `0.001000`
- meaningful_depth_m: `0.001000`
- pre_command_window_s: `1.000`
- pre_command_samples: `250`

## Pre-Command Boundary

- pre_command_final_xy_error_m: `0.001380`
- pre_command_final_depth_m: `0.000000`
- pre_command_max_xy_error_m: `0.004986`

## Drift Summary

- initial_xy_error_m: `0.001151`
- final_xy_error_m: `0.004332`
- max_xy_error_m: `0.005838`
- p95_xy_error_m: `0.004014`
- final_depth_m: `0.017916`
- max_depth_m: `0.024905`
- max_abs_joint_error_rad: `0.010498`
- p95_max_abs_joint_error_rad: `0.008381`

## First Events

| event | elapsed_s | stamp_s | xy_error_m | depth_m | z_m | worst_joint | worst_joint_error_rad |
| --- | ---: | ---: | ---: | ---: | ---: | --- | ---: |
| clearance_violation | 0.001 | 62.603 | 0.001151 | 0.000000 | 0.840535 | joint_1 | 0.000000 |
| meaningful_depth | 10.702 | 73.304 | 0.000323 | 0.001103 | 0.808897 | joint_2 | -0.006035 |
| sideload | 10.834 | 73.436 | 0.002153 | 0.001274 | 0.808726 | joint_6 | 0.007047 |

## Correlated Sensor Values

- nearest_fz_at_first_clearance_violation_n: `-10.358431`
- nearest_contact_at_first_clearance_violation_n: `130.162091`

Interpretation: this is an offline diagnostic over passive observer CSVs. It does not publish commands or alter safety gates.
