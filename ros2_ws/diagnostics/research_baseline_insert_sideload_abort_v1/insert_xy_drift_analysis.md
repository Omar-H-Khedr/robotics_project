# Insert XY Drift Analysis

- input_dir: `diagnostics/research_baseline_insert_sideload_abort_v1`
- tracking_source: `trajectory_controller_state_samples.csv`
- command_index: `2`
- command_point_count: `1`
- command_receipt_stamp_s: `54.808`
- next_command_stamp_s: `67.618`
- observed_window_s: `12.810`
- samples: `3203`
- hole_centre_xy_m: `0.520000, -0.200000`
- target_xyz_m: `0.520000, -0.200000, 0.790000`
- physical_clearance_m: `0.001000`
- meaningful_depth_m: `0.001000`
- pre_command_window_s: `1.000`
- pre_command_samples: `249`

## Pre-Command Boundary

- pre_command_final_xy_error_m: `0.001569`
- pre_command_final_depth_m: `0.000000`
- pre_command_max_xy_error_m: `0.004738`

## Drift Summary

- initial_xy_error_m: `0.002539`
- final_xy_error_m: `0.002777`
- max_xy_error_m: `0.008116`
- p95_xy_error_m: `0.005441`
- final_depth_m: `0.000677`
- max_depth_m: `0.006160`
- max_abs_joint_error_rad: `0.010430`
- p95_max_abs_joint_error_rad: `0.008357`

## First Events

| event | elapsed_s | stamp_s | xy_error_m | depth_m | z_m | worst_joint | worst_joint_error_rad |
| --- | ---: | ---: | ---: | ---: | ---: | --- | ---: |
| clearance_violation | 0.000 | 54.808 | 0.002539 | 0.000000 | 0.841878 | joint_5 | 0.003631 |
| meaningful_depth | 10.824 | 65.632 | 0.002488 | 0.001316 | 0.808684 | joint_2 | -0.006162 |
| sideload | 10.824 | 65.632 | 0.002488 | 0.001316 | 0.808684 | joint_2 | -0.006162 |

## Correlated Sensor Values

- nearest_fz_at_first_clearance_violation_n: `59.796449`
- nearest_contact_at_first_clearance_violation_n: `0.000000`

Interpretation: this is an offline diagnostic over passive observer CSVs. It does not publish commands or alter safety gates.
