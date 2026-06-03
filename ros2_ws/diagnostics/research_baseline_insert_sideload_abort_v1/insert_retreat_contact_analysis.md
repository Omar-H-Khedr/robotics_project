# Insert / Retreat Contact Analysis

- input_dir: `diagnostics/research_baseline_insert_sideload_abort_v1`
- tracking_source: `trajectory_controller_state_samples.csv`
- command_index: `2`
- command_receipt_stamp_s: `54.808`
- next_command_stamp_s: `67.618`
- insert_command_duration_s: `20.000`
- observed_window_s: `12.810`
- samples: `3203`
- max_abs_position_error_rad: `0.010430`
- p95_max_abs_position_error_rad: `0.008357`
- mean_rms_position_error_rad: `0.002893`

## Cartesian Peg Tip

- command_target_xyz_m: `0.520000, -0.200000, 0.790000`
- final_feedback_xyz_m: `0.517346, -0.200819, 0.809323`
- final_cartesian_error_xyz_m: `0.002654, 0.000819, -0.019323`
- final_cartesian_error_norm_m: `0.019522`
- missing_descent_to_target_m: `0.019323`
- hole_top_z_m: `0.810000`
- min_feedback_z_m: `0.803840`
- max_physical_depth_m: `0.006160`
- final_physical_depth_m: `0.000677`

## Contact By State

| state | first_stamp_s | samples | positive_samples | max_contact_force_n | top_collision_pair |
| --- | ---: | ---: | ---: | ---: | --- |
| ABORT | 67.717 | 2 | 2 | 0.000000 | `lbr_iisy6_r1300::ft_sensor_link::ft_sensor_link_fixed_joint_lump__grasped_peg_collision_2 <-> target_plate::plate_link::plate_left_collision` |

## Wrench By State

| state | samples | max_abs_fz_n | max_force_norm_n | mean_xy_error_m |
| --- | ---: | ---: | ---: | ---: |
| ABORT | 2470 | 124.883748 | 204.140105 | 0.104975 |
| APPROACH | 1190 | 128.381883 | 199.020908 | 0.002319 |
| DONE | 12416 | 125.774867 | 188.319738 | 0.409261 |
| INSERT | 1280 | 126.773956 | 200.165296 | 0.002909 |
| MOVING_TO_START | 4010 | 128.534595 | 201.745461 | 0.227215 |
| UNKNOWN | 271 | 116.810784 | 177.865051 | 0.409531 |

Interpretation: this is an offline diagnostic over passive observer CSVs. It does not publish commands or change task safety gates.
