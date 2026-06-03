# Insert / Retreat Contact Analysis

- input_dir: `diagnostics/research_baseline_insert_sim_time_completion_v4`
- tracking_source: `trajectory_controller_state_samples.csv`
- command_index: `2`
- command_receipt_stamp_s: `57.608`
- next_command_stamp_s: `80.719`
- insert_command_duration_s: `20.000`
- observed_window_s: `23.111`
- samples: `5778`
- max_abs_position_error_rad: `0.010552`
- p95_max_abs_position_error_rad: `0.008321`
- mean_rms_position_error_rad: `0.002871`

## Cartesian Peg Tip

- command_target_xyz_m: `0.520000, -0.200000, 0.790000`
- final_feedback_xyz_m: `0.518615, -0.199574, 0.790557`
- final_cartesian_error_xyz_m: `0.001385, -0.000426, -0.000557`
- final_cartesian_error_norm_m: `0.001552`
- missing_descent_to_target_m: `0.000557`
- hole_top_z_m: `0.810000`
- min_feedback_z_m: `0.785650`
- max_physical_depth_m: `0.024350`
- final_physical_depth_m: `0.019443`

## Contact By State

| state | first_stamp_s | samples | positive_samples | max_contact_force_n | top_collision_pair |
| --- | ---: | ---: | ---: | ---: | --- |
| RETREAT | 81.273 | 41 | 41 | 249.593329 | `lbr_iisy6_r1300::ft_sensor_link::ft_sensor_link_fixed_joint_lump__grasped_peg_collision_2 <-> target_plate::plate_link::plate_right_collision` |

## Wrench By State

| state | samples | max_abs_fz_n | max_force_norm_n | mean_xy_error_m |
| --- | ---: | ---: | ---: | ---: |
| APPROACH | 1190 | 133.329861 | 205.630221 | 0.002212 |
| DONE | 2556 | 128.954802 | 182.718042 | 0.409250 |
| INSERT | 2309 | 123.085613 | 206.668557 | 0.002086 |
| MOVING_TO_START | 4290 | 129.035041 | 211.137237 | 0.212705 |
| RETREAT | 2481 | 124.504805 | 200.945040 | 0.095468 |
| SEARCH | 20 | 109.065097 | 192.266852 | 0.002273 |
| UNKNOWN | 251 | 108.427390 | 181.476751 | 0.409438 |

Interpretation: this is an offline diagnostic over passive observer CSVs. It does not publish commands or change task safety gates.
