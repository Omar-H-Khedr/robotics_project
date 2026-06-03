# Insert / Retreat Contact Analysis

- input_dir: `diagnostics/research_baseline_retreat_clearance_lift_v1`
- tracking_source: `trajectory_controller_state_samples.csv`
- command_index: `2`
- command_receipt_stamp_s: `60.054`
- next_command_stamp_s: `70.621`
- insert_command_duration_s: `20.000`
- observed_window_s: `10.567`
- samples: `2641`
- max_abs_position_error_rad: `0.010382`
- p95_max_abs_position_error_rad: `0.008310`
- mean_rms_position_error_rad: `0.002885`

## Cartesian Peg Tip

- command_target_xyz_m: `0.520004, -0.200001, 0.790008`
- final_feedback_xyz_m: `0.518882, -0.198642, 0.815626`
- final_cartesian_error_xyz_m: `0.001122, -0.001359, -0.025618`
- final_cartesian_error_norm_m: `0.025679`
- missing_descent_to_target_m: `0.025618`
- hole_top_z_m: `0.810000`
- min_feedback_z_m: `0.809164`
- max_physical_depth_m: `0.000836`
- final_physical_depth_m: `0.000000`

## Contact By State

| state | first_stamp_s | samples | positive_samples | max_contact_force_n | top_collision_pair |
| --- | ---: | ---: | ---: | ---: | --- |
| RETREAT | 72.194 | 4 | 4 | 36.335073 | `lbr_iisy6_r1300::ft_sensor_link::ft_sensor_link_fixed_joint_lump__grasped_peg_collision_2 <-> target_plate::plate_link::plate_rear_collision` |

## Wrench By State

| state | samples | max_abs_fz_n | max_force_norm_n | mean_xy_error_m |
| --- | ---: | ---: | ---: | ---: |
| APPROACH | 1188 | 125.802257 | 198.884872 | 0.002141 |
| DONE | 1515 | 127.133583 | 181.336821 | 0.409109 |
| INSERT | 1059 | 124.064036 | 202.630104 | 0.002599 |
| MOVING_TO_START | 4532 | 127.919570 | 208.495440 | 0.199719 |
| RETREAT | 2462 | 128.114936 | 200.880897 | 0.105161 |
| SEARCH | 5 | 74.433305 | 128.896468 | 0.001753 |
| UNKNOWN | 275 | 116.648061 | 167.972038 | 0.409549 |

Interpretation: this is an offline diagnostic over passive observer CSVs. It does not publish commands or change task safety gates.
