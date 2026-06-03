# Insert / Retreat Contact Analysis

- input_dir: `diagnostics/research_baseline_staged_withdrawal_v1`
- tracking_source: `trajectory_controller_state_samples.csv`
- command_index: `2`
- command_receipt_stamp_s: `57.307`
- next_command_stamp_s: `80.422`
- insert_command_duration_s: `20.000`
- observed_window_s: `23.115`
- samples: `5779`
- max_abs_position_error_rad: `0.010497`
- p95_max_abs_position_error_rad: `0.008342`
- mean_rms_position_error_rad: `0.002887`

## Cartesian Peg Tip

- command_target_xyz_m: `0.520000, -0.200000, 0.790000`
- final_feedback_xyz_m: `0.521934, -0.200326, 0.789138`
- final_cartesian_error_xyz_m: `-0.001934, 0.000326, 0.000862`
- final_cartesian_error_norm_m: `0.002143`
- missing_descent_to_target_m: `-0.000862`
- hole_top_z_m: `0.810000`
- min_feedback_z_m: `0.785227`
- max_physical_depth_m: `0.024773`
- final_physical_depth_m: `0.020862`

## Contact By State

| state | first_stamp_s | samples | positive_samples | max_contact_force_n | top_collision_pair |
| --- | ---: | ---: | ---: | ---: | --- |
| RETREAT | 82.187 | 307 | 307 | 486.746287 | `lbr_iisy6_r1300::ft_sensor_link::ft_sensor_link_fixed_joint_lump__grasped_peg_collision_2 <-> target_plate::plate_link::plate_left_collision` |

## Wrench By State

| state | samples | max_abs_fz_n | max_force_norm_n | mean_xy_error_m |
| --- | ---: | ---: | ---: | ---: |
| APPROACH | 1190 | 127.774326 | 207.064748 | 0.002019 |
| DONE | 1494 | 122.437815 | 181.865448 | 0.408970 |
| INSERT | 2310 | 128.431127 | 204.840394 | 0.002190 |
| MOVING_TO_START | 4240 | 124.248528 | 208.308440 | 0.214244 |
| RETREAT | 4360 | 153.007764 | 285.766425 | 0.107226 |
| SEARCH | 20 | 85.058676 | 169.480457 | 0.001854 |
| UNKNOWN | 271 | 110.433446 | 184.085954 | 0.409545 |

Interpretation: this is an offline diagnostic over passive observer CSVs. It does not publish commands or change task safety gates.
