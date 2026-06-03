# Insert / Retreat Contact Analysis

- input_dir: `diagnostics/research_baseline_insert_physical_xy_gate_v1`
- tracking_source: `trajectory_controller_state_samples.csv`
- command_index: `2`
- command_receipt_stamp_s: `62.602`
- next_command_stamp_s: `85.702`
- insert_command_duration_s: `20.000`
- observed_window_s: `23.100`
- samples: `5775`
- max_abs_position_error_rad: `0.010498`
- p95_max_abs_position_error_rad: `0.008381`
- mean_rms_position_error_rad: `0.002915`

## Cartesian Peg Tip

- command_target_xyz_m: `0.520000, -0.200000, 0.790000`
- final_feedback_xyz_m: `0.515688, -0.200416, 0.792084`
- final_cartesian_error_xyz_m: `0.004312, 0.000416, -0.002084`
- final_cartesian_error_norm_m: `0.004807`
- missing_descent_to_target_m: `0.002084`
- hole_top_z_m: `0.810000`
- min_feedback_z_m: `0.785095`
- max_physical_depth_m: `0.024905`
- final_physical_depth_m: `0.017916`

## Contact By State

| state | first_stamp_s | samples | positive_samples | max_contact_force_n | top_collision_pair |
| --- | ---: | ---: | ---: | ---: | --- |
| INSERT | 74.136 | 6 | 6 | 130.162091 | `lbr_iisy6_r1300::ft_sensor_link::ft_sensor_link_fixed_joint_lump__grasped_peg_collision_2 <-> target_plate::plate_link::plate_right_collision` |
| RETREAT | 85.773 | 1287 | 1287 | 589.942680 | `lbr_iisy6_r1300::ft_sensor_link::ft_sensor_link_fixed_joint_lump__grasped_peg_collision_2 <-> target_plate::plate_link::plate_left_collision` |

## Wrench By State

| state | samples | max_abs_fz_n | max_force_norm_n | mean_xy_error_m |
| --- | ---: | ---: | ---: | ---: |
| APPROACH | 1200 | 127.503232 | 202.658047 | 0.002206 |
| DONE | 10525 | 126.880339 | 190.522725 | 0.409255 |
| INSERT | 2310 | 125.805336 | 207.218757 | 0.002176 |
| MOVING_TO_START | 4750 | 125.929109 | 206.635813 | 0.189748 |
| RETREAT | 2470 | 227.088310 | 234.958292 | 0.095483 |
| SEARCH | 20 | 107.656966 | 149.370426 | 0.002185 |
| UNKNOWN | 281 | 105.098569 | 167.529112 | 0.409493 |

Interpretation: this is an offline diagnostic over passive observer CSVs. It does not publish commands or change task safety gates.
