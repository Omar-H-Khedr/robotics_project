# Withdrawal Contact Timing Analysis

- input_dir: `diagnostics/research_baseline_staged_withdrawal_v1`
- tracking_source: `trajectory_controller_state_samples.csv`
- insert_command_index: `2`
- positive_contact_samples: `307`

## Command Windows

| index | label | receipt_stamp_s | next_command_stamp_s | duration_s | point_count | target_xyz_m |
| ---: | --- | ---: | ---: | ---: | ---: | --- |
| 0 | MOVING_TO_START | 2.608 | 45.211 | 40.000 | 20 | `0.520000, -0.200000, 0.885001` |
| 1 | APPROACH | 45.211 | 57.307 | 15.000 | 8 | `0.520000, -0.200000, 0.830000` |
| 2 | INSERT | 57.307 | 80.422 | 20.000 | 1 | `0.520000, -0.200000, 0.790000` |
| 3 | RETREAT_1 | 80.422 | 99.702 | 20.857 | 11 | `0.517257, -0.200095, 0.885000` |
| 4 | RETREAT_2 | 99.702 | none | 25.000 | 10 | `0.800000, 0.099056, 1.039872` |

## Positive Contact By Active Command

| command | samples | first_stamp_s | last_stamp_s | max_force_n | max_depth_m | max_xy_error_m | z_range_m | top_collision_pair |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- | --- |
| RETREAT_1 | 307 | 82.187 | 85.966 | 486.746287 | 0.020598 | 0.007042 | 0.789402..0.809327 | `lbr_iisy6_r1300::ft_sensor_link::ft_sensor_link_fixed_joint_lump__grasped_peg_collision_2 <-> target_plate::plate_link::plate_left_collision` |

## Highest Force Events

| stamp_s | command | source | force_n | depth_m | xy_error_m | peg_tip_xyz_m | collision_pairs |
| ---: | --- | --- | ---: | ---: | ---: | --- | --- |
| 82.399 | RETREAT_1 | target | 486.746287 | 0.019732 | 0.005979 | `0.514225, -0.201547, 0.790268` | `lbr_iisy6_r1300::ft_sensor_link::ft_sensor_link_fixed_joint_lump__grasped_peg_collision_2 <-> target_plate::plate_link::plate_left_collision` |
| 82.399 | RETREAT_1 | peg | 486.746287 | 0.019732 | 0.005979 | `0.514225, -0.201547, 0.790268` | `lbr_iisy6_r1300::ft_sensor_link::ft_sensor_link_fixed_joint_lump__grasped_peg_collision_2 <-> target_plate::plate_link::plate_left_collision` |
| 82.538 | RETREAT_1 | peg | 443.118724 | 0.018516 | 0.005743 | `0.514408, -0.201304, 0.791484` | `lbr_iisy6_r1300::ft_sensor_link::ft_sensor_link_fixed_joint_lump__grasped_peg_collision_2 <-> target_plate::plate_link::plate_left_collision` |
| 82.538 | RETREAT_1 | target | 443.118724 | 0.018516 | 0.005743 | `0.514408, -0.201304, 0.791484` | `lbr_iisy6_r1300::ft_sensor_link::ft_sensor_link_fixed_joint_lump__grasped_peg_collision_2 <-> target_plate::plate_link::plate_left_collision` |
| 85.911 | RETREAT_1 | target | 365.800830 | 0.000751 | 0.005569 | `0.514447, -0.199582, 0.809249` | `lbr_iisy6_r1300::ft_sensor_link::ft_sensor_link_fixed_joint_lump__grasped_peg_collision_2 <-> target_plate::plate_link::plate_left_collision` |
| 85.912 | RETREAT_1 | peg | 365.800830 | 0.000751 | 0.005569 | `0.514447, -0.199582, 0.809249` | `lbr_iisy6_r1300::ft_sensor_link::ft_sensor_link_fixed_joint_lump__grasped_peg_collision_2 <-> target_plate::plate_link::plate_left_collision` |
| 83.207 | RETREAT_1 | peg | 355.593595 | 0.013077 | 0.005949 | `0.514126, -0.199057, 0.796923` | `lbr_iisy6_r1300::ft_sensor_link::ft_sensor_link_fixed_joint_lump__grasped_peg_collision_2 <-> target_plate::plate_link::plate_left_collision` |
| 83.207 | RETREAT_1 | target | 355.593595 | 0.013077 | 0.005949 | `0.514126, -0.199057, 0.796923` | `lbr_iisy6_r1300::ft_sensor_link::ft_sensor_link_fixed_joint_lump__grasped_peg_collision_2 <-> target_plate::plate_link::plate_left_collision` |
| 83.328 | RETREAT_1 | peg | 348.429292 | 0.012513 | 0.005773 | `0.514337, -0.198880, 0.797487` | `lbr_iisy6_r1300::ft_sensor_link::ft_sensor_link_fixed_joint_lump__grasped_peg_collision_2 <-> target_plate::plate_link::plate_left_collision` |
| 83.328 | RETREAT_1 | target | 348.429292 | 0.012513 | 0.005773 | `0.514337, -0.198880, 0.797487` | `lbr_iisy6_r1300::ft_sensor_link::ft_sensor_link_fixed_joint_lump__grasped_peg_collision_2 <-> target_plate::plate_link::plate_left_collision` |

Interpretation: this is an offline diagnostic. It uses recorded command, contact, and tracking CSVs only; it does not publish robot commands or change safety gates.
