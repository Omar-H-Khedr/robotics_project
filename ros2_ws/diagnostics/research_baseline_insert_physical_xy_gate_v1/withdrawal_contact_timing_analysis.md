# Withdrawal Contact Timing Analysis

- input_dir: `diagnostics/research_baseline_insert_physical_xy_gate_v1`
- tracking_source: `trajectory_controller_state_samples.csv`
- insert_command_index: `2`
- positive_contact_samples: `1293`

## Command Windows

| index | label | receipt_stamp_s | next_command_stamp_s | duration_s | point_count | target_xyz_m |
| ---: | --- | ---: | ---: | ---: | ---: | --- |
| 0 | MOVING_TO_START | 2.707 | 50.423 | 40.000 | 20 | `0.520000, -0.200000, 0.885001` |
| 1 | APPROACH | 50.423 | 62.602 | 15.000 | 8 | `0.520000, -0.200000, 0.830000` |
| 2 | INSERT | 62.602 | 85.702 | 20.000 | 1 | `0.520000, -0.200000, 0.790000` |
| 3 | RETREAT_1 | 85.702 | none | 25.000 | 20 | `0.800000, 0.099056, 1.039872` |

## Positive Contact By Active Command

| command | samples | first_stamp_s | last_stamp_s | max_force_n | max_depth_m | max_xy_error_m | z_range_m | top_collision_pair |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- | --- |
| INSERT | 6 | 74.136 | 74.138 | 130.162091 | 0.002592 | 0.005656 | 0.807408..0.808441 | `lbr_iisy6_r1300::ft_sensor_link::ft_sensor_link_fixed_joint_lump__grasped_peg_collision_2 <-> target_plate::plate_link::plate_right_collision` |
| RETREAT_1 | 1287 | 85.773 | 89.662 | 589.942680 | 0.021856 | 0.007003 | 0.788144..0.810816 | `lbr_iisy6_r1300::ft_sensor_link::ft_sensor_link_fixed_joint_lump__grasped_peg_collision_2 <-> target_plate::plate_link::plate_left_collision` |

## Highest Force Events

| stamp_s | command | source | force_n | depth_m | xy_error_m | peg_tip_xyz_m | collision_pairs |
| ---: | --- | --- | ---: | ---: | ---: | --- | --- |
| 86.632 | RETREAT_1 | peg | 589.942680 | 0.017561 | 0.005906 | `0.514095, -0.199894, 0.792439` | `lbr_iisy6_r1300::ft_sensor_link::ft_sensor_link_fixed_joint_lump__grasped_peg_collision_2 <-> target_plate::plate_link::plate_left_collision` |
| 86.632 | RETREAT_1 | target | 589.942680 | 0.017561 | 0.005906 | `0.514095, -0.199894, 0.792439` | `lbr_iisy6_r1300::ft_sensor_link::ft_sensor_link_fixed_joint_lump__grasped_peg_collision_2 <-> target_plate::plate_link::plate_left_collision` |
| 88.548 | RETREAT_1 | peg | 530.912066 | 0.003704 | 0.006338 | `0.513871, -0.198386, 0.806296` | `lbr_iisy6_r1300::ft_sensor_link::ft_sensor_link_fixed_joint_lump__grasped_peg_collision_2 <-> target_plate::plate_link::plate_left_collision` |
| 88.548 | RETREAT_1 | target | 530.912066 | 0.003704 | 0.006338 | `0.513871, -0.198386, 0.806296` | `lbr_iisy6_r1300::ft_sensor_link::ft_sensor_link_fixed_joint_lump__grasped_peg_collision_2 <-> target_plate::plate_link::plate_left_collision` |
| 89.214 | RETREAT_1 | target | 511.200282 | 0.001638 | 0.005662 | `0.514466, -0.201200, 0.808362` | `lbr_iisy6_r1300::ft_sensor_link::ft_sensor_link_fixed_joint_lump__grasped_peg_collision_2 <-> target_plate::plate_link::plate_left_collision` |
| 89.216 | RETREAT_1 | peg | 511.200282 | 0.001638 | 0.005662 | `0.514466, -0.201200, 0.808362` | `lbr_iisy6_r1300::ft_sensor_link::ft_sensor_link_fixed_joint_lump__grasped_peg_collision_2 <-> target_plate::plate_link::plate_left_collision` |
| 86.979 | RETREAT_1 | peg | 506.072022 | 0.015320 | 0.005603 | `0.514462, -0.199151, 0.794680` | `lbr_iisy6_r1300::ft_sensor_link::ft_sensor_link_fixed_joint_lump__grasped_peg_collision_2 <-> target_plate::plate_link::plate_left_collision` |
| 86.979 | RETREAT_1 | target | 506.072022 | 0.015320 | 0.005603 | `0.514462, -0.199151, 0.794680` | `lbr_iisy6_r1300::ft_sensor_link::ft_sensor_link_fixed_joint_lump__grasped_peg_collision_2 <-> target_plate::plate_link::plate_left_collision` |
| 87.631 | RETREAT_1 | target | 495.737714 | 0.014616 | 0.005973 | `0.514587, -0.202524, 0.795384` | `lbr_iisy6_r1300::ft_sensor_link::ft_sensor_link_fixed_joint_lump__grasped_peg_collision_2 <-> target_plate::plate_link::plate_left_collision` |
| 87.632 | RETREAT_1 | peg | 495.737714 | 0.014616 | 0.005973 | `0.514587, -0.202524, 0.795384` | `lbr_iisy6_r1300::ft_sensor_link::ft_sensor_link_fixed_joint_lump__grasped_peg_collision_2 <-> target_plate::plate_link::plate_left_collision` |

Interpretation: this is an offline diagnostic. It uses recorded command, contact, and tracking CSVs only; it does not publish robot commands or change safety gates.
