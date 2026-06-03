# Withdrawal Contact Timing Analysis

- input_dir: `diagnostics/research_baseline_insert_sideload_abort_v1`
- tracking_source: `trajectory_controller_state_samples.csv`
- insert_command_index: `2`
- positive_contact_samples: `2`

## Command Windows

| index | label | receipt_stamp_s | next_command_stamp_s | duration_s | point_count | target_xyz_m |
| ---: | --- | ---: | ---: | ---: | ---: | --- |
| 0 | MOVING_TO_START | 2.608 | 42.901 | 40.000 | 20 | `0.520000, -0.200000, 0.885001` |
| 1 | APPROACH | 42.901 | 54.808 | 15.000 | 8 | `0.520000, -0.200000, 0.830000` |
| 2 | INSERT | 54.808 | 67.618 | 20.000 | 1 | `0.520000, -0.200000, 0.790000` |
| 3 | RETREAT_1 | 67.618 | none | 25.000 | 18 | `0.800000, 0.099056, 1.039872` |

## Positive Contact By Active Command

| command | samples | first_stamp_s | last_stamp_s | max_force_n | max_depth_m | max_xy_error_m | z_range_m | top_collision_pair |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- | --- |
| RETREAT_1 | 2 | 67.717 | 67.721 | 0.000000 | 0.000832 | 0.005272 | 0.809168..0.809942 | `lbr_iisy6_r1300::ft_sensor_link::ft_sensor_link_fixed_joint_lump__grasped_peg_collision_2 <-> target_plate::plate_link::plate_left_collision` |

## Highest Force Events

| stamp_s | command | source | force_n | depth_m | xy_error_m | peg_tip_xyz_m | collision_pairs |
| ---: | --- | --- | ---: | ---: | ---: | --- | --- |
| 67.717 | RETREAT_1 | target | 0.000000 | 0.000058 | 0.005272 | `0.514728, -0.200021, 0.809942` | `lbr_iisy6_r1300::ft_sensor_link::ft_sensor_link_fixed_joint_lump__grasped_peg_collision_2 <-> target_plate::plate_link::plate_left_collision` |
| 67.721 | RETREAT_1 | peg | 0.000000 | 0.000832 | 0.004526 | `0.515517, -0.200624, 0.809168` | `lbr_iisy6_r1300::ft_sensor_link::ft_sensor_link_fixed_joint_lump__grasped_peg_collision_2 <-> target_plate::plate_link::plate_left_collision` |

Interpretation: this is an offline diagnostic. It uses recorded command, contact, and tracking CSVs only; it does not publish robot commands or change safety gates.
