# Withdrawal Contact Timing Analysis

- input_dir: `diagnostics/research_baseline_insert_sim_time_completion_v4`
- tracking_source: `trajectory_controller_state_samples.csv`
- insert_command_index: `2`
- positive_contact_samples: `41`

## Command Windows

| index | label | receipt_stamp_s | next_command_stamp_s | duration_s | point_count | target_xyz_m |
| ---: | --- | ---: | ---: | ---: | ---: | --- |
| 0 | MOVING_TO_START | 2.443 | 45.508 | 40.000 | 20 | `0.520000, -0.200000, 0.885001` |
| 1 | APPROACH | 45.508 | 57.608 | 15.000 | 8 | `0.520000, -0.200000, 0.830000` |
| 2 | INSERT | 57.608 | 80.719 | 20.000 | 1 | `0.520000, -0.200000, 0.790000` |
| 3 | RETREAT_1 | 80.719 | none | 25.000 | 20 | `0.800000, 0.099056, 1.039872` |

## Positive Contact By Active Command

| command | samples | first_stamp_s | last_stamp_s | max_force_n | max_depth_m | max_xy_error_m | z_range_m | top_collision_pair |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- | --- |
| RETREAT_1 | 41 | 81.273 | 84.831 | 249.593329 | 0.023308 | 0.006038 | 0.786692..0.806330 | `lbr_iisy6_r1300::ft_sensor_link::ft_sensor_link_fixed_joint_lump__grasped_peg_collision_2 <-> target_plate::plate_link::plate_right_collision` |

## Highest Force Events

| stamp_s | command | source | force_n | depth_m | xy_error_m | peg_tip_xyz_m | collision_pairs |
| ---: | --- | --- | ---: | ---: | ---: | --- | --- |
| 84.828 | RETREAT_1 | target | 249.593329 | 0.003927 | 0.005688 | `0.525527, -0.201343, 0.806073` | `lbr_iisy6_r1300::ft_sensor_link::ft_sensor_link_fixed_joint_lump__grasped_peg_collision_2 <-> target_plate::plate_link::plate_right_collision` |
| 84.828 | RETREAT_1 | peg | 249.593329 | 0.003927 | 0.005688 | `0.525527, -0.201343, 0.806073` | `lbr_iisy6_r1300::ft_sensor_link::ft_sensor_link_fixed_joint_lump__grasped_peg_collision_2 <-> target_plate::plate_link::plate_right_collision` |
| 84.349 | RETREAT_1 | peg | 150.706048 | 0.006498 | 0.005073 | `0.524916, -0.201255, 0.803502` | `lbr_iisy6_r1300::ft_sensor_link::ft_sensor_link_fixed_joint_lump__grasped_peg_collision_2 <-> target_plate::plate_link::plate_right_collision` |
| 84.349 | RETREAT_1 | target | 150.706048 | 0.006498 | 0.005073 | `0.524916, -0.201255, 0.803502` | `lbr_iisy6_r1300::ft_sensor_link::ft_sensor_link_fixed_joint_lump__grasped_peg_collision_2 <-> target_plate::plate_link::plate_right_collision` |
| 84.708 | RETREAT_1 | peg | 49.507661 | 0.005117 | 0.005718 | `0.525537, -0.201427, 0.804883` | `lbr_iisy6_r1300::ft_sensor_link::ft_sensor_link_fixed_joint_lump__grasped_peg_collision_2 <-> target_plate::plate_link::plate_right_collision` |
| 84.708 | RETREAT_1 | target | 49.507661 | 0.005117 | 0.005718 | `0.525537, -0.201427, 0.804883` | `lbr_iisy6_r1300::ft_sensor_link::ft_sensor_link_fixed_joint_lump__grasped_peg_collision_2 <-> target_plate::plate_link::plate_right_collision` |
| 84.352 | RETREAT_1 | target | 38.588680 | 0.007227 | 0.006038 | `0.525673, -0.202067, 0.802773` | `lbr_iisy6_r1300::ft_sensor_link::ft_sensor_link_fixed_joint_lump__grasped_peg_collision_2 <-> target_plate::plate_link::plate_right_collision` |
| 84.352 | RETREAT_1 | peg | 38.588680 | 0.007227 | 0.006038 | `0.525673, -0.202067, 0.802773` | `lbr_iisy6_r1300::ft_sensor_link::ft_sensor_link_fixed_joint_lump__grasped_peg_collision_2 <-> target_plate::plate_link::plate_right_collision` |
| 81.273 | RETREAT_1 | peg | 9.205136 | 0.022622 | 0.005731 | `0.525580, -0.198697, 0.787378` | `lbr_iisy6_r1300::ft_sensor_link::ft_sensor_link_fixed_joint_lump__grasped_peg_collision_2 <-> target_plate::plate_link::plate_right_collision; hole_fixture::fixture_link::fixture_right_collision <-> lbr_iisy6_r1300::ft_sensor_link::ft_sensor_link_fixed_joint_lump__grasped_peg_collision_2` |
| 81.273 | RETREAT_1 | hole | 9.205136 | 0.022622 | 0.005731 | `0.525580, -0.198697, 0.787378` | `hole_fixture::fixture_link::fixture_right_collision <-> lbr_iisy6_r1300::ft_sensor_link::ft_sensor_link_fixed_joint_lump__grasped_peg_collision_2` |

Interpretation: this is an offline diagnostic. It uses recorded command, contact, and tracking CSVs only; it does not publish robot commands or change safety gates.
