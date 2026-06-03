# Contact State Summary

- state_topic: `/insertion_state`
- contact_topics: `peg:/gazebo/contacts/peg, hole:/gazebo/contacts/hole, target:/gazebo/contacts/target`
- samples: `7776`
- positive_contact_samples: `7776`
- duration_s: `4.961`
- max_contact_force_n: `1970.434828`

| State | Source | Samples | Positive Samples | Max Contacts | Mean Contacts | Max Force N |
|---|---|---:|---:|---:|---:|---:|
| RETREAT | peg | 3161 | 3161 | 2 | 1.177475 | 1587.131336 |
| RETREAT | target | 4615 | 4615 | 3 | 1.599567 | 1970.434828 |

## Collision Pairs

| State | Source | Collision Pair | Samples | Max Force N |
|---|---|---|---:|---:|
| RETREAT | peg | `lbr_iisy6_r1300::ft_sensor_link::ft_sensor_link_fixed_joint_lump__grasped_peg_collision_2 <-> target_plate::plate_link::plate_rear_collision` | 3 | 404.994130 |
| RETREAT | peg | `lbr_iisy6_r1300::ft_sensor_link::ft_sensor_link_fixed_joint_lump__grasped_peg_collision_2 <-> target_plate::plate_link::plate_right_collision` | 647 | 1587.131336 |
| RETREAT | peg | `lbr_iisy6_r1300::ft_sensor_link::ft_sensor_link_fixed_joint_lump__grasped_peg_collision_2 <-> target_plate::plate_link::target_plate_collision` | 3072 | 1587.131336 |
| RETREAT | target | `lbr_iisy6_r1300::ft_sensor_link::ft_sensor_link_fixed_joint_lump__grasped_peg_collision_2 <-> target_plate::plate_link::plate_rear_collision` | 3 | 404.994130 |
| RETREAT | target | `lbr_iisy6_r1300::ft_sensor_link::ft_sensor_link_fixed_joint_lump__grasped_peg_collision_2 <-> target_plate::plate_link::plate_right_collision` | 647 | 1970.434828 |
| RETREAT | target | `lbr_iisy6_r1300::ft_sensor_link::ft_sensor_link_fixed_joint_lump__grasped_peg_collision_2 <-> target_plate::plate_link::target_plate_collision` | 3072 | 1970.434828 |
| RETREAT | target | `lbr_iisy6_r1300::ft_sensor_link::ft_sensor_link_fixed_joint_lump__gripper_right_finger_collision_4 <-> target_plate::plate_link::target_plate_collision` | 3660 | 1970.434828 |

This observer is passive. It does not publish commands or alter controller behavior.
