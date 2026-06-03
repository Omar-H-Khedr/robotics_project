# Contact State Summary

- state_topic: `/insertion_state`
- contact_topics: `peg:/gazebo/contacts/peg, hole:/gazebo/contacts/hole, target:/gazebo/contacts/target`
- samples: `1293`
- positive_contact_samples: `1293`
- duration_s: `15.526`
- max_contact_force_n: `589.942680`

| State | Source | Samples | Positive Samples | Max Contacts | Mean Contacts | Max Force N |
|---|---|---:|---:|---:|---:|---:|
| INSERT | peg | 3 | 3 | 1 | 1.000000 | 130.162091 |
| INSERT | target | 3 | 3 | 1 | 1.000000 | 130.162091 |
| RETREAT | hole | 29 | 29 | 1 | 1.000000 | 445.488770 |
| RETREAT | peg | 629 | 629 | 2 | 1.046105 | 589.942680 |
| RETREAT | target | 629 | 629 | 1 | 1.000000 | 589.942680 |

## Collision Pairs

| State | Source | Collision Pair | Samples | Max Force N |
|---|---|---|---:|---:|
| INSERT | peg | `lbr_iisy6_r1300::ft_sensor_link::ft_sensor_link_fixed_joint_lump__grasped_peg_collision_2 <-> target_plate::plate_link::plate_right_collision` | 3 | 130.162091 |
| INSERT | target | `lbr_iisy6_r1300::ft_sensor_link::ft_sensor_link_fixed_joint_lump__grasped_peg_collision_2 <-> target_plate::plate_link::plate_right_collision` | 3 | 130.162091 |
| RETREAT | hole | `hole_fixture::fixture_link::fixture_left_collision <-> lbr_iisy6_r1300::ft_sensor_link::ft_sensor_link_fixed_joint_lump__grasped_peg_collision_2` | 29 | 445.488770 |
| RETREAT | peg | `hole_fixture::fixture_link::fixture_left_collision <-> lbr_iisy6_r1300::ft_sensor_link::ft_sensor_link_fixed_joint_lump__grasped_peg_collision_2` | 29 | 445.488770 |
| RETREAT | peg | `lbr_iisy6_r1300::ft_sensor_link::ft_sensor_link_fixed_joint_lump__grasped_peg_collision_2 <-> target_plate::plate_link::plate_left_collision` | 629 | 589.942680 |
| RETREAT | target | `lbr_iisy6_r1300::ft_sensor_link::ft_sensor_link_fixed_joint_lump__grasped_peg_collision_2 <-> target_plate::plate_link::plate_left_collision` | 629 | 589.942680 |

This observer is passive. It does not publish commands or alter controller behavior.
