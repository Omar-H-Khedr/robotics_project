# Contact State Summary

- state_topic: `/insertion_state`
- contact_topics: `peg:/gazebo/contacts/peg, hole:/gazebo/contacts/hole, target:/gazebo/contacts/target`
- samples: `41`
- positive_contact_samples: `41`
- duration_s: `3.558`
- max_contact_force_n: `249.593329`

| State | Source | Samples | Positive Samples | Max Contacts | Mean Contacts | Max Force N |
|---|---|---:|---:|---:|---:|---:|
| RETREAT | hole | 7 | 7 | 1 | 1.000000 | 9.205136 |
| RETREAT | peg | 17 | 17 | 2 | 1.411765 | 249.593329 |
| RETREAT | target | 17 | 17 | 1 | 1.000000 | 249.593329 |

## Collision Pairs

| State | Source | Collision Pair | Samples | Max Force N |
|---|---|---|---:|---:|
| RETREAT | hole | `hole_fixture::fixture_link::fixture_right_collision <-> lbr_iisy6_r1300::ft_sensor_link::ft_sensor_link_fixed_joint_lump__grasped_peg_collision_2` | 7 | 9.205136 |
| RETREAT | peg | `hole_fixture::fixture_link::fixture_right_collision <-> lbr_iisy6_r1300::ft_sensor_link::ft_sensor_link_fixed_joint_lump__grasped_peg_collision_2` | 7 | 9.205136 |
| RETREAT | peg | `lbr_iisy6_r1300::ft_sensor_link::ft_sensor_link_fixed_joint_lump__grasped_peg_collision_2 <-> target_plate::plate_link::plate_right_collision` | 17 | 249.593329 |
| RETREAT | target | `lbr_iisy6_r1300::ft_sensor_link::ft_sensor_link_fixed_joint_lump__grasped_peg_collision_2 <-> target_plate::plate_link::plate_right_collision` | 17 | 249.593329 |

This observer is passive. It does not publish commands or alter controller behavior.
