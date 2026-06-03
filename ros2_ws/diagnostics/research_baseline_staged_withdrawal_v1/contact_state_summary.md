# Contact State Summary

- state_topic: `/insertion_state`
- contact_topics: `peg:/gazebo/contacts/peg, hole:/gazebo/contacts/hole, target:/gazebo/contacts/target`
- samples: `307`
- positive_contact_samples: `307`
- duration_s: `3.779`
- max_contact_force_n: `486.746287`

| State | Source | Samples | Positive Samples | Max Contacts | Mean Contacts | Max Force N |
|---|---|---:|---:|---:|---:|---:|
| RETREAT | hole | 7 | 7 | 1 | 1.000000 | 0.000000 |
| RETREAT | peg | 150 | 150 | 2 | 1.046667 | 486.746287 |
| RETREAT | target | 150 | 150 | 1 | 1.000000 | 486.746287 |

## Collision Pairs

| State | Source | Collision Pair | Samples | Max Force N |
|---|---|---|---:|---:|
| RETREAT | hole | `hole_fixture::fixture_link::fixture_left_collision <-> lbr_iisy6_r1300::ft_sensor_link::ft_sensor_link_fixed_joint_lump__grasped_peg_collision_2` | 7 | 0.000000 |
| RETREAT | peg | `hole_fixture::fixture_link::fixture_left_collision <-> lbr_iisy6_r1300::ft_sensor_link::ft_sensor_link_fixed_joint_lump__grasped_peg_collision_2` | 7 | 253.647191 |
| RETREAT | peg | `lbr_iisy6_r1300::ft_sensor_link::ft_sensor_link_fixed_joint_lump__grasped_peg_collision_2 <-> target_plate::plate_link::plate_left_collision` | 150 | 486.746287 |
| RETREAT | target | `lbr_iisy6_r1300::ft_sensor_link::ft_sensor_link_fixed_joint_lump__grasped_peg_collision_2 <-> target_plate::plate_link::plate_left_collision` | 150 | 486.746287 |

This observer is passive. It does not publish commands or alter controller behavior.
