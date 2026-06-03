# Contact State Summary

- state_topic: `/insertion_state`
- contact_topics: `peg:/gazebo/contacts/peg, hole:/gazebo/contacts/hole, target:/gazebo/contacts/target`
- samples: `26`
- positive_contact_samples: `26`
- duration_s: `0.758`
- max_contact_force_n: `522.781110`

| State | Source | Samples | Positive Samples | Max Contacts | Mean Contacts | Max Force N |
|---|---|---:|---:|---:|---:|---:|
| ABORT | peg | 13 | 13 | 1 | 1.000000 | 522.781110 |
| ABORT | target | 13 | 13 | 1 | 1.000000 | 522.781110 |

## Collision Pairs

| State | Source | Collision Pair | Samples | Max Force N |
|---|---|---|---:|---:|
| ABORT | peg | `lbr_iisy6_r1300::ft_sensor_link::ft_sensor_link_fixed_joint_lump__grasped_peg_collision_2 <-> target_plate::plate_link::plate_right_collision` | 13 | 522.781110 |
| ABORT | target | `lbr_iisy6_r1300::ft_sensor_link::ft_sensor_link_fixed_joint_lump__grasped_peg_collision_2 <-> target_plate::plate_link::plate_right_collision` | 13 | 522.781110 |

This observer is passive. It does not publish commands or alter controller behavior.
