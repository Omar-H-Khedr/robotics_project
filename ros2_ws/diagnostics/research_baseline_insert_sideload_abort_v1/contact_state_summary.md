# Contact State Summary

- state_topic: `/insertion_state`
- contact_topics: `peg:/gazebo/contacts/peg, hole:/gazebo/contacts/hole, target:/gazebo/contacts/target`
- samples: `2`
- positive_contact_samples: `2`
- duration_s: `0.004`
- max_contact_force_n: `0.000000`

| State | Source | Samples | Positive Samples | Max Contacts | Mean Contacts | Max Force N |
|---|---|---:|---:|---:|---:|---:|
| ABORT | peg | 1 | 1 | 1 | 1.000000 | 0.000000 |
| ABORT | target | 1 | 1 | 1 | 1.000000 | 0.000000 |

## Collision Pairs

| State | Source | Collision Pair | Samples | Max Force N |
|---|---|---|---:|---:|
| ABORT | peg | `lbr_iisy6_r1300::ft_sensor_link::ft_sensor_link_fixed_joint_lump__grasped_peg_collision_2 <-> target_plate::plate_link::plate_left_collision` | 1 | 0.000000 |
| ABORT | target | `lbr_iisy6_r1300::ft_sensor_link::ft_sensor_link_fixed_joint_lump__grasped_peg_collision_2 <-> target_plate::plate_link::plate_left_collision` | 1 | 0.000000 |

This observer is passive. It does not publish commands or alter controller behavior.
