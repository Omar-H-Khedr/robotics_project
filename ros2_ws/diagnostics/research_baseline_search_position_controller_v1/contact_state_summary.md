# Contact State Summary

- state_topic: `/insertion_state`
- contact_topics: `peg:/gazebo/contacts/peg, hole:/gazebo/contacts/hole, target:/gazebo/contacts/target`
- samples: `6`
- positive_contact_samples: `6`
- duration_s: `0.002`
- max_contact_force_n: `162.466918`

| State | Source | Samples | Positive Samples | Max Contacts | Mean Contacts | Max Force N |
|---|---|---:|---:|---:|---:|---:|
| SEARCH | peg | 3 | 3 | 1 | 1.000000 | 162.466918 |
| SEARCH | target | 3 | 3 | 1 | 1.000000 | 162.466918 |

## Collision Pairs

| State | Source | Collision Pair | Samples | Max Force N |
|---|---|---|---:|---:|
| SEARCH | peg | `lbr_iisy6_r1300::ft_sensor_link::ft_sensor_link_fixed_joint_lump__grasped_peg_collision_2 <-> target_plate::plate_link::plate_right_collision` | 3 | 162.466918 |
| SEARCH | target | `lbr_iisy6_r1300::ft_sensor_link::ft_sensor_link_fixed_joint_lump__grasped_peg_collision_2 <-> target_plate::plate_link::plate_right_collision` | 3 | 162.466918 |

This observer is passive. It does not publish commands or alter controller behavior.
