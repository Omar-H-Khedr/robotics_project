# Contact State Summary

- state_topic: `/insertion_state`
- contact_topics: `peg:/gazebo/contacts/peg, hole:/gazebo/contacts/hole, target:/gazebo/contacts/target`
- samples: `4`
- positive_contact_samples: `4`
- duration_s: `0.002`
- max_contact_force_n: `36.335073`

| State | Source | Samples | Positive Samples | Max Contacts | Mean Contacts | Max Force N |
|---|---|---:|---:|---:|---:|---:|
| RETREAT | peg | 2 | 2 | 1 | 1.000000 | 36.335073 |
| RETREAT | target | 2 | 2 | 1 | 1.000000 | 36.335073 |

## Collision Pairs

| State | Source | Collision Pair | Samples | Max Force N |
|---|---|---|---:|---:|
| RETREAT | peg | `lbr_iisy6_r1300::ft_sensor_link::ft_sensor_link_fixed_joint_lump__grasped_peg_collision_2 <-> target_plate::plate_link::plate_rear_collision` | 2 | 36.335073 |
| RETREAT | target | `lbr_iisy6_r1300::ft_sensor_link::ft_sensor_link_fixed_joint_lump__grasped_peg_collision_2 <-> target_plate::plate_link::plate_rear_collision` | 2 | 36.335073 |

This observer is passive. It does not publish commands or alter controller behavior.
