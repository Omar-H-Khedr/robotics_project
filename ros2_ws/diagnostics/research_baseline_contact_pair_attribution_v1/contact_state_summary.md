# Contact State Summary

- state_topic: `/insertion_state`
- contact_topics: `peg:/gazebo/contacts/peg, hole:/gazebo/contacts/hole, target:/gazebo/contacts/target`
- samples: `726`
- positive_contact_samples: `726`
- duration_s: `4.851`
- max_contact_force_n: `8886.018914`

| State | Source | Samples | Positive Samples | Max Contacts | Mean Contacts | Max Force N |
|---|---|---:|---:|---:|---:|---:|
| ABORT | target | 496 | 496 | 1 | 1.000000 | 8886.018914 |
| MOVING_TO_START | target | 230 | 230 | 1 | 1.000000 | 6451.232097 |

## Collision Pairs

| State | Source | Collision Pair | Samples | Max Force N |
|---|---|---|---:|---:|
| ABORT | target | `lbr_iisy6_r1300::link_5::link_5_collision <-> target_plate::plate_link::target_plate_collision` | 496 | 8886.018914 |
| MOVING_TO_START | target | `lbr_iisy6_r1300::link_5::link_5_collision <-> target_plate::plate_link::target_plate_collision` | 230 | 6451.232097 |

This observer is passive. It does not publish commands or alter controller behavior.
