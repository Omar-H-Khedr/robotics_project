# Contact State Summary

- state_topic: `/insertion_state`
- contact_topics: `peg:/gazebo/contacts/peg, hole:/gazebo/contacts/hole, target:/gazebo/contacts/target`
- samples: `61629`
- positive_contact_samples: `61629`
- duration_s: `69.881`
- max_contact_force_n: `6270.966943`

| State | Source | Samples | Positive Samples | Max Contacts | Mean Contacts | Max Force N |
|---|---|---:|---:|---:|---:|---:|
| ABORT | target | 3460 | 3460 | 3 | 1.041040 | 5193.796217 |
| APPROACH | target | 40080 | 40080 | 1 | 1.000000 | 6270.966943 |
| MOVING_TO_START | target | 18089 | 18089 | 3 | 1.033667 | 4784.826998 |

This observer is passive. It does not publish commands or alter controller behavior.
