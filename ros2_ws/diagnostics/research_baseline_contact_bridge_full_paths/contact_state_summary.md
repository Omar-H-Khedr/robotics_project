# Contact State Summary

- state_topic: `/insertion_state`
- contact_topics: `peg:/gazebo/contacts/peg, hole:/gazebo/contacts/hole, target:/gazebo/contacts/target`
- samples: `50`
- positive_contact_samples: `50`
- duration_s: `6.248`
- max_contact_force_n: `3440.570902`

| State | Source | Samples | Positive Samples | Max Contacts | Mean Contacts | Max Force N |
|---|---|---:|---:|---:|---:|---:|
| ABORT | peg | 15 | 15 | 1 | 1.000000 | 3440.570902 |
| ABORT | target | 15 | 15 | 1 | 1.000000 | 3440.570902 |
| MOVING_TO_START | peg | 10 | 10 | 1 | 1.000000 | 2427.307742 |
| MOVING_TO_START | target | 10 | 10 | 1 | 1.000000 | 2427.307742 |

This observer is passive. It does not publish commands or alter controller behavior.
