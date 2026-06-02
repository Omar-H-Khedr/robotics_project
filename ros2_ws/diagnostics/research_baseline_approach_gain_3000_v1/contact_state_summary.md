# Contact State Summary

- state_topic: `/insertion_state`
- contact_topics: `peg:/gazebo/contacts/peg, hole:/gazebo/contacts/hole, target:/gazebo/contacts/target`
- samples: `60673`
- positive_contact_samples: `60673`
- duration_s: `67.558`
- max_contact_force_n: `5698.403009`

| State | Source | Samples | Positive Samples | Max Contacts | Mean Contacts | Max Force N |
|---|---|---:|---:|---:|---:|---:|
| ABORT | target | 3097 | 3097 | 3 | 1.035518 | 5123.219904 |
| APPROACH | target | 41037 | 41037 | 1 | 1.000000 | 5698.403009 |
| MOVING_TO_START | target | 16539 | 16539 | 3 | 1.035613 | 3494.684709 |

This observer is passive. It does not publish commands or alter controller behavior.
