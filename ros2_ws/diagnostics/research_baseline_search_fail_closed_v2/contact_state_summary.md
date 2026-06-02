# Contact State Summary

- state_topic: `/insertion_state`
- contact_topics: `peg:/gazebo/contacts/peg, hole:/gazebo/contacts/hole, target:/gazebo/contacts/target`
- samples: `62058`
- positive_contact_samples: `62058`
- duration_s: `69.633`
- max_contact_force_n: `6753.161455`

| State | Source | Samples | Positive Samples | Max Contacts | Mean Contacts | Max Force N |
|---|---|---:|---:|---:|---:|---:|
| ABORT | target | 3053 | 3053 | 3 | 1.056993 | 6753.161455 |
| APPROACH | target | 41078 | 41078 | 1 | 1.000000 | 5789.311900 |
| MOVING_TO_START | target | 17927 | 17927 | 3 | 1.037820 | 3801.184847 |

This observer is passive. It does not publish commands or alter controller behavior.
