# Wrench State Summary

- wrench_topic: `/ft_sensor_wrench`
- state_topic: `/insertion_state`
- joint_state_topic: `/joint_states`
- samples: `8402`
- duration_s: `84.010`
- max_abs_fz_n: `98.205519`
- max_force_norm_n: `169.397789`

| State | Samples | Mean Fz N | P95 abs Fz N | Max abs Fz N | Mean norm N | Max norm N | Min XY m | Mean XY m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| ABORT | 2453 | -7.277843 | 73.966092 | 96.287751 | 77.029858 | 167.471100 | 0.000340 | 0.125951 |
| APPROACH | 1160 | -8.269461 | 77.563107 | 98.205519 | 83.010946 | 169.397789 | 0.000001 | 0.000499 |
| DONE | 25 | -6.256458 | 49.153532 | 49.666310 | 67.448129 | 133.519483 | 0.398223 | 0.400795 |
| IDLE | 68 | -3.025406 | 65.820968 | 76.518580 | 60.415136 | 134.217486 | 0.408443 | 0.409560 |
| INSERT | 511 | -8.264657 | 76.560304 | 95.893364 | 79.860785 | 162.921601 | 0.000015 | 0.000623 |
| MOVING_TO_START | 4052 | -5.224027 | 67.368272 | 95.737623 | 71.981675 | 166.709156 | 0.000360 | 0.230903 |
| UNKNOWN | 133 | -2.126317 | 62.152783 | 71.621008 | 35.587441 | 138.729375 | 0.408568 | 0.409625 |

This observer is passive. It does not publish commands or alter controller behavior.
