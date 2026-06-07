# Wrench State Summary

- wrench_topic: `/ft_sensor_wrench`
- state_topic: `/insertion_state`
- joint_state_topic: `/joint_states`
- samples: `8310`
- duration_s: `83.090`
- max_abs_fz_n: `102.212636`
- max_force_norm_n: `170.998336`

| State | Samples | Mean Fz N | P95 abs Fz N | Max abs Fz N | Mean norm N | Max norm N | Min XY m | Mean XY m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| ABORT | 2281 | -6.655570 | 69.726530 | 102.212636 | 57.196927 | 170.998336 | 0.000025 | 0.106786 |
| APPROACH | 1184 | -6.016389 | 75.425274 | 99.086482 | 60.805761 | 167.291688 | 0.000041 | 0.001268 |
| INSERT | 604 | -8.791821 | 75.833579 | 98.046340 | 61.397743 | 165.478247 | 0.000052 | 0.001188 |
| MOVING_TO_START | 3996 | -5.904236 | 64.252070 | 101.180897 | 53.485490 | 169.440271 | 0.000812 | 0.228456 |
| UNKNOWN | 245 | -3.249102 | 55.311316 | 83.131478 | 25.609682 | 137.130542 | 0.407920 | 0.409662 |

This observer is passive. It does not publish commands or alter controller behavior.
