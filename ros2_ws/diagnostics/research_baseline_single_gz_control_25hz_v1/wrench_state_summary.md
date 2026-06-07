# Wrench State Summary

- wrench_topic: `/ft_sensor_wrench`
- state_topic: `/insertion_state`
- joint_state_topic: `/joint_states`
- samples: `7033`
- duration_s: `70.319`
- max_abs_fz_n: `126.993606`
- max_force_norm_n: `210.280851`

| State | Samples | Mean Fz N | P95 abs Fz N | Max abs Fz N | Mean norm N | Max norm N | Min XY m | Mean XY m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| APPROACH | 1188 | -5.723317 | 103.409638 | 126.531078 | 73.870627 | 210.280851 | 0.000093 | 0.002170 |
| MOVING_TO_START | 4448 | -5.504278 | 91.642159 | 126.993606 | 64.880438 | 207.712690 | 0.000116 | 0.209287 |
| SEARCH | 1164 | -7.038435 | 104.887328 | 126.618462 | 74.751618 | 197.837191 | 0.000084 | 0.002450 |
| UNKNOWN | 233 | -3.912757 | 76.101727 | 111.998104 | 31.582640 | 180.768546 | 0.404356 | 0.409451 |

This observer is passive. It does not publish commands or alter controller behavior.
