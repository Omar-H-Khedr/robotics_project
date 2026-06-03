# Wrench State Summary

- wrench_topic: `/ft_sensor_wrench`
- state_topic: `/insertion_state`
- joint_state_topic: `/joint_states`
- samples: `1844`
- duration_s: `18.429`
- max_abs_fz_n: `162.506598`
- max_force_norm_n: `246.502462`

| State | Samples | Mean Fz N | P95 abs Fz N | Max abs Fz N | Mean norm N | Max norm N | Min XY m | Mean XY m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| IDLE | 100 | 5.204101 | 135.362852 | 156.981365 | 98.657873 | 210.624821 | 0.393499 | 0.407901 |
| MOVING_TO_START | 1538 | -3.335426 | 138.972669 | 162.506598 | 93.356235 | 246.502462 | 0.250362 | 0.335391 |
| UNKNOWN | 206 | -5.712933 | 77.649851 | 153.379731 | 23.002692 | 235.495667 | 0.400001 | 0.409379 |

This observer is passive. It does not publish commands or alter controller behavior.
