# Wrench State Summary

- wrench_topic: `/ft_sensor_wrench`
- state_topic: `/insertion_state`
- joint_state_topic: `/joint_states`
- samples: `11534`
- duration_s: `115.328`
- max_abs_fz_n: `842.846787`
- max_force_norm_n: `890.272134`

| State | Samples | Mean Fz N | P95 abs Fz N | Max abs Fz N | Mean norm N | Max norm N | Min XY m | Mean XY m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| ABORT | 1462 | -7.327518 | 136.136566 | 842.846787 | 102.772823 | 890.272134 | 0.015393 | 0.264583 |
| APPROACH | 4409 | -7.131134 | 89.571275 | 425.002922 | 82.044226 | 509.865712 | 0.000235 | 0.015084 |
| DONE | 795 | -4.322526 | 137.314864 | 159.407839 | 111.097141 | 325.776325 | 0.494969 | 0.514578 |
| MOVING_TO_START | 4590 | -5.939045 | 132.259322 | 499.257032 | 96.618817 | 503.995792 | 0.001175 | 0.265701 |
| UNKNOWN | 278 | -2.579423 | 128.058904 | 156.828657 | 67.942548 | 328.190428 | 0.494138 | 0.516171 |

This observer is passive. It does not publish commands or alter controller behavior.
