# Wrench State Summary

- wrench_topic: `/ft_sensor_wrench`
- state_topic: `/insertion_state`
- joint_state_topic: `/joint_states`
- samples: `8916`
- duration_s: `89.149`
- max_abs_fz_n: `130.294881`
- max_force_norm_n: `206.332843`

| State | Samples | Mean Fz N | P95 abs Fz N | Max abs Fz N | Mean norm N | Max norm N | Min XY m | Mean XY m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| APPROACH | 1168 | -10.248290 | 105.265191 | 124.779537 | 74.444462 | 203.832354 | 0.000041 | 0.002127 |
| MOVING_TO_START | 4063 | -6.813036 | 92.042846 | 126.671128 | 65.197572 | 201.571541 | 0.000100 | 0.224278 |
| SEARCH | 3403 | -6.884191 | 103.973754 | 130.294881 | 74.483552 | 206.332843 | 0.000069 | 0.002296 |
| UNKNOWN | 282 | -5.658942 | 85.375781 | 108.788521 | 38.456481 | 167.395549 | 0.404730 | 0.409530 |

This observer is passive. It does not publish commands or alter controller behavior.
