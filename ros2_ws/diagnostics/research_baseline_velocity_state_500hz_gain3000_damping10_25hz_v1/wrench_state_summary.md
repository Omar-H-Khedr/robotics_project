# Wrench State Summary

- wrench_topic: `/ft_sensor_wrench`
- state_topic: `/insertion_state`
- joint_state_topic: `/joint_states`
- samples: `10485`
- duration_s: `104.839`
- max_abs_fz_n: `96.967518`
- max_force_norm_n: `170.020007`

| State | Samples | Mean Fz N | P95 abs Fz N | Max abs Fz N | Mean norm N | Max norm N | Min XY m | Mean XY m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| APPROACH | 1156 | -6.149970 | 77.526588 | 95.280964 | 81.382332 | 170.020007 | 0.000026 | 0.000490 |
| DONE | 60 | -14.644526 | 72.393002 | 89.403102 | 64.866551 | 140.274268 | 0.398218 | 0.405988 |
| INSERT | 2532 | -7.637399 | 78.227428 | 95.597793 | 79.912492 | 167.240519 | 0.000013 | 0.000560 |
| MOVING_TO_START | 3996 | -6.595410 | 67.758208 | 96.967518 | 72.760100 | 166.720652 | 0.000194 | 0.228131 |
| RETREAT | 2468 | -6.066716 | 75.656464 | 93.822932 | 78.282186 | 167.047509 | 0.000013 | 0.093888 |
| UNKNOWN | 273 | -2.665870 | 55.847543 | 71.141390 | 30.573562 | 140.246210 | 0.408169 | 0.409655 |

This observer is passive. It does not publish commands or alter controller behavior.
