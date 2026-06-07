# Wrench State Summary

- wrench_topic: `/ft_sensor_wrench`
- state_topic: `/insertion_state`
- joint_state_topic: `/joint_states`
- samples: `10491`
- duration_s: `104.899`
- max_abs_fz_n: `100.745873`
- max_force_norm_n: `169.801178`

| State | Samples | Mean Fz N | P95 abs Fz N | Max abs Fz N | Mean norm N | Max norm N | Min XY m | Mean XY m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| ABORT | 546 | -6.842198 | 78.391741 | 98.293781 | 60.036957 | 165.956834 | 0.000099 | 0.001888 |
| APPROACH | 1172 | -6.515931 | 77.163136 | 100.428697 | 60.889516 | 168.404371 | 0.000020 | 0.001197 |
| MOVING_TO_START | 3992 | -5.728505 | 63.052448 | 100.745873 | 52.360797 | 168.260005 | 0.000059 | 0.225787 |
| SEARCH | 4500 | -7.117339 | 77.517579 | 99.001837 | 59.735228 | 169.801178 | 0.000029 | 0.001458 |
| UNKNOWN | 281 | -1.500488 | 61.559428 | 88.343285 | 32.462523 | 139.921301 | 0.406737 | 0.409531 |

This observer is passive. It does not publish commands or alter controller behavior.
