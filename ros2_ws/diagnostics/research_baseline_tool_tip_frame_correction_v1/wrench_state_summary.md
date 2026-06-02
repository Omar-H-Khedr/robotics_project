# Wrench State Summary

- wrench_topic: `/ft_sensor_wrench`
- state_topic: `/insertion_state`
- joint_state_topic: `/joint_states`
- samples: `11775`
- duration_s: `117.739`
- max_abs_fz_n: `172.825699`
- max_force_norm_n: `270.818855`

| State | Samples | Mean Fz N | P95 abs Fz N | Max abs Fz N | Mean norm N | Max norm N | Min XY m | Mean XY m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| ABORT | 1452 | -10.067573 | 139.310141 | 170.038596 | 96.573580 | 265.133868 | 0.001313 | 0.190818 |
| DONE | 3904 | -3.339329 | 141.017489 | 162.634785 | 99.697662 | 266.323439 | 0.385365 | 0.407707 |
| MOVING_TO_START | 6138 | -6.084183 | 139.519792 | 172.825699 | 97.663810 | 270.818855 | 0.000037 | 0.149465 |
| UNKNOWN | 281 | 1.589728 | 135.364336 | 156.119255 | 61.876429 | 253.190300 | 0.392383 | 0.409234 |

This observer is passive. It does not publish commands or alter controller behavior.
