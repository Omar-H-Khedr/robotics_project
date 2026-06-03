# Wrench State Summary

- wrench_topic: `/ft_sensor_wrench`
- state_topic: `/insertion_state`
- joint_state_topic: `/joint_states`
- samples: `20213`
- duration_s: `202.121`
- max_abs_fz_n: `128.868318`
- max_force_norm_n: `208.150996`

| State | Samples | Mean Fz N | P95 abs Fz N | Max abs Fz N | Mean norm N | Max norm N | Min XY m | Mean XY m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| ABORT | 2470 | -7.101483 | 99.878340 | 126.335714 | 69.436093 | 208.028187 | 0.000054 | 0.112289 |
| APPROACH | 1180 | -5.708976 | 104.610874 | 128.868318 | 75.680102 | 205.623997 | 0.000079 | 0.002202 |
| DONE | 7042 | -3.988691 | 92.476756 | 125.745970 | 62.450921 | 185.969150 | 0.397256 | 0.409270 |
| MOVING_TO_START | 4740 | -5.958148 | 93.563719 | 127.864252 | 66.114103 | 208.150996 | 0.000013 | 0.190308 |
| SEARCH | 4500 | -7.950713 | 103.920267 | 127.086591 | 73.941813 | 206.551494 | 0.000005 | 0.002317 |
| UNKNOWN | 281 | -7.288995 | 87.656700 | 109.970368 | 39.077830 | 184.821486 | 0.404996 | 0.409310 |

This observer is passive. It does not publish commands or alter controller behavior.
