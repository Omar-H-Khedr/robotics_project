# Wrench State Summary

- wrench_topic: `/ft_sensor_wrench`
- state_topic: `/insertion_state`
- joint_state_topic: `/joint_states`
- samples: `8666`
- duration_s: `86.649`
- max_abs_fz_n: `127.233045`
- max_force_norm_n: `211.930944`

| State | Samples | Mean Fz N | P95 abs Fz N | Max abs Fz N | Mean norm N | Max norm N | Min XY m | Mean XY m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| APPROACH | 1172 | -5.008336 | 103.671203 | 126.748677 | 75.024845 | 211.930944 | 0.000134 | 0.002121 |
| MOVING_TO_START | 4036 | -6.237237 | 91.758213 | 127.233045 | 65.207110 | 205.081376 | 0.000406 | 0.225509 |
| SEARCH | 3181 | -8.170786 | 103.072481 | 126.373585 | 73.432907 | 203.010615 | 0.000052 | 0.002298 |
| UNKNOWN | 277 | -2.081625 | 85.877002 | 124.895918 | 35.848161 | 166.829091 | 0.404000 | 0.409473 |

This observer is passive. It does not publish commands or alter controller behavior.
