# Wrench State Summary

- wrench_topic: `/ft_sensor_wrench`
- state_topic: `/insertion_state`
- joint_state_topic: `/joint_states`
- samples: `20913`
- duration_s: `209.119`
- max_abs_fz_n: `130.793410`
- max_force_norm_n: `208.477234`

| State | Samples | Mean Fz N | P95 abs Fz N | Max abs Fz N | Mean norm N | Max norm N | Min XY m | Mean XY m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| ABORT | 2460 | -7.270676 | 100.267435 | 130.793410 | 68.735886 | 208.477234 | 0.000804 | 0.125661 |
| APPROACH | 1180 | -6.006979 | 103.688057 | 127.912443 | 74.710627 | 206.795515 | 0.000082 | 0.002192 |
| DONE | 7682 | -2.495352 | 92.978384 | 126.818685 | 62.251664 | 188.372106 | 0.395517 | 0.409259 |
| MOVING_TO_START | 4840 | -5.879738 | 96.057290 | 126.112589 | 66.718140 | 201.994140 | 0.000063 | 0.187411 |
| SEARCH | 4500 | -7.100414 | 104.094637 | 127.328269 | 73.076494 | 203.122386 | 0.000062 | 0.003157 |
| UNKNOWN | 251 | -3.832036 | 84.455775 | 111.777979 | 38.766241 | 180.063124 | 0.406117 | 0.409551 |

This observer is passive. It does not publish commands or alter controller behavior.
