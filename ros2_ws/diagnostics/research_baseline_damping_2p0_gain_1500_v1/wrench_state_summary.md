# Wrench State Summary

- wrench_topic: `/ft_sensor_wrench`
- state_topic: `/insertion_state`
- joint_state_topic: `/joint_states`
- samples: `9876`
- duration_s: `98.749`
- max_abs_fz_n: `166.459344`
- max_force_norm_n: `268.120423`

| State | Samples | Mean Fz N | P95 abs Fz N | Max abs Fz N | Mean norm N | Max norm N | Min XY m | Mean XY m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| ABORT | 1449 | -4.882954 | 124.507489 | 159.220003 | 86.223508 | 268.120423 | 0.001145 | 0.188572 |
| DONE | 1958 | -1.566562 | 128.704662 | 160.929534 | 86.560218 | 230.376475 | 0.385202 | 0.408488 |
| MOVING_TO_START | 6193 | -5.565549 | 126.277946 | 166.459344 | 86.811853 | 250.860262 | 0.000052 | 0.144123 |
| UNKNOWN | 276 | -2.942680 | 111.988570 | 159.795130 | 53.444672 | 222.445398 | 0.392071 | 0.408315 |

This observer is passive. It does not publish commands or alter controller behavior.
