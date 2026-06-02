# Wrench State Summary

- wrench_topic: `/ft_sensor_wrench`
- state_topic: `/insertion_state`
- joint_state_topic: `/joint_states`
- samples: `9412`
- duration_s: `94.109`
- max_abs_fz_n: `172.601476`
- max_force_norm_n: `269.970106`

| State | Samples | Mean Fz N | P95 abs Fz N | Max abs Fz N | Mean norm N | Max norm N | Min XY m | Mean XY m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| ABORT | 1461 | -6.109084 | 136.896017 | 172.601476 | 95.829561 | 262.977441 | 0.000832 | 0.189480 |
| DONE | 1482 | -1.342698 | 140.683769 | 162.543712 | 100.276828 | 255.435125 | 0.385337 | 0.407982 |
| MOVING_TO_START | 6204 | -6.475116 | 138.173917 | 169.297536 | 97.662023 | 269.970106 | 0.000394 | 0.145080 |
| UNKNOWN | 265 | -7.879831 | 133.049923 | 155.682550 | 61.821280 | 262.553040 | 0.390510 | 0.408886 |

This observer is passive. It does not publish commands or alter controller behavior.
