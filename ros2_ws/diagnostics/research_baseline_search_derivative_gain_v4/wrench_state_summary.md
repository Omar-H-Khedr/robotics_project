# Wrench State Summary

- wrench_topic: `/ft_sensor_wrench`
- state_topic: `/insertion_state`
- joint_state_topic: `/joint_states`
- samples: `20255`
- duration_s: `202.539`
- max_abs_fz_n: `129.208812`
- max_force_norm_n: `210.348992`

| State | Samples | Mean Fz N | P95 abs Fz N | Max abs Fz N | Mean norm N | Max norm N | Min XY m | Mean XY m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| ABORT | 2469 | -7.677547 | 101.688983 | 125.904797 | 70.493929 | 210.348992 | 0.000376 | 0.111615 |
| APPROACH | 1181 | -7.860174 | 104.710305 | 129.208812 | 73.665194 | 201.012671 | 0.000040 | 0.002209 |
| DONE | 7784 | -3.194500 | 92.933094 | 128.407995 | 62.485418 | 189.668120 | 0.396075 | 0.409264 |
| MOVING_TO_START | 4060 | -5.373091 | 90.516325 | 126.228347 | 63.867508 | 203.827670 | 0.000049 | 0.224111 |
| SEARCH | 4500 | -7.464880 | 104.890062 | 127.136518 | 74.137150 | 207.836918 | 0.000028 | 0.002510 |
| UNKNOWN | 261 | -3.148905 | 80.937938 | 115.135716 | 36.030967 | 175.491934 | 0.404291 | 0.409402 |

This observer is passive. It does not publish commands or alter controller behavior.
