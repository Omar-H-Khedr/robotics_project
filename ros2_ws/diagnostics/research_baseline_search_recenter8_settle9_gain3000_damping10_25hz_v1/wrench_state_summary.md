# Wrench State Summary

- wrench_topic: `/ft_sensor_wrench`
- state_topic: `/insertion_state`
- joint_state_topic: `/joint_states`
- samples: `11247`
- duration_s: `112.459`
- max_abs_fz_n: `101.676019`
- max_force_norm_n: `169.481710`

| State | Samples | Mean Fz N | P95 abs Fz N | Max abs Fz N | Mean norm N | Max norm N | Min XY m | Mean XY m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| ABORT | 2452 | -6.739024 | 72.681913 | 101.676019 | 55.738789 | 168.216678 | 0.000095 | 0.117535 |
| APPROACH | 1180 | -6.668000 | 73.348112 | 100.598072 | 61.033293 | 169.481710 | 0.000029 | 0.001263 |
| DONE | 1854 | -2.520301 | 64.180831 | 99.105437 | 46.159475 | 145.757743 | 0.397605 | 0.409294 |
| INSERT | 1208 | -6.512083 | 76.337646 | 97.853786 | 60.830854 | 167.432621 | 0.000019 | 0.001281 |
| MOVING_TO_START | 4005 | -5.663722 | 62.005207 | 99.239379 | 52.518171 | 165.192137 | 0.000236 | 0.228634 |
| SEARCH | 264 | -8.166437 | 74.813391 | 99.839640 | 59.762441 | 161.435850 | 0.000026 | 0.001156 |
| UNKNOWN | 284 | -1.493053 | 53.993437 | 86.085901 | 33.828354 | 136.519024 | 0.406593 | 0.409537 |

This observer is passive. It does not publish commands or alter controller behavior.
