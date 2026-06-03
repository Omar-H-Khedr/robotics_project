# Wrench State Summary

- wrench_topic: `/ft_sensor_wrench`
- state_topic: `/insertion_state`
- joint_state_topic: `/joint_states`
- samples: `11719`
- duration_s: `117.179`
- max_abs_fz_n: `129.397658`
- max_force_norm_n: `205.090642`

| State | Samples | Mean Fz N | P95 abs Fz N | Max abs Fz N | Mean norm N | Max norm N | Min XY m | Mean XY m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| ABORT | 2460 | -6.148058 | 99.827203 | 125.907529 | 69.305037 | 200.446259 | 0.000216 | 0.118591 |
| APPROACH | 1220 | -8.633751 | 105.135863 | 129.397658 | 73.940876 | 204.882497 | 0.000080 | 0.002113 |
| DONE | 2898 | -2.830842 | 94.034783 | 126.753893 | 63.115067 | 182.176724 | 0.395695 | 0.409188 |
| INSERT | 50 | -2.982135 | 103.633458 | 109.307240 | 67.713381 | 181.134548 | 0.000287 | 0.001924 |
| MOVING_TO_START | 4610 | -5.554068 | 92.972623 | 126.640378 | 65.741383 | 205.051445 | 0.000127 | 0.196909 |
| SEARCH | 230 | -8.028973 | 98.111352 | 123.084811 | 80.044086 | 205.090642 | 0.000192 | 0.002191 |
| UNKNOWN | 251 | -0.268185 | 87.321346 | 121.353317 | 39.152315 | 181.708571 | 0.406091 | 0.409631 |

This observer is passive. It does not publish commands or alter controller behavior.
