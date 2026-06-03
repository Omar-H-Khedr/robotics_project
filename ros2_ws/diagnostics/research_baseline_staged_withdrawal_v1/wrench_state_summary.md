# Wrench State Summary

- wrench_topic: `/ft_sensor_wrench`
- state_topic: `/insertion_state`
- joint_state_topic: `/joint_states`
- samples: `21338`
- duration_s: `213.369`
- max_abs_fz_n: `153.007764`
- max_force_norm_n: `285.766425`

| State | Samples | Mean Fz N | P95 abs Fz N | Max abs Fz N | Mean norm N | Max norm N | Min XY m | Mean XY m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| APPROACH | 1190 | -7.476428 | 104.634021 | 127.774326 | 74.372255 | 207.064748 | 0.000025 | 0.002019 |
| DONE | 8947 | -2.590269 | 92.686458 | 129.410280 | 62.656413 | 188.063058 | 0.395178 | 0.409228 |
| INSERT | 2310 | -7.158319 | 103.654178 | 128.431127 | 73.156908 | 204.840394 | 0.000022 | 0.002190 |
| MOVING_TO_START | 4240 | -5.846063 | 92.003924 | 124.248528 | 65.161804 | 208.308440 | 0.000267 | 0.214244 |
| RETREAT | 4360 | -6.612835 | 101.059018 | 153.007764 | 69.306894 | 285.766425 | 0.000027 | 0.107226 |
| SEARCH | 20 | 14.312939 | 83.426939 | 85.058676 | 71.570729 | 169.480457 | 0.000263 | 0.001854 |
| UNKNOWN | 271 | 0.083585 | 81.711958 | 110.433446 | 34.841272 | 184.085954 | 0.405305 | 0.409545 |

This observer is passive. It does not publish commands or alter controller behavior.
