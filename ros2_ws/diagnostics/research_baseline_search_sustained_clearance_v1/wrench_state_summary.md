# Wrench State Summary

- wrench_topic: `/ft_sensor_wrench`
- state_topic: `/insertion_state`
- joint_state_topic: `/joint_states`
- samples: `14814`
- duration_s: `148.130`
- max_abs_fz_n: `129.141386`
- max_force_norm_n: `211.418245`

| State | Samples | Mean Fz N | P95 abs Fz N | Max abs Fz N | Mean norm N | Max norm N | Min XY m | Mean XY m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| ABORT | 2460 | -6.973245 | 100.047714 | 126.168948 | 71.641193 | 200.512445 | 0.000449 | 0.110703 |
| APPROACH | 1250 | -7.160708 | 104.486127 | 127.378866 | 75.198623 | 211.418245 | 0.000134 | 0.002459 |
| DONE | 2153 | -2.714985 | 92.700773 | 120.883859 | 61.831755 | 183.621683 | 0.395314 | 0.409126 |
| MOVING_TO_START | 4220 | -5.019058 | 89.736982 | 129.141386 | 64.840479 | 209.989024 | 0.000130 | 0.217933 |
| SEARCH | 4500 | -8.637371 | 104.391874 | 127.492995 | 73.762276 | 206.642989 | 0.000037 | 0.003108 |
| UNKNOWN | 231 | -4.293123 | 87.881055 | 103.305978 | 49.509562 | 165.767980 | 0.405358 | 0.409566 |

This observer is passive. It does not publish commands or alter controller behavior.
