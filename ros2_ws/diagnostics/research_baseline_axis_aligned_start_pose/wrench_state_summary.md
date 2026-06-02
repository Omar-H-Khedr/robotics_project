# Wrench State Summary

- wrench_topic: `/ft_sensor_wrench`
- state_topic: `/insertion_state`
- joint_state_topic: `/joint_states`
- samples: `7233`
- duration_s: `72.319`
- max_abs_fz_n: `554.238843`
- max_force_norm_n: `628.609265`

| State | Samples | Mean Fz N | P95 abs Fz N | Max abs Fz N | Mean norm N | Max norm N | Min XY m | Mean XY m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| ABORT | 1469 | -6.030649 | 133.111114 | 546.182305 | 98.679460 | 628.609265 | 0.003380 | 0.257607 |
| DONE | 960 | -5.497815 | 135.966402 | 165.136207 | 111.244431 | 332.855175 | 0.493583 | 0.514521 |
| IDLE | 96 | -8.411862 | 138.295894 | 156.100209 | 105.857385 | 273.176883 | 0.502253 | 0.515759 |
| MOVING_TO_START | 4553 | -3.925599 | 131.866308 | 554.238843 | 98.716843 | 598.510522 | 0.005870 | 0.271630 |
| UNKNOWN | 155 | -3.915521 | 113.565269 | 144.670633 | 31.214803 | 321.721401 | 0.500445 | 0.515896 |

This observer is passive. It does not publish commands or alter controller behavior.
