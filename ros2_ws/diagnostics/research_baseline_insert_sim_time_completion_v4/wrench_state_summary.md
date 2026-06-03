# Wrench State Summary

- wrench_topic: `/ft_sensor_wrench`
- state_topic: `/insertion_state`
- joint_state_topic: `/joint_states`
- samples: `18159`
- duration_s: `181.580`
- max_abs_fz_n: `133.329861`
- max_force_norm_n: `211.137237`

| State | Samples | Mean Fz N | P95 abs Fz N | Max abs Fz N | Mean norm N | Max norm N | Min XY m | Mean XY m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| APPROACH | 1190 | -7.830433 | 104.792915 | 133.329861 | 75.380875 | 205.630221 | 0.000049 | 0.002212 |
| DONE | 7618 | -2.432368 | 92.966403 | 128.954802 | 62.901954 | 189.895810 | 0.399227 | 0.409278 |
| INSERT | 2309 | -6.875445 | 102.621274 | 123.085613 | 71.428853 | 206.668557 | 0.000051 | 0.002086 |
| MOVING_TO_START | 4290 | -5.955837 | 93.951259 | 129.035041 | 65.602439 | 211.137237 | 0.000083 | 0.212705 |
| RETREAT | 2481 | -5.199347 | 100.446001 | 124.504805 | 70.274689 | 200.945040 | 0.000067 | 0.095468 |
| SEARCH | 20 | -7.451892 | 104.629443 | 109.065097 | 75.825971 | 192.266852 | 0.000642 | 0.002273 |
| UNKNOWN | 251 | 1.407423 | 87.379962 | 108.427390 | 38.830390 | 181.476751 | 0.404659 | 0.409438 |

This observer is passive. It does not publish commands or alter controller behavior.
