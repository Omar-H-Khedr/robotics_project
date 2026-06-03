# Wrench State Summary

- wrench_topic: `/ft_sensor_wrench`
- state_topic: `/insertion_state`
- joint_state_topic: `/joint_states`
- samples: `20241`
- duration_s: `202.399`
- max_abs_fz_n: `132.073037`
- max_force_norm_n: `208.349224`

| State | Samples | Mean Fz N | P95 abs Fz N | Max abs Fz N | Mean norm N | Max norm N | Min XY m | Mean XY m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| ABORT | 2470 | -5.734064 | 99.240075 | 132.073037 | 70.072188 | 208.349224 | 0.000069 | 0.118688 |
| APPROACH | 1230 | -4.477121 | 103.026545 | 128.769260 | 72.847972 | 208.338233 | 0.000066 | 0.002089 |
| DONE | 7430 | -2.869317 | 92.941102 | 127.347493 | 62.232148 | 185.509677 | 0.398501 | 0.409258 |
| MOVING_TO_START | 4303 | -5.918994 | 93.685645 | 126.198065 | 65.219408 | 205.570633 | 0.000120 | 0.217827 |
| SEARCH | 4500 | -7.248049 | 104.989955 | 130.537100 | 72.777218 | 204.389868 | 0.000059 | 0.002276 |
| UNKNOWN | 308 | -3.403627 | 80.739694 | 114.527337 | 30.446641 | 162.234070 | 0.404466 | 0.409619 |

This observer is passive. It does not publish commands or alter controller behavior.
