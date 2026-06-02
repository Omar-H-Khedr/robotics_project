# Wrench State Summary

- wrench_topic: `/ft_sensor_wrench`
- state_topic: `/insertion_state`
- joint_state_topic: `/joint_states`
- samples: `7383`
- duration_s: `73.818`
- max_abs_fz_n: `939.358852`
- max_force_norm_n: `1748.498723`

| State | Samples | Mean Fz N | P95 abs Fz N | Max abs Fz N | Mean norm N | Max norm N | Min XY m | Mean XY m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| ABORT | 609 | 4.437086 | 143.311535 | 175.562291 | 112.814470 | 312.585830 | 0.000319 | 0.170489 |
| DONE | 2379 | -1.223472 | 140.409672 | 174.224230 | 109.026523 | 332.696935 | 0.454107 | 0.514040 |
| MOVING_TO_START | 4110 | 2.040130 | 141.506453 | 939.358852 | 110.549789 | 1748.498723 | 0.000334 | 0.132957 |
| UNKNOWN | 285 | -0.245430 | 130.823384 | 151.612320 | 65.940688 | 308.042632 | 0.500578 | 0.515131 |

This observer is passive. It does not publish commands or alter controller behavior.
