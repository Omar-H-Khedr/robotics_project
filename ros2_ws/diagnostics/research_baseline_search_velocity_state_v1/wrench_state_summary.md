# Wrench State Summary

- wrench_topic: `/ft_sensor_wrench`
- state_topic: `/insertion_state`
- joint_state_topic: `/joint_states`
- samples: `21114`
- duration_s: `211.130`
- max_abs_fz_n: `132.453473`
- max_force_norm_n: `211.071638`

| State | Samples | Mean Fz N | P95 abs Fz N | Max abs Fz N | Mean norm N | Max norm N | Min XY m | Mean XY m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| ABORT | 2470 | -4.788738 | 98.017803 | 125.041815 | 68.813255 | 205.553701 | 0.000340 | 0.111716 |
| APPROACH | 1200 | -9.301822 | 104.307257 | 132.453473 | 75.158153 | 211.071638 | 0.000096 | 0.002124 |
| DONE | 8153 | -3.301925 | 91.923659 | 127.026315 | 62.416197 | 185.481067 | 0.396960 | 0.409254 |
| MOVING_TO_START | 4572 | -5.375513 | 93.946451 | 124.382387 | 65.498299 | 207.617265 | 0.000097 | 0.200198 |
| SEARCH | 4500 | -7.212922 | 103.450579 | 127.552191 | 73.317672 | 205.172954 | 0.000049 | 0.002329 |
| UNKNOWN | 219 | -3.948102 | 74.916847 | 117.159539 | 29.099124 | 177.050659 | 0.404045 | 0.409435 |

This observer is passive. It does not publish commands or alter controller behavior.
