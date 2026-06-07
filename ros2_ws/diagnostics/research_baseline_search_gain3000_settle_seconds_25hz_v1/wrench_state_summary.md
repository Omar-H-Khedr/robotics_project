# Wrench State Summary

- wrench_topic: `/ft_sensor_wrench`
- state_topic: `/insertion_state`
- joint_state_topic: `/joint_states`
- samples: `8562`
- duration_s: `85.609`
- max_abs_fz_n: `128.621720`
- max_force_norm_n: `207.417160`

| State | Samples | Mean Fz N | P95 abs Fz N | Max abs Fz N | Mean norm N | Max norm N | Min XY m | Mean XY m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| ABORT | 2452 | -4.354106 | 97.720869 | 126.488493 | 68.708212 | 201.379392 | 0.000078 | 0.126089 |
| APPROACH | 1144 | -7.124829 | 102.076498 | 124.755091 | 74.082818 | 207.417160 | 0.000052 | 0.002223 |
| DONE | 685 | -2.934572 | 96.788677 | 119.752169 | 62.740542 | 180.963282 | 0.396737 | 0.408752 |
| INSERT | 12 | -38.125315 | 90.526146 | 103.021854 | 59.669553 | 104.936070 | 0.000347 | 0.001946 |
| MOVING_TO_START | 4000 | -4.246022 | 89.783720 | 128.621720 | 64.515374 | 196.521849 | 0.000766 | 0.228384 |
| UNKNOWN | 269 | -2.762643 | 84.696208 | 114.773500 | 38.844870 | 170.677379 | 0.405019 | 0.409523 |

This observer is passive. It does not publish commands or alter controller behavior.
