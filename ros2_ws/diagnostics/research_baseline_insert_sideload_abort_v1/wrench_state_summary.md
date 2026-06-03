# Wrench State Summary

- wrench_topic: `/ft_sensor_wrench`
- state_topic: `/insertion_state`
- joint_state_topic: `/joint_states`
- samples: `21637`
- duration_s: `216.359`
- max_abs_fz_n: `128.534595`
- max_force_norm_n: `204.140105`

| State | Samples | Mean Fz N | P95 abs Fz N | Max abs Fz N | Mean norm N | Max norm N | Min XY m | Mean XY m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| ABORT | 2470 | -4.454773 | 98.938271 | 124.883748 | 68.569767 | 204.140105 | 0.000043 | 0.104975 |
| APPROACH | 1190 | -6.698333 | 101.958685 | 128.381883 | 73.019726 | 199.020908 | 0.000056 | 0.002319 |
| DONE | 12416 | -2.597118 | 92.872234 | 125.774867 | 62.431367 | 188.319738 | 0.396946 | 0.409261 |
| INSERT | 1280 | -7.156839 | 103.902892 | 126.773956 | 73.444700 | 200.165296 | 0.000047 | 0.002909 |
| MOVING_TO_START | 4010 | -5.908305 | 90.134617 | 128.534595 | 64.065111 | 201.745461 | 0.000273 | 0.227215 |
| UNKNOWN | 271 | -1.491836 | 91.561550 | 116.810784 | 38.669521 | 177.865051 | 0.405760 | 0.409531 |

This observer is passive. It does not publish commands or alter controller behavior.
