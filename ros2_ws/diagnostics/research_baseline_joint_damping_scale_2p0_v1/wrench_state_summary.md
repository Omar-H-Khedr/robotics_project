# Wrench State Summary

- wrench_topic: `/ft_sensor_wrench`
- state_topic: `/insertion_state`
- joint_state_topic: `/joint_states`
- samples: `9041`
- duration_s: `90.400`
- max_abs_fz_n: `165.445536`
- max_force_norm_n: `255.543901`

| State | Samples | Mean Fz N | P95 abs Fz N | Max abs Fz N | Mean norm N | Max norm N | Min XY m | Mean XY m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| ABORT | 1458 | -4.415818 | 124.499294 | 157.992363 | 85.315647 | 240.477719 | 0.000359 | 0.189237 |
| DONE | 1025 | -5.733580 | 127.757470 | 159.387049 | 87.353441 | 236.413796 | 0.390034 | 0.408259 |
| IDLE | 66 | -0.581831 | 122.525099 | 141.988151 | 87.715630 | 186.129024 | 0.401708 | 0.408696 |
| MOVING_TO_START | 6234 | -6.370971 | 126.052515 | 165.445536 | 87.358137 | 255.543901 | 0.000153 | 0.148320 |
| UNKNOWN | 258 | -0.210632 | 101.005226 | 134.627006 | 25.210332 | 222.624893 | 0.402015 | 0.409501 |

This observer is passive. It does not publish commands or alter controller behavior.
