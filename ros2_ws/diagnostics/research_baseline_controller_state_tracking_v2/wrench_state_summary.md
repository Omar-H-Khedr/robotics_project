# Wrench State Summary

- wrench_topic: `/ft_sensor_wrench`
- state_topic: `/insertion_state`
- joint_state_topic: `/joint_states`
- samples: `9670`
- duration_s: `96.689`
- max_abs_fz_n: `171.662452`
- max_force_norm_n: `272.806449`

| State | Samples | Mean Fz N | P95 abs Fz N | Max abs Fz N | Mean norm N | Max norm N | Min XY m | Mean XY m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| ABORT | 1457 | -2.492153 | 136.892127 | 169.914329 | 95.803099 | 272.404993 | 0.002969 | 0.189515 |
| DONE | 1580 | -1.905132 | 141.155144 | 159.143165 | 100.901149 | 255.712317 | 0.383540 | 0.407609 |
| MOVING_TO_START | 6364 | -5.944253 | 139.921528 | 171.662452 | 98.804297 | 272.806449 | 0.000218 | 0.144654 |
| UNKNOWN | 269 | -3.883799 | 131.580613 | 156.934054 | 59.521620 | 230.506907 | 0.396242 | 0.408754 |

This observer is passive. It does not publish commands or alter controller behavior.
