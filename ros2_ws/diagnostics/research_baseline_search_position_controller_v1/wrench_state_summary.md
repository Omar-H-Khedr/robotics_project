# Wrench State Summary

- wrench_topic: `/ft_sensor_wrench`
- state_topic: `/insertion_state`
- joint_state_topic: `/joint_states`
- samples: `19712`
- duration_s: `197.109`
- max_abs_fz_n: `128.720445`
- max_force_norm_n: `204.696168`

| State | Samples | Mean Fz N | P95 abs Fz N | Max abs Fz N | Mean norm N | Max norm N | Min XY m | Mean XY m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| ABORT | 1170 | -7.327972 | 100.456060 | 125.614135 | 69.752034 | 204.696168 | 0.000097 | 0.112427 |
| APPROACH | 600 | -8.117631 | 101.183495 | 122.627566 | 73.478084 | 195.962695 | 0.000028 | 0.002122 |
| DONE | 11061 | -2.301413 | 92.894897 | 128.720445 | 62.474401 | 187.810238 | 0.398621 | 0.409307 |
| MOVING_TO_START | 2106 | -5.520097 | 92.206902 | 118.660226 | 65.658874 | 197.366675 | 0.000335 | 0.215068 |
| SEARCH | 4500 | -8.802491 | 103.889407 | 127.441032 | 73.554304 | 200.805234 | 0.000059 | 0.002366 |
| UNKNOWN | 275 | -1.145566 | 87.791771 | 125.176171 | 38.118785 | 166.764485 | 0.404274 | 0.409296 |

This observer is passive. It does not publish commands or alter controller behavior.
