# Wrench State Summary

- wrench_topic: `/ft_sensor_wrench`
- state_topic: `/insertion_state`
- joint_state_topic: `/joint_states`
- samples: `7455`
- duration_s: `74.540`
- max_abs_fz_n: `1943.293077`
- max_force_norm_n: `2936.479541`

| State | Samples | Mean Fz N | P95 abs Fz N | Max abs Fz N | Mean norm N | Max norm N | Min XY m | Mean XY m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| ABORT | 596 | 2.847378 | 141.218347 | 1184.891784 | 113.617421 | 2198.320058 | 0.000297 | 0.167202 |
| DONE | 2379 | -2.793069 | 139.972201 | 173.401596 | 110.805005 | 327.731293 | 0.446091 | 0.514223 |
| MOVING_TO_START | 4202 | 3.298117 | 141.305845 | 1943.293077 | 111.766691 | 2936.479541 | 0.000065 | 0.134294 |
| UNKNOWN | 278 | -2.347455 | 134.169414 | 156.100209 | 65.523859 | 321.721401 | 0.500445 | 0.516064 |

This observer is passive. It does not publish commands or alter controller behavior.
