# Wrench State Summary

- wrench_topic: `/ft_sensor_wrench`
- state_topic: `/insertion_state`
- joint_state_topic: `/joint_states`
- samples: `12542`
- duration_s: `125.409`
- max_abs_fz_n: `1020.099637`
- max_force_norm_n: `1043.003851`

| State | Samples | Mean Fz N | P95 abs Fz N | Max abs Fz N | Mean norm N | Max norm N | Min XY m | Mean XY m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| ABORT | 1454 | -5.289965 | 226.993577 | 1020.099637 | 175.664232 | 1043.003851 | 0.000168 | 0.251632 |
| APPROACH | 29 | -63.064093 | 727.823202 | 968.406086 | 266.101611 | 1009.722857 | 0.000220 | 0.002702 |
| DONE | 6702 | -0.045917 | 233.990862 | 284.121751 | 204.265381 | 546.869341 | 0.473367 | 0.514737 |
| MOVING_TO_START | 4081 | -4.324073 | 228.332572 | 771.664203 | 175.063771 | 799.817516 | 0.000389 | 0.300120 |
| UNKNOWN | 276 | -0.677957 | 227.630701 | 259.624550 | 131.545750 | 544.411182 | 0.488608 | 0.515269 |

This observer is passive. It does not publish commands or alter controller behavior.
