# Wrench State Summary

- wrench_topic: `/ft_sensor_wrench`
- state_topic: `/insertion_state`
- joint_state_topic: `/joint_states`
- samples: `2512`
- duration_s: `25.109`
- max_abs_fz_n: `160.004884`
- max_force_norm_n: `241.038866`

| State | Samples | Mean Fz N | P95 abs Fz N | Max abs Fz N | Mean norm N | Max norm N | Min XY m | Mean XY m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| MOVING_TO_START | 2241 | -4.856154 | 135.872317 | 158.535904 | 91.150009 | 241.038866 | 0.187464 | 0.321004 |
| UNKNOWN | 271 | 2.173177 | 124.264474 | 160.004884 | 58.355977 | 235.493769 | 0.388263 | 0.408426 |

This observer is passive. It does not publish commands or alter controller behavior.
