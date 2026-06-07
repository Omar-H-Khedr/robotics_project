# Wrench State Summary

- wrench_topic: `/ft_sensor_wrench`
- state_topic: `/insertion_state`
- joint_state_topic: `/joint_states`
- samples: `12459`
- duration_s: `124.579`
- max_abs_fz_n: `100.507143`
- max_force_norm_n: `170.494237`

| State | Samples | Mean Fz N | P95 abs Fz N | Max abs Fz N | Mean norm N | Max norm N | Min XY m | Mean XY m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| ABORT | 2456 | -5.824145 | 70.469216 | 99.705248 | 56.853772 | 168.681149 | 0.000005 | 0.118179 |
| APPROACH | 1164 | -7.403491 | 76.333291 | 100.507143 | 61.689333 | 169.239505 | 0.000048 | 0.001244 |
| DONE | 58 | 0.262223 | 54.534128 | 91.321631 | 40.908870 | 150.766979 | 0.396833 | 0.404528 |
| MOVING_TO_START | 4004 | -6.025749 | 62.129027 | 97.192305 | 52.493613 | 165.361875 | 0.000431 | 0.226876 |
| SEARCH | 4500 | -7.390531 | 76.273334 | 99.167988 | 60.297536 | 170.494237 | 0.000008 | 0.001249 |
| UNKNOWN | 277 | -2.811112 | 54.683248 | 79.909657 | 30.892557 | 132.605820 | 0.407339 | 0.409535 |

This observer is passive. It does not publish commands or alter controller behavior.
