# Wrench State Summary

- wrench_topic: `/ft_sensor_wrench`
- state_topic: `/insertion_state`
- joint_state_topic: `/joint_states`
- samples: `10869`
- duration_s: `108.679`
- max_abs_fz_n: `176.567202`
- max_force_norm_n: `272.232776`

| State | Samples | Mean Fz N | P95 abs Fz N | Max abs Fz N | Mean norm N | Max norm N | Min XY m | Mean XY m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| ABORT | 1450 | -6.649369 | 135.476266 | 176.567202 | 95.232660 | 267.639364 | 0.000743 | 0.190639 |
| DONE | 2937 | -5.086833 | 142.154591 | 159.395702 | 99.818819 | 263.485610 | 0.383297 | 0.407703 |
| MOVING_TO_START | 6205 | -5.429438 | 140.211576 | 171.759781 | 98.468915 | 272.232776 | 0.000257 | 0.146228 |
| UNKNOWN | 277 | 0.628907 | 131.639786 | 157.686839 | 52.419108 | 262.553808 | 0.390330 | 0.408925 |

This observer is passive. It does not publish commands or alter controller behavior.
