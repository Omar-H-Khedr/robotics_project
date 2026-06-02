# Wrench State Summary

- wrench_topic: `/ft_sensor_wrench`
- state_topic: `/insertion_state`
- joint_state_topic: `/joint_states`
- samples: `11630`
- duration_s: `116.288`
- max_abs_fz_n: `171.250782`
- max_force_norm_n: `271.344775`

| State | Samples | Mean Fz N | P95 abs Fz N | Max abs Fz N | Mean norm N | Max norm N | Min XY m | Mean XY m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| ABORT | 1460 | -10.501618 | 137.662228 | 163.468393 | 97.023027 | 263.273369 | 0.001475 | 0.186804 |
| DONE | 3851 | -3.371879 | 140.885379 | 161.620273 | 100.415545 | 263.143193 | 0.382210 | 0.407680 |
| MOVING_TO_START | 6061 | -8.058381 | 140.022503 | 171.250782 | 98.992131 | 271.344775 | 0.000140 | 0.153131 |
| UNKNOWN | 258 | -2.098418 | 132.046499 | 155.722261 | 60.748400 | 248.469908 | 0.391153 | 0.408564 |

This observer is passive. It does not publish commands or alter controller behavior.
