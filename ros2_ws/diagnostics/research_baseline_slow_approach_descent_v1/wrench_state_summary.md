# Wrench State Summary

- wrench_topic: `/ft_sensor_wrench`
- state_topic: `/insertion_state`
- joint_state_topic: `/joint_states`
- samples: `12538`
- duration_s: `125.366`
- max_abs_fz_n: `570.066682`
- max_force_norm_n: `627.034441`

| State | Samples | Mean Fz N | P95 abs Fz N | Max abs Fz N | Mean norm N | Max norm N | Min XY m | Mean XY m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| ABORT | 1468 | -4.936696 | 132.725493 | 444.621298 | 100.302576 | 536.263223 | 0.013453 | 0.264320 |
| APPROACH | 4400 | -6.739581 | 95.755594 | 570.066682 | 81.848764 | 627.034441 | 0.000108 | 0.008399 |
| DONE | 1621 | -2.289326 | 140.512391 | 170.870651 | 109.953119 | 332.292671 | 0.493975 | 0.515058 |
| MOVING_TO_START | 4771 | -6.056718 | 131.157086 | 499.376828 | 95.584753 | 547.018926 | 0.001153 | 0.259998 |
| UNKNOWN | 278 | -2.149040 | 139.152196 | 158.530443 | 71.054762 | 320.986999 | 0.502139 | 0.516705 |

This observer is passive. It does not publish commands or alter controller behavior.
