# Wrench State Summary

- wrench_topic: `/ft_sensor_wrench`
- state_topic: `/insertion_state`
- joint_state_topic: `/joint_states`
- samples: `21556`
- duration_s: `215.549`
- max_abs_fz_n: `227.088310`
- max_force_norm_n: `234.958292`

| State | Samples | Mean Fz N | P95 abs Fz N | Max abs Fz N | Mean norm N | Max norm N | Min XY m | Mean XY m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| APPROACH | 1200 | -7.145279 | 104.350803 | 127.503232 | 74.874122 | 202.658047 | 0.000030 | 0.002206 |
| DONE | 10525 | -3.585765 | 92.668038 | 126.880339 | 62.518493 | 190.522725 | 0.395463 | 0.409255 |
| INSERT | 2310 | -8.864126 | 104.502590 | 125.805336 | 74.406611 | 207.218757 | 0.000047 | 0.002176 |
| MOVING_TO_START | 4750 | -5.886349 | 93.848501 | 125.929109 | 65.652986 | 206.635813 | 0.000075 | 0.189748 |
| RETREAT | 2470 | -4.947258 | 101.127778 | 227.088310 | 71.098666 | 234.958292 | 0.000366 | 0.095483 |
| SEARCH | 20 | -5.427803 | 98.620043 | 107.656966 | 71.868616 | 149.370426 | 0.001366 | 0.002185 |
| UNKNOWN | 281 | -6.550269 | 84.870432 | 105.098569 | 38.250979 | 167.529112 | 0.404084 | 0.409493 |

This observer is passive. It does not publish commands or alter controller behavior.
