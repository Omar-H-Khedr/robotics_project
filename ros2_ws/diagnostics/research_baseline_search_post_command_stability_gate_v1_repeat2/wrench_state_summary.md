# Wrench State Summary

- wrench_topic: `/ft_sensor_wrench`
- state_topic: `/insertion_state`
- joint_state_topic: `/joint_states`
- samples: `21853`
- duration_s: `218.519`
- max_abs_fz_n: `129.840459`
- max_force_norm_n: `209.043013`

| State | Samples | Mean Fz N | P95 abs Fz N | Max abs Fz N | Mean norm N | Max norm N | Min XY m | Mean XY m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| ABORT | 2470 | -5.198941 | 98.730789 | 128.481501 | 70.231796 | 209.043013 | 0.000103 | 0.111376 |
| APPROACH | 1190 | -6.623306 | 100.883754 | 129.032036 | 73.578606 | 199.328511 | 0.000030 | 0.002444 |
| DONE | 8422 | -2.489119 | 92.792224 | 127.164229 | 62.375900 | 188.331260 | 0.398274 | 0.409255 |
| IDLE | 80 | 1.030865 | 95.739191 | 110.706309 | 64.983073 | 172.071496 | 0.403351 | 0.409307 |
| MOVING_TO_START | 5070 | -5.147659 | 94.184328 | 129.840459 | 66.794165 | 204.469139 | 0.000052 | 0.181293 |
| SEARCH | 4500 | -6.859177 | 104.930214 | 126.336145 | 73.944848 | 205.533519 | 0.000025 | 0.002371 |
| UNKNOWN | 121 | 0.365766 | 84.653805 | 108.211139 | 36.612678 | 165.503291 | 0.405830 | 0.409594 |

This observer is passive. It does not publish commands or alter controller behavior.
