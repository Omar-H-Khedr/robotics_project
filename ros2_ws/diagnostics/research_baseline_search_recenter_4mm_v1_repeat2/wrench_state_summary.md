# Wrench State Summary

- wrench_topic: `/ft_sensor_wrench`
- state_topic: `/insertion_state`
- joint_state_topic: `/joint_states`
- samples: `21612`
- duration_s: `216.109`
- max_abs_fz_n: `130.549946`
- max_force_norm_n: `211.531302`

| State | Samples | Mean Fz N | P95 abs Fz N | Max abs Fz N | Mean norm N | Max norm N | Min XY m | Mean XY m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| ABORT | 2470 | -6.872863 | 100.436196 | 125.149141 | 68.091892 | 211.531302 | 0.000129 | 0.112079 |
| APPROACH | 1181 | -5.673343 | 103.959142 | 130.179058 | 76.610279 | 207.642491 | 0.000113 | 0.002130 |
| DONE | 8641 | -2.741463 | 92.792045 | 126.152427 | 62.430333 | 186.511427 | 0.397852 | 0.409270 |
| IDLE | 68 | -0.438378 | 103.247888 | 117.313004 | 62.702747 | 143.213530 | 0.406017 | 0.409652 |
| MOVING_TO_START | 4620 | -5.487067 | 94.145434 | 130.549946 | 65.954553 | 209.249710 | 0.000080 | 0.198241 |
| SEARCH | 4499 | -7.620851 | 103.748123 | 128.524295 | 73.635126 | 206.451815 | 0.000057 | 0.002402 |
| UNKNOWN | 133 | -5.803565 | 76.982920 | 116.344353 | 36.667079 | 170.131239 | 0.405353 | 0.409621 |

This observer is passive. It does not publish commands or alter controller behavior.
