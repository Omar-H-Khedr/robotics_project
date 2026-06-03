# Wrench State Summary

- wrench_topic: `/ft_sensor_wrench`
- state_topic: `/insertion_state`
- joint_state_topic: `/joint_states`
- samples: `19200`
- duration_s: `191.989`
- max_abs_fz_n: `128.608358`
- max_force_norm_n: `209.874192`

| State | Samples | Mean Fz N | P95 abs Fz N | Max abs Fz N | Mean norm N | Max norm N | Min XY m | Mean XY m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| ABORT | 2470 | -7.692392 | 101.144058 | 128.608358 | 69.536809 | 209.874192 | 0.000114 | 0.111887 |
| APPROACH | 1190 | -9.615780 | 104.606896 | 127.461468 | 74.472143 | 203.212667 | 0.000062 | 0.002287 |
| DONE | 5769 | -2.812141 | 93.556958 | 123.782197 | 62.877743 | 186.362847 | 0.395957 | 0.409264 |
| MOVING_TO_START | 4920 | -6.156546 | 93.880595 | 128.250357 | 66.103565 | 209.033294 | 0.000064 | 0.188614 |
| SEARCH | 4500 | -6.413925 | 104.119320 | 126.631078 | 73.065283 | 206.216639 | 0.000044 | 0.002975 |
| UNKNOWN | 351 | -1.136731 | 76.845847 | 112.426771 | 35.598576 | 174.362320 | 0.404504 | 0.409502 |

This observer is passive. It does not publish commands or alter controller behavior.
