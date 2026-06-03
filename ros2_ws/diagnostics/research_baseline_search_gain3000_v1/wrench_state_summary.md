# Wrench State Summary

- wrench_topic: `/ft_sensor_wrench`
- state_topic: `/insertion_state`
- joint_state_topic: `/joint_states`
- samples: `20975`
- duration_s: `209.740`
- max_abs_fz_n: `131.775716`
- max_force_norm_n: `209.134285`

| State | Samples | Mean Fz N | P95 abs Fz N | Max abs Fz N | Mean norm N | Max norm N | Min XY m | Mean XY m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| ABORT | 2470 | -7.088347 | 99.569170 | 126.394760 | 69.497806 | 201.124847 | 0.000245 | 0.112433 |
| APPROACH | 1140 | -9.010768 | 104.390328 | 129.103283 | 73.922281 | 209.134285 | 0.000092 | 0.002112 |
| DONE | 8154 | -2.475055 | 92.860735 | 127.235759 | 62.486392 | 186.965089 | 0.397518 | 0.409236 |
| MOVING_TO_START | 4444 | -6.868589 | 91.976761 | 123.947588 | 64.969189 | 203.498187 | 0.000085 | 0.204687 |
| SEARCH | 4500 | -8.030581 | 104.114682 | 131.775716 | 74.372477 | 206.430404 | 0.000026 | 0.003125 |
| UNKNOWN | 267 | -3.116424 | 83.738628 | 105.170937 | 39.426769 | 162.982943 | 0.406128 | 0.409551 |

This observer is passive. It does not publish commands or alter controller behavior.
