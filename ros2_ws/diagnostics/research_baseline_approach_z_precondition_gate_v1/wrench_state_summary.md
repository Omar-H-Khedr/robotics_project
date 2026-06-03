# Wrench State Summary

- wrench_topic: `/ft_sensor_wrench`
- state_topic: `/insertion_state`
- joint_state_topic: `/joint_states`
- samples: `7272`
- duration_s: `72.710`
- max_abs_fz_n: `594.283889`
- max_force_norm_n: `949.257744`

| State | Samples | Mean Fz N | P95 abs Fz N | Max abs Fz N | Mean norm N | Max norm N | Min XY m | Mean XY m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| APPROACH | 1178 | -5.651230 | 104.030912 | 127.543219 | 73.712499 | 208.030911 | 0.000022 | 0.002042 |
| IDLE | 81 | -2.741632 | 99.165832 | 116.365116 | 64.360597 | 175.929637 | 0.405051 | 0.409227 |
| INSERT | 1035 | -7.009043 | 104.286961 | 125.636943 | 72.690595 | 209.244996 | 0.000087 | 0.002775 |
| MOVING_TO_START | 4116 | -5.109756 | 92.205285 | 128.729012 | 64.113933 | 207.792381 | 0.000236 | 0.221566 |
| RETREAT | 691 | 135.488836 | 416.306067 | 594.283889 | 325.848980 | 949.257744 | 0.000237 | 0.018550 |
| SEARCH | 5 | 2.198697 | 94.361819 | 94.361819 | 62.968459 | 105.167668 | 0.000834 | 0.002661 |
| UNKNOWN | 166 | -3.926981 | 81.167655 | 100.296453 | 22.210111 | 168.150419 | 0.404407 | 0.409489 |

This observer is passive. It does not publish commands or alter controller behavior.
