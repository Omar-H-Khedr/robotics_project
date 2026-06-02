# Wrench State Summary

- wrench_topic: `/ft_sensor_wrench`
- state_topic: `/insertion_state`
- joint_state_topic: `/joint_states`
- samples: `12417`
- duration_s: `124.159`
- max_abs_fz_n: `1018.895796`
- max_force_norm_n: `1195.802719`

| State | Samples | Mean Fz N | P95 abs Fz N | Max abs Fz N | Mean norm N | Max norm N | Min XY m | Mean XY m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| ABORT | 1439 | -2.548274 | 230.426301 | 535.620683 | 192.098863 | 597.725290 | 0.171202 | 0.335719 |
| DONE | 7781 | -2.567339 | 235.735262 | 284.648365 | 205.146942 | 549.980965 | 0.480894 | 0.515495 |
| IDLE | 25 | -14.427924 | 226.858398 | 234.804666 | 206.456976 | 466.634540 | 0.498247 | 0.514465 |
| MOVING_TO_START | 2913 | -4.266952 | 229.010373 | 1018.895796 | 191.749793 | 1195.802719 | 0.177806 | 0.370378 |
| UNKNOWN | 259 | -4.125715 | 219.242612 | 262.960939 | 85.995475 | 476.773266 | 0.484634 | 0.515722 |

This observer is passive. It does not publish commands or alter controller behavior.
