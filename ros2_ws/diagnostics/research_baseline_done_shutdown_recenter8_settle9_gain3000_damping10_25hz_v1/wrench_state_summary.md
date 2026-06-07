# Wrench State Summary

- wrench_topic: `/ft_sensor_wrench`
- state_topic: `/insertion_state`
- joint_state_topic: `/joint_states`
- samples: `8944`
- duration_s: `89.429`
- max_abs_fz_n: `102.138673`
- max_force_norm_n: `168.213537`

| State | Samples | Mean Fz N | P95 abs Fz N | Max abs Fz N | Mean norm N | Max norm N | Min XY m | Mean XY m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| APPROACH | 1156 | -7.530820 | 75.337503 | 102.138673 | 61.212830 | 168.213537 | 0.000011 | 0.001205 |
| MOVING_TO_START | 4006 | -6.006533 | 63.272354 | 97.104421 | 52.435734 | 164.678659 | 0.000513 | 0.227128 |
| SEARCH | 3507 | -7.717954 | 75.763871 | 99.445907 | 60.025446 | 166.958833 | 0.000011 | 0.001199 |
| UNKNOWN | 275 | -1.252888 | 52.710060 | 86.085901 | 33.222583 | 136.519024 | 0.406593 | 0.409494 |

This observer is passive. It does not publish commands or alter controller behavior.
