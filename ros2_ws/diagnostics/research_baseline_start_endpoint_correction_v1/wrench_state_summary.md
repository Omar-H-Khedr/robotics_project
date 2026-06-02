# Wrench State Summary

- wrench_topic: `/ft_sensor_wrench`
- state_topic: `/insertion_state`
- joint_state_topic: `/joint_states`
- samples: `8861`
- duration_s: `88.600`
- max_abs_fz_n: `174.122143`
- max_force_norm_n: `270.241130`

| State | Samples | Mean Fz N | P95 abs Fz N | Max abs Fz N | Mean norm N | Max norm N | Min XY m | Mean XY m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| ABORT | 1456 | -3.126194 | 138.020627 | 163.976334 | 96.086920 | 255.710085 | 0.001710 | 0.188488 |
| DONE | 987 | -1.916736 | 142.405965 | 158.109156 | 101.062912 | 254.906687 | 0.378642 | 0.407563 |
| MOVING_TO_START | 6147 | -6.460830 | 138.341637 | 174.122143 | 98.875654 | 270.241130 | 0.000173 | 0.150799 |
| UNKNOWN | 271 | -7.371986 | 131.847603 | 158.572525 | 61.052154 | 210.354092 | 0.398106 | 0.409190 |

This observer is passive. It does not publish commands or alter controller behavior.
