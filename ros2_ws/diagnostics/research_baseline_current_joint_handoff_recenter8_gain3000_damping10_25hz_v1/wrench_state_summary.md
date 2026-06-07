# Wrench State Summary

- wrench_topic: `/ft_sensor_wrench`
- state_topic: `/insertion_state`
- joint_state_topic: `/joint_states`
- samples: `10498`
- duration_s: `104.969`
- max_abs_fz_n: `101.522856`
- max_force_norm_n: `171.985291`

| State | Samples | Mean Fz N | P95 abs Fz N | Max abs Fz N | Mean norm N | Max norm N | Min XY m | Mean XY m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| ABORT | 545 | -7.784331 | 76.554432 | 90.392917 | 58.313718 | 163.461072 | 0.000147 | 0.001853 |
| APPROACH | 1152 | -7.733336 | 76.820888 | 101.522856 | 61.277935 | 171.985291 | 0.000040 | 0.001199 |
| MOVING_TO_START | 4004 | -5.746231 | 61.025659 | 98.003841 | 51.901809 | 164.004910 | 0.000371 | 0.227058 |
| SEARCH | 4500 | -7.429412 | 76.720726 | 100.018836 | 60.027862 | 167.948616 | 0.000007 | 0.001326 |
| UNKNOWN | 297 | -1.957539 | 51.514509 | 80.749547 | 31.844940 | 136.132003 | 0.407241 | 0.409532 |

This observer is passive. It does not publish commands or alter controller behavior.
