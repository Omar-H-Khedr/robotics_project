# Wrench State Summary

- wrench_topic: `/ft_sensor_wrench`
- state_topic: `/insertion_state`
- joint_state_topic: `/joint_states`
- samples: `11475`
- duration_s: `114.739`
- max_abs_fz_n: `691.368902`
- max_force_norm_n: `703.093506`

| State | Samples | Mean Fz N | P95 abs Fz N | Max abs Fz N | Mean norm N | Max norm N | Min XY m | Mean XY m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| ABORT | 1454 | -9.630076 | 136.719758 | 620.137896 | 105.430105 | 700.627400 | 0.014938 | 0.263038 |
| APPROACH | 4399 | -7.466410 | 85.618421 | 691.368902 | 80.153217 | 703.093506 | 0.000117 | 0.014789 |
| DONE | 642 | -0.987831 | 140.060646 | 170.022191 | 107.009162 | 331.088804 | 0.488714 | 0.514203 |
| MOVING_TO_START | 4702 | -6.497447 | 132.701491 | 495.064636 | 94.589704 | 509.664971 | 0.001023 | 0.257751 |
| UNKNOWN | 278 | -4.234871 | 128.660487 | 147.051600 | 71.042799 | 335.356114 | 0.498237 | 0.514788 |

This observer is passive. It does not publish commands or alter controller behavior.
