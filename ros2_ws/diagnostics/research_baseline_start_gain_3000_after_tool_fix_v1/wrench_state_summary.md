# Wrench State Summary

- wrench_topic: `/ft_sensor_wrench`
- state_topic: `/insertion_state`
- joint_state_topic: `/joint_states`
- samples: `11709`
- duration_s: `117.078`
- max_abs_fz_n: `174.243262`
- max_force_norm_n: `273.601799`

| State | Samples | Mean Fz N | P95 abs Fz N | Max abs Fz N | Mean norm N | Max norm N | Min XY m | Mean XY m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| ABORT | 1459 | -6.932645 | 137.907305 | 174.243262 | 96.671292 | 273.601799 | 0.000576 | 0.186800 |
| DONE | 3880 | -4.139953 | 141.744610 | 159.224816 | 100.109267 | 272.836700 | 0.382366 | 0.407560 |
| MOVING_TO_START | 6093 | -5.563626 | 139.434975 | 170.613232 | 99.375135 | 272.380062 | 0.000034 | 0.146062 |
| UNKNOWN | 277 | -0.547775 | 134.638563 | 151.142062 | 59.064514 | 208.660735 | 0.387171 | 0.407577 |

This observer is passive. It does not publish commands or alter controller behavior.
