# Wrench State Summary

- wrench_topic: `/ft_sensor_wrench`
- state_topic: `/insertion_state`
- joint_state_topic: `/joint_states`
- samples: `15266`
- duration_s: `152.650`
- max_abs_fz_n: `128.159836`
- max_force_norm_n: `207.495740`

| State | Samples | Mean Fz N | P95 abs Fz N | Max abs Fz N | Mean norm N | Max norm N | Min XY m | Mean XY m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| ABORT | 2460 | -7.600720 | 98.988523 | 126.284155 | 68.467474 | 207.495740 | 0.000148 | 0.111407 |
| APPROACH | 1140 | -9.289373 | 103.312799 | 128.159836 | 73.710884 | 206.907703 | 0.000087 | 0.002048 |
| DONE | 2655 | -2.568716 | 93.088919 | 125.915855 | 61.930990 | 182.651309 | 0.394277 | 0.409127 |
| MOVING_TO_START | 4183 | -6.473840 | 91.691996 | 122.494594 | 65.379823 | 206.224495 | 0.000091 | 0.218729 |
| SEARCH | 4500 | -7.123896 | 105.184077 | 126.482979 | 73.451561 | 204.548600 | 0.000040 | 0.002472 |
| UNKNOWN | 328 | -2.948088 | 77.767978 | 123.847121 | 32.041714 | 164.355807 | 0.405731 | 0.409543 |

This observer is passive. It does not publish commands or alter controller behavior.
