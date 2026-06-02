# Wrench State Summary

- wrench_topic: `/ft_sensor_wrench`
- state_topic: `/insertion_state`
- joint_state_topic: `/joint_states`
- samples: `7228`
- duration_s: `72.269`
- max_abs_fz_n: `1396.750557`
- max_force_norm_n: `2624.114841`

| State | Samples | Mean Fz N | P95 abs Fz N | Max abs Fz N | Mean norm N | Max norm N | Min XY m | Mean XY m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| ABORT | 584 | 3.633091 | 145.355835 | 162.147994 | 113.664342 | 329.775965 | 0.000026 | 0.169895 |
| DONE | 4366 | -3.833366 | 140.291188 | 177.702280 | 110.663748 | 330.596500 | 0.448511 | 0.514441 |
| IDLE | 91 | -1.638084 | 130.495068 | 140.228240 | 108.504725 | 316.051832 | 0.502041 | 0.515082 |
| MOVING_TO_START | 2039 | 1.307826 | 139.855167 | 1396.750557 | 111.724142 | 2624.114841 | 0.000697 | 0.272858 |
| UNKNOWN | 148 | -3.556448 | 85.382598 | 155.011440 | 28.155984 | 286.303820 | 0.510453 | 0.516348 |

This observer is passive. It does not publish commands or alter controller behavior.
