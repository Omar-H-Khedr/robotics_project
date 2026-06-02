# Wrench State Summary

- wrench_topic: `/ft_sensor_wrench`
- state_topic: `/insertion_state`
- joint_state_topic: `/joint_states`
- samples: `11763`
- duration_s: `117.619`
- max_abs_fz_n: `172.339403`
- max_force_norm_n: `272.660305`

| State | Samples | Mean Fz N | P95 abs Fz N | Max abs Fz N | Mean norm N | Max norm N | Min XY m | Mean XY m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| ABORT | 1465 | -3.904441 | 137.967080 | 171.760201 | 96.914651 | 257.664992 | 0.000469 | 0.189032 |
| DONE | 3869 | -3.119526 | 142.304754 | 159.993211 | 101.057183 | 261.327397 | 0.384750 | 0.407621 |
| IDLE | 91 | -5.999526 | 145.646524 | 153.119003 | 105.899503 | 262.254659 | 0.393022 | 0.408363 |
| MOVING_TO_START | 6201 | -8.116186 | 139.436902 | 172.339403 | 98.694918 | 272.660305 | 0.000455 | 0.146921 |
| UNKNOWN | 137 | -4.539379 | 118.739473 | 158.420427 | 33.623415 | 226.098470 | 0.402297 | 0.409333 |

This observer is passive. It does not publish commands or alter controller behavior.
