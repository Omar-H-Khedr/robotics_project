# Wrench State Summary

- wrench_topic: `/ft_sensor_wrench`
- state_topic: `/insertion_state`
- joint_state_topic: `/joint_states`
- samples: `20348`
- duration_s: `203.469`
- max_abs_fz_n: `132.211485`
- max_force_norm_n: `211.531894`

| State | Samples | Mean Fz N | P95 abs Fz N | Max abs Fz N | Mean norm N | Max norm N | Min XY m | Mean XY m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| ABORT | 2460 | -5.071818 | 100.579665 | 126.882208 | 69.379542 | 205.948229 | 0.000069 | 0.125915 |
| APPROACH | 1231 | -8.542516 | 100.934791 | 130.373732 | 73.935523 | 206.091853 | 0.000087 | 0.002042 |
| DONE | 11077 | -3.193973 | 92.599134 | 126.705733 | 62.843116 | 190.991374 | 0.393567 | 0.409253 |
| INSERT | 40 | -9.639093 | 93.032566 | 99.779883 | 65.445496 | 186.953012 | 0.000254 | 0.001813 |
| MOVING_TO_START | 5279 | -5.520078 | 95.059449 | 132.211485 | 66.694206 | 211.531894 | 0.000081 | 0.175671 |
| UNKNOWN | 261 | -0.762062 | 78.140081 | 109.127047 | 30.077430 | 182.006796 | 0.405163 | 0.409578 |

This observer is passive. It does not publish commands or alter controller behavior.
