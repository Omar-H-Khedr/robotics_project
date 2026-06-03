# Wrench State Summary

- wrench_topic: `/ft_sensor_wrench`
- state_topic: `/insertion_state`
- joint_state_topic: `/joint_states`
- samples: `20266`
- duration_s: `202.650`
- max_abs_fz_n: `128.176814`
- max_force_norm_n: `208.407159`

| State | Samples | Mean Fz N | P95 abs Fz N | Max abs Fz N | Mean norm N | Max norm N | Min XY m | Mean XY m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| ABORT | 2470 | -6.112866 | 101.540096 | 128.176814 | 68.951942 | 208.407159 | 0.000242 | 0.112434 |
| APPROACH | 1210 | -6.393131 | 102.839840 | 127.694465 | 74.950155 | 204.571985 | 0.000019 | 0.002380 |
| DONE | 7845 | -2.962072 | 92.909142 | 127.245152 | 62.882214 | 188.348029 | 0.397873 | 0.409243 |
| IDLE | 62 | -3.268463 | 83.777117 | 104.495502 | 65.690433 | 167.167527 | 0.405786 | 0.409302 |
| MOVING_TO_START | 4040 | -5.244640 | 89.988082 | 124.153562 | 64.425558 | 208.070854 | 0.000422 | 0.228276 |
| SEARCH | 4500 | -8.002574 | 103.763858 | 127.230016 | 73.321755 | 202.528409 | 0.000043 | 0.002393 |
| UNKNOWN | 139 | -3.730383 | 75.739832 | 102.095992 | 20.783535 | 173.266874 | 0.406939 | 0.409638 |

This observer is passive. It does not publish commands or alter controller behavior.
