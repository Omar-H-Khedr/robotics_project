# Wrench State Summary

- wrench_topic: `/ft_sensor_wrench`
- state_topic: `/insertion_state`
- joint_state_topic: `/joint_states`
- samples: `12428`
- duration_s: `124.269`
- max_abs_fz_n: `102.834283`
- max_force_norm_n: `168.280969`

| State | Samples | Mean Fz N | P95 abs Fz N | Max abs Fz N | Mean norm N | Max norm N | Min XY m | Mean XY m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| ABORT | 2460 | -6.915283 | 71.726281 | 102.834283 | 56.523877 | 167.245319 | 0.000028 | 0.118640 |
| APPROACH | 1140 | -7.762428 | 76.876944 | 99.130204 | 60.720943 | 168.280969 | 0.000050 | 0.001228 |
| DONE | 59 | -6.039504 | 67.757999 | 80.463243 | 49.422426 | 138.013853 | 0.397130 | 0.405528 |
| MOVING_TO_START | 3996 | -5.668812 | 61.321491 | 100.690959 | 52.315264 | 167.067943 | 0.000397 | 0.227759 |
| SEARCH | 4500 | -7.462221 | 77.239308 | 99.697491 | 60.671675 | 166.720655 | 0.000012 | 0.001286 |
| UNKNOWN | 273 | -2.940098 | 56.709515 | 84.786063 | 30.071433 | 125.927799 | 0.407252 | 0.409514 |

This observer is passive. It does not publish commands or alter controller behavior.
