# Wrench State Summary

- wrench_topic: `/ft_sensor_wrench`
- state_topic: `/insertion_state`
- joint_state_topic: `/joint_states`
- samples: `11004`
- duration_s: `110.029`
- max_abs_fz_n: `98.873090`
- max_force_norm_n: `171.072691`

| State | Samples | Mean Fz N | P95 abs Fz N | Max abs Fz N | Mean norm N | Max norm N | Min XY m | Mean XY m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| APPROACH | 1172 | -7.423771 | 77.784703 | 94.814824 | 80.661943 | 171.072691 | 0.000010 | 0.000490 |
| DONE | 59 | -5.554344 | 64.404971 | 77.450486 | 60.027132 | 138.103907 | 0.398619 | 0.406279 |
| INSERT | 3040 | -7.346991 | 79.414852 | 94.642172 | 80.919435 | 166.260221 | 0.000011 | 0.000575 |
| MOVING_TO_START | 4000 | -5.549830 | 67.616574 | 95.655465 | 72.563796 | 167.410118 | 0.000173 | 0.228515 |
| RETREAT | 2468 | -5.861913 | 74.895572 | 98.873090 | 78.142552 | 169.332907 | 0.000019 | 0.094229 |
| UNKNOWN | 265 | -2.584406 | 62.307016 | 87.039509 | 40.762133 | 135.435213 | 0.408097 | 0.409629 |

This observer is passive. It does not publish commands or alter controller behavior.
