# Wrench State Summary

- wrench_topic: `/ft_sensor_wrench`
- state_topic: `/insertion_state`
- joint_state_topic: `/joint_states`
- samples: `10448`
- duration_s: `104.469`
- max_abs_fz_n: `99.988675`
- max_force_norm_n: `172.723827`

| State | Samples | Mean Fz N | P95 abs Fz N | Max abs Fz N | Mean norm N | Max norm N | Min XY m | Mean XY m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| APPROACH | 1168 | -7.837555 | 78.190269 | 96.084818 | 83.251286 | 172.723827 | 0.000012 | 0.000462 |
| DONE | 43 | -2.493239 | 70.424404 | 88.393684 | 66.309867 | 150.941769 | 0.398123 | 0.404654 |
| INSERT | 2536 | -7.028863 | 77.687625 | 93.509973 | 80.681681 | 164.663021 | 0.000011 | 0.000639 |
| MOVING_TO_START | 4006 | -5.696250 | 67.724800 | 93.717280 | 72.065616 | 166.781173 | 0.000234 | 0.228773 |
| RETREAT | 2468 | -7.062596 | 75.463568 | 99.988675 | 77.251582 | 167.708860 | 0.000003 | 0.094073 |
| UNKNOWN | 227 | -1.070072 | 65.018643 | 87.998227 | 48.895327 | 143.056690 | 0.408480 | 0.409589 |

This observer is passive. It does not publish commands or alter controller behavior.
