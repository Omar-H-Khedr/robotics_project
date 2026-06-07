# Wrench State Summary

- wrench_topic: `/ft_sensor_wrench`
- state_topic: `/insertion_state`
- joint_state_topic: `/joint_states`
- samples: `9115`
- duration_s: `91.138`
- max_abs_fz_n: `101.549897`
- max_force_norm_n: `170.631857`

| State | Samples | Mean Fz N | P95 abs Fz N | Max abs Fz N | Mean norm N | Max norm N | Min XY m | Mean XY m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| ABORT | 2451 | -6.544950 | 71.582471 | 98.122706 | 55.204103 | 170.631857 | 0.000073 | 0.135116 |
| APPROACH | 1168 | -8.346057 | 77.428541 | 96.421805 | 61.438414 | 167.411105 | 0.000041 | 0.001142 |
| DONE | 59 | -2.135936 | 63.870211 | 70.355908 | 47.186667 | 133.475880 | 0.397842 | 0.404378 |
| IDLE | 46 | 2.661119 | 78.929966 | 79.393088 | 52.979951 | 139.921301 | 0.406737 | 0.409257 |
| INSERT | 1204 | -8.021431 | 76.842836 | 97.403532 | 59.944032 | 167.675802 | 0.000036 | 0.001165 |
| MOVING_TO_START | 4024 | -5.702179 | 65.143685 | 101.549897 | 53.105280 | 168.317057 | 0.000124 | 0.226686 |
| UNKNOWN | 163 | -1.440240 | 40.178247 | 88.343285 | 19.823589 | 114.847866 | 0.407594 | 0.409621 |

This observer is passive. It does not publish commands or alter controller behavior.
