# Wrench State Summary

- wrench_topic: `/ft_sensor_wrench`
- state_topic: `/insertion_state`
- joint_state_topic: `/joint_states`
- samples: `19899`
- duration_s: `198.979`
- max_abs_fz_n: `128.297002`
- max_force_norm_n: `206.505362`

| State | Samples | Mean Fz N | P95 abs Fz N | Max abs Fz N | Mean norm N | Max norm N | Min XY m | Mean XY m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| ABORT | 2470 | -7.993837 | 102.119193 | 128.297002 | 70.250756 | 206.505362 | 0.000002 | 0.105346 |
| APPROACH | 1190 | -6.729972 | 101.830734 | 126.890710 | 73.942940 | 204.644344 | 0.000082 | 0.002151 |
| DONE | 7078 | -2.631435 | 91.534906 | 126.268250 | 62.927611 | 188.020759 | 0.395799 | 0.409263 |
| MOVING_TO_START | 4400 | -5.548952 | 92.978886 | 125.576578 | 65.365982 | 201.488933 | 0.000153 | 0.207250 |
| SEARCH | 4500 | -9.433118 | 103.672399 | 126.447939 | 73.382288 | 204.098132 | 0.000008 | 0.002375 |
| UNKNOWN | 261 | -3.074148 | 82.980101 | 111.697345 | 39.055779 | 170.511914 | 0.405304 | 0.409459 |

This observer is passive. It does not publish commands or alter controller behavior.
