# Wrench State Summary

- wrench_topic: `/ft_sensor_wrench`
- state_topic: `/insertion_state`
- joint_state_topic: `/joint_states`
- samples: `11036`
- duration_s: `110.349`
- max_abs_fz_n: `128.114936`
- max_force_norm_n: `208.495440`

| State | Samples | Mean Fz N | P95 abs Fz N | Max abs Fz N | Mean norm N | Max norm N | Min XY m | Mean XY m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| APPROACH | 1188 | -7.792819 | 106.019960 | 125.802257 | 76.304255 | 198.884872 | 0.000053 | 0.002141 |
| DONE | 1515 | -4.413129 | 91.692929 | 127.133583 | 61.660889 | 181.336821 | 0.396419 | 0.409109 |
| INSERT | 1059 | -7.080454 | 103.368585 | 124.064036 | 72.685244 | 202.630104 | 0.000065 | 0.002599 |
| MOVING_TO_START | 4532 | -5.287462 | 92.880851 | 127.919570 | 65.645307 | 208.495440 | 0.000096 | 0.199719 |
| RETREAT | 2462 | -7.798190 | 99.926388 | 128.114936 | 69.228231 | 200.880897 | 0.000196 | 0.105161 |
| SEARCH | 5 | -14.655771 | 74.433305 | 74.433305 | 72.772729 | 128.896468 | 0.000709 | 0.001753 |
| UNKNOWN | 275 | -5.112571 | 90.086943 | 116.648061 | 39.064386 | 167.972038 | 0.405010 | 0.409549 |

This observer is passive. It does not publish commands or alter controller behavior.
