# Wrench State Summary

- wrench_topic: `/ft_sensor_wrench`
- state_topic: `/insertion_state`
- joint_state_topic: `/joint_states`
- samples: `19724`
- duration_s: `197.230`
- max_abs_fz_n: `127.864342`
- max_force_norm_n: `207.493460`

| State | Samples | Mean Fz N | P95 abs Fz N | Max abs Fz N | Mean norm N | Max norm N | Min XY m | Mean XY m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| ABORT | 1160 | -5.434577 | 100.831889 | 123.103783 | 69.500168 | 197.171792 | 0.000062 | 0.117114 |
| APPROACH | 590 | -6.041868 | 103.252942 | 127.864342 | 76.787968 | 207.493460 | 0.000243 | 0.002414 |
| DONE | 10433 | -2.862061 | 93.235621 | 126.156406 | 62.714075 | 190.179825 | 0.397488 | 0.409276 |
| MOVING_TO_START | 2780 | -5.084963 | 95.415219 | 127.350837 | 67.578910 | 206.849456 | 0.000060 | 0.158251 |
| SEARCH | 4500 | -7.837746 | 104.360379 | 124.676755 | 73.852802 | 204.354909 | 0.000038 | 0.002285 |
| UNKNOWN | 261 | -4.184821 | 80.949731 | 113.079614 | 37.078318 | 169.741311 | 0.404076 | 0.409344 |

This observer is passive. It does not publish commands or alter controller behavior.
