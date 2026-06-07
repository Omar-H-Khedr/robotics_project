# Wrench State Summary

- wrench_topic: `/ft_sensor_wrench`
- state_topic: `/insertion_state`
- joint_state_topic: `/joint_states`
- samples: `9160`
- duration_s: `91.588`
- max_abs_fz_n: `103.006042`
- max_force_norm_n: `169.947388`

| State | Samples | Mean Fz N | P95 abs Fz N | Max abs Fz N | Mean norm N | Max norm N | Min XY m | Mean XY m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| ABORT | 2452 | -7.134270 | 70.069661 | 98.541263 | 55.893058 | 169.947388 | 0.000051 | 0.126029 |
| APPROACH | 1176 | -8.198031 | 77.151754 | 103.006042 | 61.616555 | 169.207341 | 0.000067 | 0.001273 |
| DONE | 59 | -2.037743 | 60.482908 | 73.832006 | 45.031436 | 92.625169 | 0.395991 | 0.404245 |
| INSERT | 1208 | -8.353640 | 77.525337 | 98.481520 | 59.978931 | 165.547486 | 0.000054 | 0.001216 |
| MOVING_TO_START | 4005 | -5.157632 | 63.108887 | 99.590635 | 53.113162 | 165.076430 | 0.000442 | 0.226831 |
| UNKNOWN | 260 | -2.120673 | 48.825252 | 94.919056 | 23.170929 | 126.262427 | 0.407158 | 0.409612 |

This observer is passive. It does not publish commands or alter controller behavior.
