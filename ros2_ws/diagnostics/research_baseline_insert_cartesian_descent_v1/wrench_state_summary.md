# Wrench State Summary

- wrench_topic: `/ft_sensor_wrench`
- state_topic: `/insertion_state`
- joint_state_topic: `/joint_states`
- samples: `12353`
- duration_s: `123.519`
- max_abs_fz_n: `130.638117`
- max_force_norm_n: `210.620984`

| State | Samples | Mean Fz N | P95 abs Fz N | Max abs Fz N | Mean norm N | Max norm N | Min XY m | Mean XY m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| ABORT | 2470 | -6.754849 | 98.679042 | 130.547196 | 68.430041 | 205.964963 | 0.000164 | 0.119145 |
| APPROACH | 1170 | -5.442391 | 102.769585 | 130.638117 | 76.166471 | 210.620984 | 0.000049 | 0.002107 |
| DONE | 3692 | -2.809055 | 91.545653 | 128.391794 | 62.585563 | 187.167429 | 0.398346 | 0.409218 |
| INSERT | 30 | -5.795139 | 105.664686 | 110.301512 | 71.920959 | 175.470887 | 0.000382 | 0.001597 |
| MOVING_TO_START | 4296 | -5.255020 | 91.274291 | 129.565946 | 66.197208 | 199.660665 | 0.000048 | 0.211179 |
| SEARCH | 390 | -0.873206 | 101.475256 | 125.325476 | 70.947298 | 201.486693 | 0.000119 | 0.002039 |
| UNKNOWN | 305 | 0.213064 | 91.608352 | 111.241309 | 40.916665 | 177.865051 | 0.405815 | 0.409622 |

This observer is passive. It does not publish commands or alter controller behavior.
