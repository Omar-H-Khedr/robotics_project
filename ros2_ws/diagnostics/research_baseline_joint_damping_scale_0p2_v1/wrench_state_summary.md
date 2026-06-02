# Wrench State Summary

- wrench_topic: `/ft_sensor_wrench`
- state_topic: `/insertion_state`
- joint_state_topic: `/joint_states`
- samples: `10528`
- duration_s: `105.269`
- max_abs_fz_n: `1181.037416`
- max_force_norm_n: `1272.736100`

| State | Samples | Mean Fz N | P95 abs Fz N | Max abs Fz N | Mean norm N | Max norm N | Min XY m | Mean XY m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| ABORT | 1143 | -5.065681 | 118.447173 | 826.893482 | 105.222882 | 843.497886 | 0.230733 | 0.372276 |
| DONE | 6986 | -1.367135 | 119.686972 | 131.097831 | 112.110163 | 359.383949 | 0.423645 | 0.508739 |
| IDLE | 79 | -17.282712 | 115.384918 | 126.407216 | 116.337738 | 301.233321 | 0.482362 | 0.507503 |
| MOVING_TO_START | 2082 | -1.746131 | 117.957472 | 1181.037416 | 104.383960 | 1272.736100 | 0.251293 | 0.393503 |
| UNKNOWN | 238 | -1.448681 | 106.928327 | 128.776406 | 30.138726 | 290.187097 | 0.500700 | 0.515751 |

This observer is passive. It does not publish commands or alter controller behavior.
