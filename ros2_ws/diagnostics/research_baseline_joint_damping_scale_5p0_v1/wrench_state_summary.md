# Wrench State Summary

- wrench_topic: `/ft_sensor_wrench`
- state_topic: `/insertion_state`
- joint_state_topic: `/joint_states`
- samples: `8399`
- duration_s: `83.979`
- max_abs_fz_n: `129.841765`
- max_force_norm_n: `209.198759`

| State | Samples | Mean Fz N | P95 abs Fz N | Max abs Fz N | Mean norm N | Max norm N | Min XY m | Mean XY m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| ABORT | 1459 | -4.480421 | 92.242851 | 124.792615 | 66.778475 | 205.884646 | 0.000515 | 0.188543 |
| APPROACH | 944 | -7.304691 | 102.769918 | 129.841765 | 74.832831 | 209.198759 | 0.000101 | 0.002096 |
| DONE | 1659 | -3.408253 | 91.398275 | 126.352008 | 62.888700 | 182.457239 | 0.396747 | 0.409131 |
| MOVING_TO_START | 4099 | -7.075720 | 92.757655 | 126.101113 | 65.160028 | 204.823121 | 0.000145 | 0.221761 |
| UNKNOWN | 238 | -5.093494 | 91.520306 | 120.344140 | 49.978590 | 186.892751 | 0.404703 | 0.409248 |

This observer is passive. It does not publish commands or alter controller behavior.
