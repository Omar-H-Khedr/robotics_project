# Wrench State Summary

- wrench_topic: `/ft_sensor_wrench`
- state_topic: `/insertion_state`
- joint_state_topic: `/joint_states`
- samples: `8285`
- duration_s: `82.839`
- max_abs_fz_n: `96.580254`
- max_force_norm_n: `171.396990`

| State | Samples | Mean Fz N | P95 abs Fz N | Max abs Fz N | Mean norm N | Max norm N | Min XY m | Mean XY m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| ABORT | 2452 | -7.108296 | 74.800437 | 96.580254 | 77.097237 | 169.265820 | 0.000032 | 0.125823 |
| APPROACH | 1176 | -7.425756 | 77.492674 | 95.415632 | 83.522821 | 168.614552 | 0.000018 | 0.000497 |
| DONE | 48 | -1.805642 | 68.841702 | 84.698540 | 59.186831 | 134.428141 | 0.397987 | 0.403586 |
| INSERT | 288 | -12.157391 | 78.275596 | 89.952105 | 79.631820 | 165.988235 | 0.000020 | 0.000540 |
| MOVING_TO_START | 4005 | -5.581664 | 66.864402 | 95.864557 | 72.954619 | 171.396990 | 0.000235 | 0.228431 |
| UNKNOWN | 316 | -2.838195 | 60.852308 | 79.520565 | 39.218144 | 138.596624 | 0.408590 | 0.409641 |

This observer is passive. It does not publish commands or alter controller behavior.
