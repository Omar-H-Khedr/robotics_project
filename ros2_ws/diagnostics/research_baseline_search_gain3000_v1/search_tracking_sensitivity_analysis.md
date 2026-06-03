# Search Tracking Sensitivity Analysis

- input_dir: `diagnostics/research_baseline_search_gain3000_v1`
- command_source: `trajectory_commands.csv`
- tracking_source: `trajectory_controller_state_samples.csv`
- physical_clearance_m: `0.001000`
- hold_like_command_count: `7`
- centered_hold_like_command_count: `3`
- max_centered_hold_p95_actual_xy_drift_m: `0.004013`
- max_centered_hold_p95_joint_error_rad: `0.008314`

| Cmd | Hold | Centered target | Samples | Target XY m | Feedback mean XY m | Actual XY drift p95 m | Linearized XY drift p95 m | Residual p95 m | JTC p95 rad | Max sens p95 m/rad | Cond p95 | Dominant joint p95 m |
| ---: | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 0 | no | yes | 11140 | 0.000000 | 0.204158 | 0.004607 | 0.004607 | 0.000021 | 0.013334 | 0.847107 | 1.869 | joint_1 0.003499 |
| 1 | no | yes | 4371 | 0.000000 | 0.002121 | 0.003819 | 0.003814 | 0.000018 | 0.008261 | 0.665332 | 1.106 | joint_1 0.002714 |
| 2 | yes | yes | 1525 | 0.000001 | 0.002506 | 0.003946 | 0.003946 | 0.000020 | 0.008210 | 0.663408 | 1.098 | joint_1 0.002824 |
| 3 | yes | yes | 1524 | 0.000000 | 0.002243 | 0.003808 | 0.003810 | 0.000018 | 0.008184 | 0.654769 | 1.084 | joint_1 0.002767 |
| 4 | yes | no | 1525 | 0.003001 | 0.003822 | 0.003862 | 0.003861 | 0.000018 | 0.008449 | 0.654732 | 1.086 | joint_1 0.002714 |
| 5 | yes | no | 1525 | 0.002997 | 0.004589 | 0.003751 | 0.003746 | 0.000019 | 0.008444 | 0.655520 | 1.090 | joint_1 0.002466 |
| 6 | yes | no | 1526 | 0.003000 | 0.002970 | 0.003865 | 0.003858 | 0.000020 | 0.008211 | 0.654160 | 1.080 | joint_1 0.002595 |
| 7 | yes | no | 1524 | 0.003000 | 0.003083 | 0.003732 | 0.003724 | 0.000018 | 0.008386 | 0.654633 | 1.081 | joint_3 0.002171 |
| 8 | yes | yes | 580 | 0.000004 | 0.004653 | 0.004013 | 0.004011 | 0.000021 | 0.008314 | 0.653621 | 1.078 | joint_1 0.003006 |
| 9 | no | no | 26532 | 0.409676 | 0.340618 | 0.004359 | 0.004356 | 0.000018 | 0.011611 | 0.849056 | 1.888 | joint_1 0.003378 |

Interpretation: this passive diagnostic uses the current finite-difference peg-tip Jacobian at each controller reference. The linearized XY drift is `J_xy * (feedback - reference)`. Close agreement between actual and linearized drift means controller tracking error is sufficient to explain the Cartesian offset; a large residual points to model, frame, or log consistency issues.
