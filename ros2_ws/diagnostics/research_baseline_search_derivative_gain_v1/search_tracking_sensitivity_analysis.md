# Search Tracking Sensitivity Analysis

- input_dir: `diagnostics/research_baseline_search_derivative_gain_v1`
- command_source: `trajectory_commands.csv`
- tracking_source: `trajectory_controller_state_samples.csv`
- physical_clearance_m: `0.001000`
- hold_like_command_count: `7`
- centered_hold_like_command_count: `7`
- max_centered_hold_p95_actual_xy_drift_m: `0.003919`
- max_centered_hold_p95_joint_error_rad: `0.008617`

| Cmd | Hold | Centered target | Samples | Target XY m | Feedback mean XY m | Actual XY drift p95 m | Linearized XY drift p95 m | Residual p95 m | JTC p95 rad | Max sens p95 m/rad | Cond p95 | Dominant joint p95 m |
| ---: | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 0 | no | yes | 11057 | 0.000000 | 0.207158 | 0.004477 | 0.004479 | 0.000020 | 0.013200 | 0.848118 | 1.872 | joint_1 0.003433 |
| 1 | no | yes | 4470 | 0.000000 | 0.002159 | 0.003775 | 0.003773 | 0.000019 | 0.008337 | 0.665389 | 1.106 | joint_1 0.002600 |
| 2 | yes | yes | 1524 | 0.000001 | 0.002387 | 0.003492 | 0.003491 | 0.000016 | 0.008330 | 0.665963 | 1.109 | joint_3 0.002145 |
| 3 | yes | yes | 1525 | 0.000002 | 0.002460 | 0.003856 | 0.003853 | 0.000020 | 0.008367 | 0.654424 | 1.084 | joint_1 0.002700 |
| 4 | yes | yes | 1526 | 0.000002 | 0.002311 | 0.003705 | 0.003702 | 0.000018 | 0.008220 | 0.654534 | 1.084 | joint_1 0.002593 |
| 5 | yes | yes | 1524 | 0.000000 | 0.002638 | 0.003809 | 0.003807 | 0.000019 | 0.008453 | 0.655207 | 1.089 | joint_1 0.002662 |
| 6 | yes | yes | 1525 | 0.000000 | 0.002396 | 0.003783 | 0.003774 | 0.000019 | 0.008392 | 0.655316 | 1.089 | joint_1 0.002550 |
| 7 | yes | yes | 1525 | 0.000005 | 0.002196 | 0.003731 | 0.003731 | 0.000018 | 0.008423 | 0.655193 | 1.088 | joint_1 0.002553 |
| 8 | yes | yes | 605 | 0.000004 | 0.002692 | 0.003919 | 0.003919 | 0.000020 | 0.008617 | 0.656456 | 1.092 | joint_1 0.002790 |
| 9 | no | no | 23842 | 0.409676 | 0.331064 | 0.004320 | 0.004320 | 0.000018 | 0.011644 | 0.849056 | 1.888 | joint_1 0.003397 |

Interpretation: this passive diagnostic uses the current finite-difference peg-tip Jacobian at each controller reference. The linearized XY drift is `J_xy * (feedback - reference)`. Close agreement between actual and linearized drift means controller tracking error is sufficient to explain the Cartesian offset; a large residual points to model, frame, or log consistency issues.
