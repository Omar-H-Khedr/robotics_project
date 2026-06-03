# Search Tracking Sensitivity Analysis

- input_dir: `diagnostics/research_baseline_search_derivative_gain_v4`
- command_source: `trajectory_commands.csv`
- tracking_source: `trajectory_controller_state_samples.csv`
- physical_clearance_m: `0.001000`
- hold_like_command_count: `7`
- centered_hold_like_command_count: `7`
- max_centered_hold_p95_actual_xy_drift_m: `0.004123`
- max_centered_hold_p95_joint_error_rad: `0.008472`

| Cmd | Hold | Centered target | Samples | Target XY m | Feedback mean XY m | Actual XY drift p95 m | Linearized XY drift p95 m | Residual p95 m | JTC p95 rad | Max sens p95 m/rad | Cond p95 | Dominant joint p95 m |
| ---: | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 0 | no | yes | 10180 | 0.000000 | 0.223490 | 0.004592 | 0.004587 | 0.000021 | 0.013526 | 0.847142 | 1.877 | joint_1 0.003496 |
| 1 | no | yes | 4446 | 0.000000 | 0.002207 | 0.003894 | 0.003894 | 0.000019 | 0.008370 | 0.665303 | 1.106 | joint_1 0.002789 |
| 2 | yes | yes | 1526 | 0.000001 | 0.002686 | 0.003896 | 0.003887 | 0.000017 | 0.008256 | 0.665130 | 1.107 | joint_1 0.002743 |
| 3 | yes | yes | 1524 | 0.000009 | 0.002223 | 0.003912 | 0.003913 | 0.000020 | 0.008324 | 0.654662 | 1.084 | joint_1 0.002647 |
| 4 | yes | yes | 1526 | 0.000006 | 0.002317 | 0.004123 | 0.004123 | 0.000020 | 0.008472 | 0.654429 | 1.083 | joint_1 0.003160 |
| 5 | yes | yes | 1524 | 0.000004 | 0.002617 | 0.003682 | 0.003684 | 0.000018 | 0.008375 | 0.655103 | 1.084 | joint_1 0.002589 |
| 6 | yes | yes | 1550 | 0.000002 | 0.002709 | 0.003860 | 0.003860 | 0.000020 | 0.008316 | 0.654627 | 1.086 | joint_1 0.002710 |
| 7 | yes | yes | 1550 | 0.000002 | 0.002333 | 0.003800 | 0.003793 | 0.000019 | 0.008438 | 0.655850 | 1.088 | joint_1 0.002690 |
| 8 | yes | yes | 555 | 0.000003 | 0.003818 | 0.003695 | 0.003683 | 0.000018 | 0.008472 | 0.656062 | 1.090 | joint_3 0.002125 |
| 9 | no | no | 25605 | 0.409676 | 0.337944 | 0.004310 | 0.004311 | 0.000018 | 0.011697 | 0.849056 | 1.888 | joint_1 0.003369 |

Interpretation: this passive diagnostic uses the current finite-difference peg-tip Jacobian at each controller reference. The linearized XY drift is `J_xy * (feedback - reference)`. Close agreement between actual and linearized drift means controller tracking error is sufficient to explain the Cartesian offset; a large residual points to model, frame, or log consistency issues.
