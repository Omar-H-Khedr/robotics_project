# Search Tracking Sensitivity Analysis

- input_dir: `diagnostics/research_baseline_search_velocity_state_v1`
- command_source: `trajectory_commands.csv`
- tracking_source: `trajectory_controller_state_samples.csv`
- physical_clearance_m: `0.001000`
- hold_like_command_count: `7`
- centered_hold_like_command_count: `7`
- max_centered_hold_p95_actual_xy_drift_m: `0.004015`
- max_centered_hold_p95_joint_error_rad: `0.008408`

| Cmd | Hold | Centered target | Samples | Target XY m | Feedback mean XY m | Actual XY drift p95 m | Linearized XY drift p95 m | Residual p95 m | JTC p95 rad | Max sens p95 m/rad | Cond p95 | Dominant joint p95 m |
| ---: | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 0 | no | yes | 11400 | 0.000000 | 0.198683 | 0.004481 | 0.004480 | 0.000020 | 0.013169 | 0.846274 | 1.855 | joint_1 0.003443 |
| 1 | no | yes | 4497 | 0.000000 | 0.002145 | 0.003795 | 0.003793 | 0.000019 | 0.008279 | 0.665173 | 1.105 | joint_1 0.002667 |
| 2 | yes | yes | 1525 | 0.000001 | 0.002812 | 0.004014 | 0.004011 | 0.000020 | 0.008341 | 0.664821 | 1.106 | joint_1 0.002818 |
| 3 | yes | yes | 1549 | 0.000005 | 0.002192 | 0.004015 | 0.004003 | 0.000020 | 0.008366 | 0.654115 | 1.081 | joint_1 0.002669 |
| 4 | yes | yes | 1575 | 0.000007 | 0.002909 | 0.003788 | 0.003790 | 0.000017 | 0.008408 | 0.656171 | 1.090 | joint_1 0.002656 |
| 5 | yes | yes | 1524 | 0.000004 | 0.001952 | 0.003630 | 0.003616 | 0.000018 | 0.008384 | 0.654559 | 1.083 | joint_1 0.002346 |
| 6 | yes | yes | 1525 | 0.000005 | 0.002018 | 0.003638 | 0.003634 | 0.000016 | 0.008246 | 0.655925 | 1.086 | joint_1 0.002389 |
| 7 | yes | yes | 1525 | 0.000003 | 0.002170 | 0.003825 | 0.003824 | 0.000019 | 0.008386 | 0.654548 | 1.083 | joint_1 0.002750 |
| 8 | yes | yes | 530 | 0.000006 | 0.002398 | 0.003790 | 0.003791 | 0.000017 | 0.008394 | 0.654890 | 1.085 | joint_1 0.002628 |
| 9 | no | no | 26529 | 0.409676 | 0.340468 | 0.004436 | 0.004434 | 0.000019 | 0.011678 | 0.849056 | 1.888 | joint_1 0.003459 |

Interpretation: this passive diagnostic uses the current finite-difference peg-tip Jacobian at each controller reference. The linearized XY drift is `J_xy * (feedback - reference)`. Close agreement between actual and linearized drift means controller tracking error is sufficient to explain the Cartesian offset; a large residual points to model, frame, or log consistency issues.
