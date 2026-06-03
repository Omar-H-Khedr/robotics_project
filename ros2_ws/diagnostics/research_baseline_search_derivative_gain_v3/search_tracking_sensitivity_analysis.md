# Search Tracking Sensitivity Analysis

- input_dir: `diagnostics/research_baseline_search_derivative_gain_v3`
- command_source: `trajectory_commands.csv`
- tracking_source: `trajectory_controller_state_samples.csv`
- physical_clearance_m: `0.001000`
- hold_like_command_count: `7`
- centered_hold_like_command_count: `7`
- max_centered_hold_p95_actual_xy_drift_m: `0.004029`
- max_centered_hold_p95_joint_error_rad: `0.008481`

| Cmd | Hold | Centered target | Samples | Target XY m | Feedback mean XY m | Actual XY drift p95 m | Linearized XY drift p95 m | Residual p95 m | JTC p95 rad | Max sens p95 m/rad | Cond p95 | Dominant joint p95 m |
| ---: | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 0 | no | yes | 10100 | 0.000000 | 0.227235 | 0.004569 | 0.004563 | 0.000020 | 0.013323 | 0.850068 | 1.885 | joint_1 0.003487 |
| 1 | no | yes | 4525 | 0.000000 | 0.002310 | 0.003850 | 0.003845 | 0.000018 | 0.008417 | 0.664809 | 1.105 | joint_1 0.002592 |
| 2 | yes | yes | 1524 | 0.000001 | 0.002387 | 0.003889 | 0.003877 | 0.000019 | 0.008259 | 0.663646 | 1.099 | joint_1 0.002873 |
| 3 | yes | yes | 1526 | 0.000000 | 0.002450 | 0.003801 | 0.003799 | 0.000018 | 0.008306 | 0.654310 | 1.082 | joint_1 0.002713 |
| 4 | yes | yes | 1524 | 0.000000 | 0.002708 | 0.003806 | 0.003807 | 0.000017 | 0.008299 | 0.654487 | 1.082 | joint_1 0.002589 |
| 5 | yes | yes | 1550 | 0.000000 | 0.002512 | 0.003542 | 0.003545 | 0.000017 | 0.008481 | 0.655456 | 1.089 | joint_1 0.002391 |
| 6 | yes | yes | 1526 | 0.000009 | 0.002220 | 0.004029 | 0.004017 | 0.000020 | 0.008282 | 0.654635 | 1.084 | joint_1 0.002676 |
| 7 | yes | yes | 1524 | 0.000004 | 0.002481 | 0.003845 | 0.003837 | 0.000019 | 0.008237 | 0.655689 | 1.089 | joint_1 0.002769 |
| 8 | yes | yes | 581 | 0.000007 | 0.002221 | 0.003563 | 0.003555 | 0.000016 | 0.008382 | 0.654660 | 1.087 | joint_1 0.002263 |
| 9 | no | no | 25758 | 0.409676 | 0.338583 | 0.004346 | 0.004346 | 0.000019 | 0.011757 | 0.849056 | 1.888 | joint_1 0.003354 |

Interpretation: this passive diagnostic uses the current finite-difference peg-tip Jacobian at each controller reference. The linearized XY drift is `J_xy * (feedback - reference)`. Close agreement between actual and linearized drift means controller tracking error is sufficient to explain the Cartesian offset; a large residual points to model, frame, or log consistency issues.
