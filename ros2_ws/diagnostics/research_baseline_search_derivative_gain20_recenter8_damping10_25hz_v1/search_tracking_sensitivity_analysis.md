# Search Tracking Sensitivity Analysis

- input_dir: `diagnostics/research_baseline_search_derivative_gain20_recenter8_damping10_25hz_v1`
- command_source: `trajectory_commands.csv`
- tracking_source: `trajectory_controller_state_samples.csv`
- physical_clearance_m: `0.001000`
- hold_like_command_count: `4`
- centered_hold_like_command_count: `4`
- max_centered_hold_p95_actual_xy_drift_m: `0.002337`
- max_centered_hold_p95_joint_error_rad: `0.007646`

| Cmd | Hold | Centered target | Samples | Target XY m | Feedback mean XY m | Actual XY drift p95 m | Linearized XY drift p95 m | Residual p95 m | JTC p95 rad | Max sens p95 m/rad | Cond p95 | Dominant joint p95 m |
| ---: | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 0 | no | yes | 10038 | 0.000000 | 0.226822 | 0.002801 | 0.002800 | 0.000010 | 0.013056 | 0.849380 | 1.882 | joint_5 0.001785 |
| 1 | no | yes | 5167 | 0.000000 | 0.001219 | 0.002233 | 0.002232 | 0.000007 | 0.007564 | 0.664965 | 1.105 | joint_1 0.001363 |
| 2 | yes | yes | 2259 | 0.000001 | 0.001391 | 0.002328 | 0.002327 | 0.000007 | 0.007615 | 0.665447 | 1.108 | joint_1 0.001475 |
| 3 | yes | yes | 2260 | 0.000002 | 0.001187 | 0.002337 | 0.002337 | 0.000007 | 0.007551 | 0.654190 | 1.081 | joint_1 0.001450 |
| 4 | yes | yes | 2300 | 0.000002 | 0.001238 | 0.002333 | 0.002333 | 0.000007 | 0.007646 | 0.654023 | 1.082 | joint_1 0.001477 |
| 5 | yes | yes | 2174 | 0.000003 | 0.001244 | 0.002280 | 0.002279 | 0.000007 | 0.007632 | 0.654090 | 1.081 | joint_1 0.001360 |
| 6 | no | no | 6273 | 0.409676 | 0.125139 | 0.002653 | 0.002652 | 0.000009 | 0.011833 | 0.845783 | 1.727 | joint_1 0.001735 |

Interpretation: this passive diagnostic uses the current finite-difference peg-tip Jacobian at each controller reference. The linearized XY drift is `J_xy * (feedback - reference)`. Close agreement between actual and linearized drift means controller tracking error is sufficient to explain the Cartesian offset; a large residual points to model, frame, or log consistency issues.
