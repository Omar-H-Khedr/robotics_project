# Search Tracking Sensitivity Analysis

- input_dir: `diagnostics/research_baseline_search_post_settle_count_recenter8_gain3000_damping10_25hz_v1`
- command_source: `trajectory_commands.csv`
- tracking_source: `trajectory_controller_state_samples.csv`
- physical_clearance_m: `0.001000`
- hold_like_command_count: `1`
- centered_hold_like_command_count: `1`
- max_centered_hold_p95_actual_xy_drift_m: `0.002226`
- max_centered_hold_p95_joint_error_rad: `0.007530`

| Cmd | Hold | Centered target | Samples | Target XY m | Feedback mean XY m | Actual XY drift p95 m | Linearized XY drift p95 m | Residual p95 m | JTC p95 rad | Max sens p95 m/rad | Cond p95 | Dominant joint p95 m |
| ---: | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 0 | no | yes | 10035 | 0.000000 | 0.226786 | 0.002704 | 0.002703 | 0.000010 | 0.012767 | 0.847947 | 1.882 | joint_1 0.001767 |
| 1 | no | yes | 2940 | 0.000000 | 0.001285 | 0.002146 | 0.002149 | 0.000007 | 0.007598 | 0.659473 | 1.089 | joint_1 0.001274 |
| 2 | yes | yes | 3020 | 0.000000 | 0.001221 | 0.002226 | 0.002226 | 0.000007 | 0.007530 | 0.658674 | 1.089 | joint_1 0.001417 |
| 3 | no | no | 6268 | 0.409676 | 0.132844 | 0.002697 | 0.002690 | 0.000009 | 0.011746 | 0.845978 | 1.734 | joint_1 0.001839 |

Interpretation: this passive diagnostic uses the current finite-difference peg-tip Jacobian at each controller reference. The linearized XY drift is `J_xy * (feedback - reference)`. Close agreement between actual and linearized drift means controller tracking error is sufficient to explain the Cartesian offset; a large residual points to model, frame, or log consistency issues.
