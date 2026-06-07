# Search Tracking Sensitivity Analysis

- input_dir: `diagnostics/research_baseline_search_gain3000_settle_seconds_25hz_v1`
- command_source: `trajectory_commands.csv`
- tracking_source: `trajectory_controller_state_samples.csv`
- physical_clearance_m: `0.001000`
- hold_like_command_count: `1`
- centered_hold_like_command_count: `1`
- max_centered_hold_p95_actual_xy_drift_m: `0.003254`
- max_centered_hold_p95_joint_error_rad: `0.007132`

| Cmd | Hold | Centered target | Samples | Target XY m | Feedback mean XY m | Actual XY drift p95 m | Linearized XY drift p95 m | Residual p95 m | JTC p95 rad | Max sens p95 m/rad | Cond p95 | Dominant joint p95 m |
| ---: | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 0 | no | yes | 10012 | 0.000000 | 0.228441 | 0.004615 | 0.004615 | 0.000020 | 0.013408 | 0.848004 | 1.876 | joint_1 0.003496 |
| 1 | no | yes | 2858 | 0.000000 | 0.002212 | 0.003750 | 0.003751 | 0.000018 | 0.008381 | 0.658724 | 1.088 | joint_1 0.002678 |
| 2 | yes | yes | 30 | 0.000000 | 0.001814 | 0.003254 | 0.003257 | 0.000016 | 0.007132 | 0.660793 | 1.094 | joint_1 0.001614 |
| 3 | no | no | 7831 | 0.409676 | 0.188055 | 0.004319 | 0.004315 | 0.000018 | 0.011889 | 0.849056 | 1.888 | joint_1 0.003211 |

Interpretation: this passive diagnostic uses the current finite-difference peg-tip Jacobian at each controller reference. The linearized XY drift is `J_xy * (feedback - reference)`. Close agreement between actual and linearized drift means controller tracking error is sufficient to explain the Cartesian offset; a large residual points to model, frame, or log consistency issues.
