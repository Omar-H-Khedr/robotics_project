# Search Tracking Sensitivity Analysis

- input_dir: `diagnostics/research_baseline_insert_handoff_gate_order_v1`
- command_source: `trajectory_commands.csv`
- tracking_source: `trajectory_controller_state_samples.csv`
- physical_clearance_m: `0.001000`
- hold_like_command_count: `5`
- centered_hold_like_command_count: `5`
- max_centered_hold_p95_actual_xy_drift_m: `0.003940`
- max_centered_hold_p95_joint_error_rad: `0.008577`

| Cmd | Hold | Centered target | Samples | Target XY m | Feedback mean XY m | Actual XY drift p95 m | Linearized XY drift p95 m | Residual p95 m | JTC p95 rad | Max sens p95 m/rad | Cond p95 | Dominant joint p95 m |
| ---: | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 0 | no | yes | 10169 | 0.000000 | 0.223979 | 0.004584 | 0.004584 | 0.000021 | 0.013390 | 0.848796 | 1.885 | joint_1 0.003531 |
| 1 | no | yes | 4417 | 0.000000 | 0.002135 | 0.003888 | 0.003888 | 0.000019 | 0.008339 | 0.665211 | 1.105 | joint_1 0.002816 |
| 2 | yes | yes | 1510 | 0.000001 | 0.002093 | 0.003828 | 0.003822 | 0.000018 | 0.008519 | 0.664619 | 1.103 | joint_1 0.002609 |
| 3 | yes | yes | 1509 | 0.000000 | 0.002560 | 0.003940 | 0.003943 | 0.000019 | 0.008440 | 0.655626 | 1.087 | joint_1 0.002744 |
| 4 | yes | yes | 1510 | 0.000000 | 0.002098 | 0.003768 | 0.003767 | 0.000018 | 0.008394 | 0.655015 | 1.085 | joint_1 0.002593 |
| 5 | yes | yes | 1520 | 0.000006 | 0.002278 | 0.003930 | 0.003925 | 0.000020 | 0.008555 | 0.654893 | 1.086 | joint_1 0.002707 |
| 6 | yes | yes | 949 | 0.000007 | 0.002571 | 0.003744 | 0.003746 | 0.000018 | 0.008577 | 0.655285 | 1.087 | joint_1 0.002635 |

Interpretation: this passive diagnostic uses the current finite-difference peg-tip Jacobian at each controller reference. The linearized XY drift is `J_xy * (feedback - reference)`. Close agreement between actual and linearized drift means controller tracking error is sufficient to explain the Cartesian offset; a large residual points to model, frame, or log consistency issues.
