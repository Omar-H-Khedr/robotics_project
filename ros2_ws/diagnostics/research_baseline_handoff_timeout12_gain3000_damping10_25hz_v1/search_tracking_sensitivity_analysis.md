# Search Tracking Sensitivity Analysis

- input_dir: `diagnostics/research_baseline_handoff_timeout12_gain3000_damping10_25hz_v1`
- command_source: `trajectory_commands.csv`
- tracking_source: `trajectory_controller_state_samples.csv`
- physical_clearance_m: `0.001000`
- hold_like_command_count: `7`
- centered_hold_like_command_count: `7`
- max_centered_hold_p95_actual_xy_drift_m: `0.002304`
- max_centered_hold_p95_joint_error_rad: `0.007709`

| Cmd | Hold | Centered target | Samples | Target XY m | Feedback mean XY m | Actual XY drift p95 m | Linearized XY drift p95 m | Residual p95 m | JTC p95 rad | Max sens p95 m/rad | Cond p95 | Dominant joint p95 m |
| ---: | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 0 | no | yes | 10049 | 0.000000 | 0.226570 | 0.002754 | 0.002752 | 0.000010 | 0.012879 | 0.848724 | 1.881 | joint_5 0.001793 |
| 1 | no | yes | 4428 | 0.000000 | 0.001172 | 0.002183 | 0.002181 | 0.000007 | 0.007532 | 0.665240 | 1.105 | joint_1 0.001324 |
| 2 | yes | yes | 1519 | 0.000001 | 0.001673 | 0.002286 | 0.002286 | 0.000008 | 0.007709 | 0.665203 | 1.108 | joint_1 0.001250 |
| 3 | yes | yes | 1510 | 0.000006 | 0.001519 | 0.002304 | 0.002304 | 0.000007 | 0.007493 | 0.654694 | 1.086 | joint_1 0.001526 |
| 4 | yes | yes | 1509 | 0.000003 | 0.001378 | 0.002191 | 0.002190 | 0.000007 | 0.007580 | 0.654871 | 1.086 | joint_1 0.001361 |
| 5 | yes | yes | 1521 | 0.000005 | 0.001203 | 0.002238 | 0.002238 | 0.000007 | 0.007664 | 0.654120 | 1.083 | joint_1 0.001321 |
| 6 | yes | yes | 1509 | 0.000002 | 0.001597 | 0.002257 | 0.002253 | 0.000007 | 0.007614 | 0.655276 | 1.089 | joint_1 0.001421 |
| 7 | yes | yes | 1530 | 0.000001 | 0.001482 | 0.002223 | 0.002221 | 0.000007 | 0.007564 | 0.654283 | 1.084 | joint_1 0.001367 |
| 8 | yes | yes | 653 | 0.000002 | 0.001855 | 0.002157 | 0.002156 | 0.000007 | 0.007550 | 0.655427 | 1.089 | joint_1 0.001252 |
| 9 | no | no | 1354 | 0.409676 | 0.001910 | 0.002208 | 0.002206 | 0.000007 | 0.007619 | 0.655506 | 1.088 | joint_1 0.001303 |

Interpretation: this passive diagnostic uses the current finite-difference peg-tip Jacobian at each controller reference. The linearized XY drift is `J_xy * (feedback - reference)`. Close agreement between actual and linearized drift means controller tracking error is sufficient to explain the Cartesian offset; a large residual points to model, frame, or log consistency issues.
