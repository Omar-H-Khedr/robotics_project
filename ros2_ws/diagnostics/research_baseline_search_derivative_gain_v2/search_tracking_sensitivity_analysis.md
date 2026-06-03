# Search Tracking Sensitivity Analysis

- input_dir: `diagnostics/research_baseline_search_derivative_gain_v2`
- command_source: `trajectory_commands.csv`
- tracking_source: `trajectory_controller_state_samples.csv`
- physical_clearance_m: `0.001000`
- hold_like_command_count: `7`
- centered_hold_like_command_count: `7`
- max_centered_hold_p95_actual_xy_drift_m: `0.004018`
- max_centered_hold_p95_joint_error_rad: `0.008434`

| Cmd | Hold | Centered target | Samples | Target XY m | Feedback mean XY m | Actual XY drift p95 m | Linearized XY drift p95 m | Residual p95 m | JTC p95 rad | Max sens p95 m/rad | Cond p95 | Dominant joint p95 m |
| ---: | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 0 | no | yes | 11937 | 0.000000 | 0.190879 | 0.004508 | 0.004508 | 0.000020 | 0.013025 | 0.848312 | 1.866 | joint_1 0.003427 |
| 1 | no | yes | 4445 | 0.000000 | 0.002147 | 0.003796 | 0.003793 | 0.000018 | 0.008421 | 0.665353 | 1.105 | joint_1 0.002612 |
| 2 | yes | yes | 1525 | 0.000001 | 0.002742 | 0.004011 | 0.004001 | 0.000020 | 0.008430 | 0.665548 | 1.109 | joint_1 0.002814 |
| 3 | yes | yes | 1525 | 0.000003 | 0.002077 | 0.003483 | 0.003481 | 0.000017 | 0.008311 | 0.654587 | 1.082 | joint_1 0.002511 |
| 4 | yes | yes | 1526 | 0.000002 | 0.002407 | 0.003880 | 0.003881 | 0.000019 | 0.008393 | 0.654825 | 1.084 | joint_1 0.002859 |
| 5 | yes | yes | 1524 | 0.000000 | 0.002432 | 0.003791 | 0.003791 | 0.000017 | 0.008376 | 0.655496 | 1.087 | joint_1 0.002589 |
| 6 | yes | yes | 1525 | 0.000009 | 0.002110 | 0.003717 | 0.003717 | 0.000018 | 0.008434 | 0.654764 | 1.085 | joint_1 0.002777 |
| 7 | yes | yes | 1525 | 0.000009 | 0.002294 | 0.004018 | 0.004017 | 0.000020 | 0.008345 | 0.655028 | 1.086 | joint_1 0.002783 |
| 8 | yes | yes | 603 | 0.000002 | 0.002415 | 0.003911 | 0.003908 | 0.000020 | 0.008206 | 0.655360 | 1.086 | joint_1 0.002756 |
| 9 | no | no | 23752 | 0.409676 | 0.332560 | 0.004335 | 0.004333 | 0.000018 | 0.011678 | 0.849056 | 1.888 | joint_1 0.003361 |

Interpretation: this passive diagnostic uses the current finite-difference peg-tip Jacobian at each controller reference. The linearized XY drift is `J_xy * (feedback - reference)`. Close agreement between actual and linearized drift means controller tracking error is sufficient to explain the Cartesian offset; a large residual points to model, frame, or log consistency issues.
