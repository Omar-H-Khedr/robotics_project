# Search Tracking Sensitivity Analysis

- input_dir: `diagnostics/research_baseline_search_post_command_stability_gate_v1_repeat2`
- command_source: `trajectory_commands.csv`
- tracking_source: `trajectory_controller_state_samples.csv`
- physical_clearance_m: `0.001000`
- hold_like_command_count: `7`
- centered_hold_like_command_count: `6`
- max_centered_hold_p95_actual_xy_drift_m: `0.003876`
- max_centered_hold_p95_joint_error_rad: `0.008488`

| Cmd | Hold | Centered target | Samples | Target XY m | Feedback mean XY m | Actual XY drift p95 m | Linearized XY drift p95 m | Residual p95 m | JTC p95 rad | Max sens p95 m/rad | Cond p95 | Dominant joint p95 m |
| ---: | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 0 | no | yes | 12673 | 0.000000 | 0.180451 | 0.004463 | 0.004460 | 0.000020 | 0.012759 | 0.848211 | 1.852 | joint_1 0.003430 |
| 1 | no | yes | 4476 | 0.000000 | 0.002335 | 0.003811 | 0.003804 | 0.000018 | 0.008352 | 0.665144 | 1.107 | joint_1 0.002561 |
| 2 | yes | yes | 1524 | 0.000001 | 0.002192 | 0.003557 | 0.003549 | 0.000016 | 0.008377 | 0.664981 | 1.109 | joint_1 0.002420 |
| 3 | yes | no | 1525 | 0.003001 | 0.003303 | 0.003796 | 0.003787 | 0.000019 | 0.008481 | 0.654748 | 1.087 | joint_1 0.002772 |
| 4 | yes | yes | 1525 | 0.000009 | 0.002215 | 0.003723 | 0.003725 | 0.000018 | 0.008442 | 0.654434 | 1.085 | joint_1 0.002657 |
| 5 | yes | yes | 1525 | 0.000006 | 0.002253 | 0.003817 | 0.003805 | 0.000018 | 0.008333 | 0.654855 | 1.088 | joint_1 0.002787 |
| 6 | yes | yes | 1525 | 0.000003 | 0.002289 | 0.003867 | 0.003865 | 0.000019 | 0.008457 | 0.654925 | 1.086 | joint_1 0.002744 |
| 7 | yes | yes | 1525 | 0.000000 | 0.002304 | 0.003750 | 0.003748 | 0.000017 | 0.008262 | 0.655564 | 1.090 | joint_1 0.002686 |
| 8 | yes | yes | 604 | 0.000001 | 0.002466 | 0.003876 | 0.003867 | 0.000017 | 0.008488 | 0.654818 | 1.084 | joint_1 0.002759 |
| 9 | no | no | 27202 | 0.409676 | 0.342059 | 0.004320 | 0.004318 | 0.000018 | 0.011699 | 0.849056 | 1.888 | joint_1 0.003367 |

Interpretation: this passive diagnostic uses the current finite-difference peg-tip Jacobian at each controller reference. The linearized XY drift is `J_xy * (feedback - reference)`. Close agreement between actual and linearized drift means controller tracking error is sufficient to explain the Cartesian offset; a large residual points to model, frame, or log consistency issues.
