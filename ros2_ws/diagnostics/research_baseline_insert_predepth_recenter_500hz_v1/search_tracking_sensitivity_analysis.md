# Search Tracking Sensitivity Analysis

- input_dir: `diagnostics/research_baseline_insert_predepth_recenter_500hz_v1`
- command_source: `trajectory_commands.csv`
- tracking_source: `trajectory_controller_state_samples.csv`
- physical_clearance_m: `0.001000`
- hold_like_command_count: `4`
- centered_hold_like_command_count: `4`
- max_centered_hold_p95_actual_xy_drift_m: `0.000963`
- max_centered_hold_p95_joint_error_rad: `0.003784`

| Cmd | Hold | Centered target | Samples | Target XY m | Feedback mean XY m | Actual XY drift p95 m | Linearized XY drift p95 m | Residual p95 m | JTC p95 rad | Max sens p95 m/rad | Cond p95 | Dominant joint p95 m |
| ---: | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 0 | no | yes | 20009 | 0.000000 | 0.228244 | 0.001178 | 0.001178 | 0.000002 | 0.006323 | 0.848786 | 1.884 | joint_5 0.000834 |
| 1 | no | yes | 5848 | 0.000000 | 0.000487 | 0.000913 | 0.000914 | 0.000001 | 0.003708 | 0.659266 | 1.089 | joint_3 0.000507 |
| 2 | yes | yes | 1221 | 0.000000 | 0.000513 | 0.000963 | 0.000963 | 0.000001 | 0.003692 | 0.660256 | 1.092 | joint_1 0.000618 |
| 3 | yes | yes | 1319 | 0.000000 | 0.000799 | 0.000889 | 0.000889 | 0.000001 | 0.003707 | 0.653229 | 1.078 | joint_3 0.000464 |
| 4 | yes | yes | 1138 | 0.000000 | 0.000545 | 0.000917 | 0.000916 | 0.000001 | 0.003762 | 0.653202 | 1.078 | joint_3 0.000492 |
| 5 | yes | yes | 11508 | 0.000000 | 0.000565 | 0.000935 | 0.000935 | 0.000001 | 0.003784 | 0.656715 | 1.099 | joint_1 0.000530 |
| 6 | no | no | 12551 | 0.409676 | 0.101295 | 0.001057 | 0.001057 | 0.000002 | 0.005332 | 0.844996 | 1.705 | joint_5 0.000673 |

Interpretation: this passive diagnostic uses the current finite-difference peg-tip Jacobian at each controller reference. The linearized XY drift is `J_xy * (feedback - reference)`. Close agreement between actual and linearized drift means controller tracking error is sufficient to explain the Cartesian offset; a large residual points to model, frame, or log consistency issues.
