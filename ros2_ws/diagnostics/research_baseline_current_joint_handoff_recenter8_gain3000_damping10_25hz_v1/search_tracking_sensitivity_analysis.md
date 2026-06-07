# Search Tracking Sensitivity Analysis

- input_dir: `diagnostics/research_baseline_current_joint_handoff_recenter8_gain3000_damping10_25hz_v1`
- command_source: `trajectory_commands.csv`
- tracking_source: `trajectory_controller_state_samples.csv`
- physical_clearance_m: `0.001000`
- hold_like_command_count: `4`
- centered_hold_like_command_count: `4`
- max_centered_hold_p95_actual_xy_drift_m: `0.002290`
- max_centered_hold_p95_joint_error_rad: `0.007722`

| Cmd | Hold | Centered target | Samples | Target XY m | Feedback mean XY m | Actual XY drift p95 m | Linearized XY drift p95 m | Residual p95 m | JTC p95 rad | Max sens p95 m/rad | Cond p95 | Dominant joint p95 m |
| ---: | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 0 | no | yes | 10018 | 0.000000 | 0.226732 | 0.002755 | 0.002755 | 0.000010 | 0.013034 | 0.846871 | 1.880 | joint_5 0.001785 |
| 1 | no | yes | 5127 | 0.000000 | 0.001180 | 0.002225 | 0.002224 | 0.000007 | 0.007560 | 0.665197 | 1.105 | joint_1 0.001358 |
| 2 | yes | yes | 2259 | 0.000001 | 0.001352 | 0.002290 | 0.002290 | 0.000007 | 0.007722 | 0.665224 | 1.107 | joint_1 0.001408 |
| 3 | yes | yes | 2280 | 0.000003 | 0.001492 | 0.002186 | 0.002185 | 0.000007 | 0.007521 | 0.654974 | 1.085 | joint_1 0.001418 |
| 4 | yes | yes | 2280 | 0.000002 | 0.001191 | 0.002258 | 0.002260 | 0.000007 | 0.007564 | 0.654203 | 1.082 | joint_1 0.001337 |
| 5 | yes | yes | 2180 | 0.000002 | 0.001472 | 0.002239 | 0.002236 | 0.000007 | 0.007603 | 0.654835 | 1.084 | joint_1 0.001414 |
| 6 | no | no | 1353 | 0.409676 | 0.001872 | 0.002223 | 0.002220 | 0.000007 | 0.007548 | 0.654955 | 1.084 | joint_1 0.001472 |

Interpretation: this passive diagnostic uses the current finite-difference peg-tip Jacobian at each controller reference. The linearized XY drift is `J_xy * (feedback - reference)`. Close agreement between actual and linearized drift means controller tracking error is sufficient to explain the Cartesian offset; a large residual points to model, frame, or log consistency issues.
