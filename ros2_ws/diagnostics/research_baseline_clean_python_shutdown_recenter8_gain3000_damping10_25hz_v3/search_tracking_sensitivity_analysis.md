# Search Tracking Sensitivity Analysis

- input_dir: `diagnostics/research_baseline_clean_python_shutdown_recenter8_gain3000_damping10_25hz_v3`
- command_source: `trajectory_commands.csv`
- tracking_source: `trajectory_controller_state_samples.csv`
- physical_clearance_m: `0.001000`
- hold_like_command_count: `4`
- centered_hold_like_command_count: `4`
- max_centered_hold_p95_actual_xy_drift_m: `0.002290`
- max_centered_hold_p95_joint_error_rad: `0.007629`

| Cmd | Hold | Centered target | Samples | Target XY m | Feedback mean XY m | Actual XY drift p95 m | Linearized XY drift p95 m | Residual p95 m | JTC p95 rad | Max sens p95 m/rad | Cond p95 | Dominant joint p95 m |
| ---: | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 0 | no | yes | 10015 | 0.000000 | 0.227683 | 0.002774 | 0.002773 | 0.000010 | 0.013156 | 0.848789 | 1.887 | joint_5 0.001807 |
| 1 | no | yes | 5118 | 0.000000 | 0.001195 | 0.002219 | 0.002217 | 0.000008 | 0.007561 | 0.665366 | 1.105 | joint_1 0.001343 |
| 2 | yes | yes | 2299 | 0.000001 | 0.001261 | 0.002231 | 0.002230 | 0.000007 | 0.007565 | 0.665048 | 1.106 | joint_1 0.001467 |
| 3 | yes | yes | 2260 | 0.000002 | 0.001248 | 0.002238 | 0.002238 | 0.000007 | 0.007629 | 0.653990 | 1.080 | joint_1 0.001419 |
| 4 | yes | yes | 2269 | 0.000001 | 0.001380 | 0.002274 | 0.002274 | 0.000007 | 0.007589 | 0.653968 | 1.080 | joint_1 0.001310 |
| 5 | yes | yes | 2154 | 0.000001 | 0.001366 | 0.002290 | 0.002288 | 0.000007 | 0.007620 | 0.654191 | 1.081 | joint_1 0.001366 |
| 6 | no | no | 6286 | 0.409676 | 0.125675 | 0.002573 | 0.002572 | 0.000009 | 0.011190 | 0.845956 | 1.734 | joint_1 0.001721 |

Interpretation: this passive diagnostic uses the current finite-difference peg-tip Jacobian at each controller reference. The linearized XY drift is `J_xy * (feedback - reference)`. Close agreement between actual and linearized drift means controller tracking error is sufficient to explain the Cartesian offset; a large residual points to model, frame, or log consistency issues.
