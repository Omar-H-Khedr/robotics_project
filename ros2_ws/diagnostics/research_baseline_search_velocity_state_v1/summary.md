# research_baseline_search_velocity_state_v1

## Setup
- launch: `ros2 launch thesis_bringup research_baseline.launch.py use_gui:=false inject_velocity_state:=true position_gain:=2000.0 position_derivative_gain:=10.0 joint_damping_scale:=5.0`
- velocity-state interface added to URDF via `inject_velocity_state_urdf.py` (idempotent).
- JTC `state_interfaces: [position, velocity]` from
  `config/research_baseline_velocity_state.yaml`.
- Real joint velocity from `gz_ros2_control/GazeboSimSystem` is now
  available to the JTC's `position_derivative_gain` (D-term uses
  `joint_*/velocity` instead of finite-difference of position).
- Baseline D-term: `10.0` (best from v3/v4 line of work).
- Baseline gain: `2000.0`.
- Damping: `5.0` × original.

## Outcome
- ABORTED SEARCH timeout.
- final XY 0.0034 m (reporter), Max |Fz| 132.5 N, Max |F| 211.1 N.
- SEARCH final XY 0.0018 m, ~within 0.0010 m physical clearance band
  (XY error remains 0.0018 m, still above 0.0010 m).
- Linearization residual p95: 0.000016-0.000020 m (consistent with
  Jacobian estimate).

## Centered-hold tracking (JTC controller state, the actual JTC sees)
| metric                              |     value |
| ----------------------------------- | --------: |
| max_centered_hold_p95_actual_xy_drift_m | 0.004015 |
| max_centered_hold_p95_joint_error_rad   | 0.008408 |
| controller_state_p95_joint_error_rad    | 0.011460 |
| controller_state_mean_rms_error_rad     | 0.003351 |
| hold_like_best_feedback_1mm_ticks       | 2 |

## Comparison vs prior diagnostics
| variant                    | D-term | gain  | centered-hold p95 actual XY drift | 1mm window ticks |
| -------------------------- | -----: | ----: | ---------------------------------: | ---------------: |
| streak_preservation_v1     |  0     |  2000 |                          0.004040 |                3 |
| derivative_gain_v1         |  0.5   |  2000 |                          0.003919 |                3 |
| derivative_gain_v2         |  5.0   |  2000 |                          0.004018 |                3 |
| derivative_gain_v3         | 10.0   |  3000 |                          0.004029 |                3 |
| derivative_gain_v4         | 10.0   |  2000 |                          0.004123 |                3 |
| gain3000_v1                |  0     |  3000 |                          0.004013 |                2 |
| position_controller_v1     |  0     |  2000 |                              n/a |                2 |
| position_controller_v2     |  0     |  2000 |                              n/a |                4 |
| **velocity_state_v1**      | 10.0   |  2000 |                          0.004015 |                2 |

## Conclusion
- Injecting a real `joint_*/velocity` state interface from
  `gz_ros2_control/GazeboSimSystem` and adding `velocity` to the JTC's
  `state_interfaces` did **not** unblock the 1 mm sustained window.
  Centered-hold p95 actual XY drift stayed at ~4.0 mm (≈ position
  controller v1 and gain3000_v1).
- This diagnostic closes out the "use real joint velocity" lever. The
  D-term's input source (finite-difference vs. real velocity) is not
  the binding constraint.
- The 1 mm sustained window remains unblocked across every variant
  tried: position gain scaling (2000-3000), D-term scaling (0-10),
  controller type (JTC vs. position controller), and velocity state
  source (finite-difference vs. real). The binding constraint is
  somewhere outside these four levers.
