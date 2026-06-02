# MOVING_TO_START Tracking Analysis

- input_dir: `diagnostics/research_baseline_canonical_after_command_capture_v1`
- result: `OK`
- command_index: `0`
- command_duration_s: `40.000`
- post_command_hold_before_next_command_s: `22.093`
- samples: `15524`
- p95_max_abs_position_error_rad: `0.024627`
- worst_joint_by_p95_error: `joint_3` `0.021407` rad
- worst_joint_by_final_error: `joint_3` `0.005190` rad
- command_target_xyz_m: `0.520000, -0.200000, 0.885001`
- final_feedback_xyz_m: `0.517003, -0.198046, 0.884357`
- final_cartesian_error_xyz_m: `0.002997, -0.001955, 0.000644`
- final_cartesian_error_norm_m: `0.003636`
- final_xy_error_m: `0.003578`

## Per Joint Error

| joint | max_abs_rad | p95_abs_rad | mean_abs_rad | final_error_rad | target_minus_feedback_rad | target_rad | final_feedback_rad |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| joint_1 | 0.028257 | 0.018732 | 0.009927 | 0.003697 | 0.003697 | -0.502569 | -0.506266 |
| joint_2 | 0.033519 | 0.017555 | 0.007049 | -0.004719 | -0.004719 | -0.992068 | -0.987349 |
| joint_3 | 0.045188 | 0.021407 | 0.008807 | 0.005190 | 0.005190 | 2.340682 | 2.335492 |
| joint_4 | 0.028104 | 0.017730 | 0.007062 | 0.001453 | 0.001453 | -0.112781 | -0.114234 |
| joint_5 | 0.033053 | 0.015790 | 0.006906 | 0.004217 | 0.004217 | -1.349980 | -1.354197 |
| joint_6 | 0.046586 | 0.017708 | 0.007407 | -0.002567 | -0.002567 | 0.024802 | 0.027369 |

Interpretation: this is an offline diagnostic over passive trajectory observer CSVs. It selects the MOVING_TO_START command by peg-tip target pose, not by command index, and does not alter controller behavior or safety gates.
