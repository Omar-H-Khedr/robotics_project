# MOVING_TO_START Tracking Analysis

- input_dir: `diagnostics/research_baseline_trajectory_command_capture_v1`
- result: `OK`
- command_index: `0`
- command_duration_s: `40.000`
- post_command_hold_before_next_command_s: `0.000`
- samples: `5602`
- p95_max_abs_position_error_rad: `0.027239`
- worst_joint_by_p95_error: `joint_3` `0.026942` rad
- worst_joint_by_final_error: `joint_3` `0.023885` rad
- command_target_xyz_m: `0.520000, -0.200000, 0.885001`
- final_feedback_xyz_m: `0.601778, -0.013772, 0.867164`
- final_cartesian_error_xyz_m: `-0.081778, -0.186229, 0.017837`
- final_cartesian_error_norm_m: `0.204174`
- final_xy_error_m: `0.203393`

## Per Joint Error

| joint | max_abs_rad | p95_abs_rad | mean_abs_rad | final_error_rad | target_minus_feedback_rad | target_rad | final_feedback_rad |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| joint_1 | 0.026845 | 0.019393 | 0.010307 | -0.011408 | -0.242983 | -0.502569 | -0.259587 |
| joint_2 | 0.027429 | 0.015587 | 0.005937 | 0.000947 | -0.091584 | -0.992068 | -0.900483 |
| joint_3 | 0.052973 | 0.026942 | 0.009784 | -0.023885 | 0.505962 | 2.340682 | 1.834719 |
| joint_4 | 0.024905 | 0.013932 | 0.005941 | -0.000588 | -0.060952 | -0.112781 | -0.051829 |
| joint_5 | 0.028567 | 0.015412 | 0.006889 | -0.002946 | -0.999494 | -1.349980 | -0.350486 |
| joint_6 | 0.030328 | 0.020394 | 0.008672 | 0.010222 | 0.022309 | 0.024802 | 0.002493 |

Interpretation: this is an offline diagnostic over passive trajectory observer CSVs. It selects the MOVING_TO_START command by peg-tip target pose, not by command index, and does not alter controller behavior or safety gates.
