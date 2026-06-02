# MOVING_TO_START Tracking Analysis

- input_dir: `diagnostics/research_baseline_zero_derivative_trajectory_hold_v1`
- result: `OK`
- command_index: `0`
- command_duration_s: `40.000`
- post_command_hold_before_next_command_s: `22.038`
- samples: `15510`
- p95_max_abs_position_error_rad: `0.028158`
- worst_joint_by_p95_error: `joint_5` `0.024759` rad
- worst_joint_by_final_error: `joint_1` `0.017967` rad
- command_target_xyz_m: `0.520000, -0.200000, 0.885001`
- final_feedback_xyz_m: `0.530295, -0.196056, 0.878804`
- final_cartesian_error_xyz_m: `-0.010295, -0.003944, 0.006197`
- final_cartesian_error_norm_m: `0.012647`
- final_xy_error_m: `0.011025`

## Per Joint Error

| joint | max_abs_rad | p95_abs_rad | mean_abs_rad | final_error_rad | target_minus_feedback_rad | target_rad | final_feedback_rad |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| joint_1 | 0.030431 | 0.019930 | 0.010228 | -0.017967 | -0.017967 | -0.502569 | -0.484603 |
| joint_2 | 0.034970 | 0.017053 | 0.007020 | -0.010786 | -0.010786 | -0.992068 | -0.981281 |
| joint_3 | 0.049119 | 0.024069 | 0.009593 | 0.000595 | 0.000595 | 2.340682 | 2.340087 |
| joint_4 | 0.027969 | 0.017962 | 0.007152 | 0.001717 | 0.001717 | -0.112781 | -0.114498 |
| joint_5 | 0.037822 | 0.024759 | 0.010193 | 0.001604 | 0.001604 | -1.349980 | -1.351584 |
| joint_6 | 0.028412 | 0.017021 | 0.007234 | 0.003356 | 0.003356 | 0.024802 | 0.021446 |

Interpretation: this is an offline diagnostic over passive trajectory observer CSVs. It selects the MOVING_TO_START command by peg-tip target pose, not by command index, and does not alter controller behavior or safety gates.
