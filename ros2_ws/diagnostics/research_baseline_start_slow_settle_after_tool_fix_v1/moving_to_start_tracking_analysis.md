# MOVING_TO_START Tracking Analysis

- input_dir: `diagnostics/research_baseline_start_slow_settle_after_tool_fix_v1`
- result: `OK`
- command_index: `0`
- command_duration_s: `20.000`
- post_command_hold_before_next_command_s: `2.274`
- samples: `5569`
- p95_max_abs_position_error_rad: `0.023095`
- worst_joint_by_p95_error: `joint_4` `0.020421` rad
- worst_joint_by_final_error: `joint_1` `0.020130` rad
- command_target_xyz_m: `0.520000, -0.200000, 0.885001`
- final_feedback_xyz_m: `0.508012, -0.204392, 0.888316`
- final_cartesian_error_xyz_m: `0.011988, 0.004392, -0.003315`
- final_cartesian_error_norm_m: `0.013191`
- final_xy_error_m: `0.012767`

## Per Joint Error

| joint | max_abs_rad | p95_abs_rad | mean_abs_rad | final_error_rad | target_minus_feedback_rad | target_rad | final_feedback_rad |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| joint_1 | 0.034553 | 0.018220 | 0.009345 | 0.020130 | 0.020130 | -0.502569 | -0.522699 |
| joint_2 | 0.033123 | 0.017329 | 0.007628 | 0.002439 | 0.002439 | -0.992068 | -0.994506 |
| joint_3 | 0.026502 | 0.017723 | 0.007884 | 0.000046 | 0.000046 | 2.340682 | 2.340636 |
| joint_4 | 0.028694 | 0.020421 | 0.008198 | 0.000190 | 0.000190 | -0.112781 | -0.112970 |
| joint_5 | 0.028450 | 0.016127 | 0.006908 | 0.010044 | 0.010044 | -1.349980 | -1.360025 |
| joint_6 | 0.021638 | 0.014366 | 0.006203 | 0.003031 | 0.003031 | 0.024802 | 0.021771 |

Interpretation: this is an offline diagnostic over passive trajectory observer CSVs. It selects the MOVING_TO_START command by peg-tip target pose, not by command index, and does not alter controller behavior or safety gates.
