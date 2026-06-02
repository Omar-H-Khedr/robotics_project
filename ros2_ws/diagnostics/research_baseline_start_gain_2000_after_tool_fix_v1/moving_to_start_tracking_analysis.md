# MOVING_TO_START Tracking Analysis

- input_dir: `diagnostics/research_baseline_start_gain_2000_after_tool_fix_v1`
- result: `OK`
- command_index: `0`
- command_duration_s: `40.000`
- post_command_hold_before_next_command_s: `22.000`
- samples: `15500`
- p95_max_abs_position_error_rad: `0.024488`
- worst_joint_by_p95_error: `joint_3` `0.022163` rad
- worst_joint_by_final_error: `joint_4` `0.013703` rad
- command_target_xyz_m: `0.520000, -0.200000, 0.885001`
- final_feedback_xyz_m: `0.516039, -0.200495, 0.889141`
- final_cartesian_error_xyz_m: `0.003962, 0.000494, -0.004140`
- final_cartesian_error_norm_m: `0.005751`
- final_xy_error_m: `0.003992`

## Per Joint Error

| joint | max_abs_rad | p95_abs_rad | mean_abs_rad | final_error_rad | target_minus_feedback_rad | target_rad | final_feedback_rad |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| joint_1 | 0.028686 | 0.018779 | 0.009981 | 0.003756 | 0.003756 | -0.502569 | -0.506325 |
| joint_2 | 0.031318 | 0.017618 | 0.007164 | 0.005459 | 0.005459 | -0.992068 | -0.997527 |
| joint_3 | 0.047487 | 0.022163 | 0.008774 | -0.000186 | -0.000186 | 2.340682 | 2.340867 |
| joint_4 | 0.027937 | 0.017496 | 0.006984 | -0.013703 | -0.013703 | -0.112781 | -0.099078 |
| joint_5 | 0.035367 | 0.016093 | 0.006984 | 0.004278 | 0.004278 | -1.349980 | -1.354259 |
| joint_6 | 0.027976 | 0.017067 | 0.007182 | -0.004288 | -0.004288 | 0.024802 | 0.029090 |

Interpretation: this is an offline diagnostic over passive trajectory observer CSVs. It selects the MOVING_TO_START command by peg-tip target pose, not by command index, and does not alter controller behavior or safety gates.
