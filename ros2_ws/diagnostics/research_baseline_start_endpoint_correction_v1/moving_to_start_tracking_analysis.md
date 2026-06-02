# MOVING_TO_START Tracking Analysis

- input_dir: `diagnostics/research_baseline_start_endpoint_correction_v1`
- result: `OK`
- command_index: `3`
- command_duration_s: `5.000`
- post_command_hold_before_next_command_s: `9.552`
- samples: `3638`
- p95_max_abs_position_error_rad: `0.023946`
- worst_joint_by_p95_error: `joint_4` `0.020776` rad
- worst_joint_by_final_error: `joint_5` `0.014251` rad
- command_target_xyz_m: `0.520000, -0.200000, 0.885000`
- final_feedback_xyz_m: `0.512317, -0.201028, 0.892162`
- final_cartesian_error_xyz_m: `0.007683, 0.001028, -0.007162`
- final_cartesian_error_norm_m: `0.010554`
- final_xy_error_m: `0.007752`

## Per Joint Error

| joint | max_abs_rad | p95_abs_rad | mean_abs_rad | final_error_rad | target_minus_feedback_rad | target_rad | final_feedback_rad |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| joint_1 | 0.033050 | 0.020686 | 0.010270 | 0.011167 | 0.011167 | -0.499864 | -0.511031 |
| joint_2 | 0.030452 | 0.018632 | 0.007897 | 0.007684 | 0.007684 | -0.992131 | -0.999815 |
| joint_3 | 0.025539 | 0.016361 | 0.007206 | -0.000450 | -0.000450 | 2.340926 | 2.341376 |
| joint_4 | 0.034442 | 0.020776 | 0.008241 | 0.003989 | 0.003989 | -0.103117 | -0.107106 |
| joint_5 | 0.027884 | 0.016174 | 0.007068 | 0.014251 | 0.014251 | -1.349936 | -1.364187 |
| joint_6 | 0.021640 | 0.014210 | 0.006199 | 0.009166 | 0.009166 | 0.022666 | 0.013501 |

Interpretation: this is an offline diagnostic over passive trajectory observer CSVs. It selects the MOVING_TO_START command by peg-tip target pose, not by command index, and does not alter controller behavior or safety gates.
