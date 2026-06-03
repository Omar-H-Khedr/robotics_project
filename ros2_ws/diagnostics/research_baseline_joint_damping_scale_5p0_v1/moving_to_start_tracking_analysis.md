# MOVING_TO_START Tracking Analysis

- input_dir: `diagnostics/research_baseline_joint_damping_scale_5p0_v1`
- result: `OK`
- command_index: `0`
- tracking_source: `trajectory_controller_state_samples.csv`
- command_duration_s: `40.000`
- post_command_hold_before_next_command_s: `1.108`
- samples: `10278`
- p95_max_abs_position_error_rad: `0.013400`
- worst_joint_by_p95_error: `joint_6` `0.013386` rad
- worst_joint_by_final_error: `joint_6` `0.005355` rad
- command_target_xyz_m: `0.520000, -0.200000, 0.885001`
- final_feedback_xyz_m: `0.517753, -0.202272, 0.883571`
- final_cartesian_error_xyz_m: `0.002247, 0.002272, 0.001429`
- final_cartesian_error_norm_m: `0.003501`
- final_xy_error_m: `0.003196`
- xy_error_min_m: `0.000088`
- xy_error_mean_m: `0.221740`
- xy_error_p95_m: `0.406720`
- xy_error_max_m: `0.413772`
- strict_xy_sample_count: `168`
- strict_xy_sample_fraction: `0.016346`
- final_xy_window_s: `1.000`
- final_xy_window_min_m: `0.000088`
- final_xy_window_mean_m: `0.002044`
- final_xy_window_max_m: `0.004176`

## Per Joint Error

| joint | max_abs_rad | p95_abs_rad | mean_abs_rad | final_error_rad | target_minus_feedback_rad | target_rad | final_feedback_rad |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| joint_1 | 0.006680 | 0.004439 | 0.002133 | 0.003551 | 0.003551 | -0.502569 | -0.506120 |
| joint_2 | 0.008115 | 0.004202 | 0.001587 | -0.001891 | -0.001891 | -0.992068 | -0.990177 |
| joint_3 | 0.009406 | 0.004988 | 0.001918 | -0.001360 | -0.001360 | 2.340682 | 2.342041 |
| joint_4 | 0.012366 | 0.007325 | 0.003112 | -0.001544 | -0.001544 | -0.112781 | -0.111237 |
| joint_5 | 0.013887 | 0.008803 | 0.003802 | 0.000712 | 0.000712 | -1.349980 | -1.350692 |
| joint_6 | 0.018547 | 0.013386 | 0.005459 | 0.005355 | 0.005355 | 0.024802 | 0.019448 |

Interpretation: this is an offline diagnostic over passive trajectory observer CSVs. It selects the MOVING_TO_START command by peg-tip target pose, not by command index, and does not alter controller behavior or safety gates.
