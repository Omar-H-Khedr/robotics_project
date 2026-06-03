# MOVING_TO_START Tracking Analysis

- input_dir: `diagnostics/research_baseline_insert_sim_time_completion_v4`
- result: `OK`
- command_index: `0`
- tracking_source: `trajectory_controller_state_samples.csv`
- command_duration_s: `40.000`
- post_command_hold_before_next_command_s: `3.061`
- samples: `10766`
- p95_max_abs_position_error_rad: `0.013376`
- worst_joint_by_p95_error: `joint_6` `0.013364` rad
- worst_joint_by_final_error: `joint_6` `0.004381` rad
- command_target_xyz_m: `0.520000, -0.200000, 0.885001`
- final_feedback_xyz_m: `0.521889, -0.199905, 0.883203`
- final_cartesian_error_xyz_m: `-0.001889, -0.000095, 0.001798`
- final_cartesian_error_norm_m: `0.002610`
- final_xy_error_m: `0.001892`
- xy_error_min_m: `0.000083`
- xy_error_mean_m: `0.212480`
- xy_error_p95_m: `0.406839`
- xy_error_max_m: `0.413906`
- strict_xy_sample_count: `400`
- strict_xy_sample_fraction: `0.037154`
- final_xy_window_s: `1.000`
- final_xy_window_min_m: `0.000083`
- final_xy_window_mean_m: `0.002027`
- final_xy_window_max_m: `0.004905`

## Per Joint Error

| joint | max_abs_rad | p95_abs_rad | mean_abs_rad | final_error_rad | target_minus_feedback_rad | target_rad | final_feedback_rad |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| joint_1 | 0.006592 | 0.004343 | 0.002120 | -0.003145 | -0.003145 | -0.502569 | -0.499424 |
| joint_2 | 0.007364 | 0.003858 | 0.001485 | -0.002426 | -0.002426 | -0.992068 | -0.989642 |
| joint_3 | 0.011288 | 0.004846 | 0.001916 | -0.000477 | -0.000477 | 2.340682 | 2.341158 |
| joint_4 | 0.012448 | 0.007323 | 0.003110 | 0.002234 | 0.002234 | -0.112781 | -0.115014 |
| joint_5 | 0.014689 | 0.008689 | 0.003782 | -0.000871 | -0.000871 | -1.349980 | -1.349109 |
| joint_6 | 0.018882 | 0.013364 | 0.005398 | 0.004381 | 0.004381 | 0.024802 | 0.020422 |

Interpretation: this is an offline diagnostic over passive trajectory observer CSVs. It selects the MOVING_TO_START command by peg-tip target pose, not by command index, and does not alter controller behavior or safety gates.
