# MOVING_TO_START Tracking Analysis

- input_dir: `diagnostics/research_baseline_controller_state_tracking_v2`
- result: `OK`
- command_index: `0`
- tracking_source: `trajectory_controller_state_samples.csv`
- command_duration_s: `40.000`
- post_command_hold_before_next_command_s: `23.719`
- samples: `15930`
- p95_max_abs_position_error_rad: `0.023935`
- worst_joint_by_p95_error: `joint_3` `0.021702` rad
- worst_joint_by_final_error: `joint_5` `0.012237` rad
- command_target_xyz_m: `0.520000, -0.200000, 0.885001`
- final_feedback_xyz_m: `0.510333, -0.199096, 0.891092`
- final_cartesian_error_xyz_m: `0.009667, -0.000905, -0.006091`
- final_cartesian_error_norm_m: `0.011462`
- final_xy_error_m: `0.009709`
- xy_error_min_m: `0.000218`
- xy_error_mean_m: `0.144731`
- xy_error_p95_m: `0.394336`
- xy_error_max_m: `0.423051`
- strict_xy_sample_count: `143`
- strict_xy_sample_fraction: `0.008977`
- final_xy_window_s: `1.000`
- final_xy_window_min_m: `0.000589`
- final_xy_window_mean_m: `0.010512`
- final_xy_window_max_m: `0.023077`

## Per Joint Error

| joint | max_abs_rad | p95_abs_rad | mean_abs_rad | final_error_rad | target_minus_feedback_rad | target_rad | final_feedback_rad |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| joint_1 | 0.029587 | 0.019821 | 0.010229 | 0.010885 | 0.010885 | -0.502569 | -0.513454 |
| joint_2 | 0.032073 | 0.017263 | 0.007083 | 0.003953 | 0.003953 | -0.992068 | -0.996021 |
| joint_3 | 0.047585 | 0.021702 | 0.008510 | 0.004720 | 0.004720 | 2.340682 | 2.335962 |
| joint_4 | 0.029008 | 0.017218 | 0.006911 | -0.010125 | -0.010125 | -0.112781 | -0.102656 |
| joint_5 | 0.023795 | 0.015605 | 0.006815 | 0.012237 | 0.012237 | -1.349980 | -1.362218 |
| joint_6 | 0.029216 | 0.016918 | 0.007153 | -0.000046 | -0.000046 | 0.024802 | 0.024848 |

Interpretation: this is an offline diagnostic over passive trajectory observer CSVs. It selects the MOVING_TO_START command by peg-tip target pose, not by command index, and does not alter controller behavior or safety gates.
