# MOVING_TO_START Tracking Analysis

- input_dir: `diagnostics/research_baseline_retreat_clearance_lift_v1`
- result: `OK`
- command_index: `0`
- tracking_source: `trajectory_controller_state_samples.csv`
- command_duration_s: `40.000`
- post_command_hold_before_next_command_s: `5.372`
- samples: `11340`
- p95_max_abs_position_error_rad: `0.013223`
- worst_joint_by_p95_error: `joint_6` `0.013213` rad
- worst_joint_by_final_error: `joint_4` `0.002462` rad
- command_target_xyz_m: `0.520000, -0.200000, 0.885001`
- final_feedback_xyz_m: `0.521842, -0.198994, 0.886226`
- final_cartesian_error_xyz_m: `-0.001842, -0.001006, -0.001225`
- final_cartesian_error_norm_m: `0.002431`
- final_xy_error_m: `0.002099`
- xy_error_min_m: `0.000046`
- xy_error_mean_m: `0.199501`
- xy_error_p95_m: `0.402041`
- xy_error_max_m: `0.413214`
- strict_xy_sample_count: `711`
- strict_xy_sample_fraction: `0.062698`
- final_xy_window_s: `1.000`
- final_xy_window_min_m: `0.000386`
- final_xy_window_mean_m: `0.002233`
- final_xy_window_max_m: `0.004957`

## Per Joint Error

| joint | max_abs_rad | p95_abs_rad | mean_abs_rad | final_error_rad | target_minus_feedback_rad | target_rad | final_feedback_rad |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| joint_1 | 0.006705 | 0.004524 | 0.002133 | -0.002452 | -0.002452 | -0.502569 | -0.500117 |
| joint_2 | 0.007139 | 0.004025 | 0.001542 | 0.001909 | 0.001909 | -0.992068 | -0.993977 |
| joint_3 | 0.011262 | 0.004779 | 0.001872 | -0.000475 | -0.000475 | 2.340682 | 2.341157 |
| joint_4 | 0.012541 | 0.007145 | 0.003060 | 0.002462 | 0.002462 | -0.112781 | -0.115243 |
| joint_5 | 0.014259 | 0.008730 | 0.003714 | 0.001115 | 0.001115 | -1.349980 | -1.351095 |
| joint_6 | 0.018933 | 0.013213 | 0.005327 | -0.001680 | -0.001680 | 0.024802 | 0.026482 |

Interpretation: this is an offline diagnostic over passive trajectory observer CSVs. It selects the MOVING_TO_START command by peg-tip target pose, not by command index, and does not alter controller behavior or safety gates.
