# MOVING_TO_START Tracking Analysis

- input_dir: `diagnostics/research_baseline_insert_cartesian_descent_v1`
- result: `OK`
- command_index: `0`
- tracking_source: `trajectory_controller_state_samples.csv`
- command_duration_s: `40.000`
- post_command_hold_before_next_command_s: `3.265`
- samples: `10817`
- p95_max_abs_position_error_rad: `0.013301`
- worst_joint_by_p95_error: `joint_6` `0.013298` rad
- worst_joint_by_final_error: `joint_1` `0.002526` rad
- command_target_xyz_m: `0.520000, -0.200000, 0.885001`
- final_feedback_xyz_m: `0.517790, -0.199582, 0.886340`
- final_cartesian_error_xyz_m: `0.002211, -0.000418, -0.001340`
- final_cartesian_error_norm_m: `0.002618`
- final_xy_error_m: `0.002249`
- xy_error_min_m: `0.000048`
- xy_error_mean_m: `0.211659`
- xy_error_p95_m: `0.406992`
- xy_error_max_m: `0.414466`
- strict_xy_sample_count: `456`
- strict_xy_sample_fraction: `0.042156`
- final_xy_window_s: `1.000`
- final_xy_window_min_m: `0.000121`
- final_xy_window_mean_m: `0.001776`
- final_xy_window_max_m: `0.005072`

## Per Joint Error

| joint | max_abs_rad | p95_abs_rad | mean_abs_rad | final_error_rad | target_minus_feedback_rad | target_rad | final_feedback_rad |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| joint_1 | 0.006413 | 0.004452 | 0.002132 | 0.002526 | 0.002526 | -0.502569 | -0.505095 |
| joint_2 | 0.006839 | 0.003989 | 0.001502 | 0.001163 | 0.001163 | -0.992068 | -0.993231 |
| joint_3 | 0.012003 | 0.005043 | 0.001968 | 0.001731 | 0.001731 | 2.340682 | 2.338951 |
| joint_4 | 0.012064 | 0.007196 | 0.003060 | -0.000760 | -0.000760 | -0.112781 | -0.112021 |
| joint_5 | 0.014723 | 0.008841 | 0.003763 | 0.000602 | 0.000602 | -1.349980 | -1.350582 |
| joint_6 | 0.018639 | 0.013298 | 0.005369 | 0.000605 | 0.000605 | 0.024802 | 0.024197 |

Interpretation: this is an offline diagnostic over passive trajectory observer CSVs. It selects the MOVING_TO_START command by peg-tip target pose, not by command index, and does not alter controller behavior or safety gates.
