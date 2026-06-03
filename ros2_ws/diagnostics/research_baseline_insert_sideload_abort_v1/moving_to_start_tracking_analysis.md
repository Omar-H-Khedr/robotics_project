# MOVING_TO_START Tracking Analysis

- input_dir: `diagnostics/research_baseline_insert_sideload_abort_v1`
- result: `OK`
- command_index: `0`
- tracking_source: `trajectory_controller_state_samples.csv`
- command_duration_s: `40.000`
- post_command_hold_before_next_command_s: `0.292`
- samples: `10074`
- p95_max_abs_position_error_rad: `0.013558`
- worst_joint_by_p95_error: `joint_6` `0.013542` rad
- worst_joint_by_final_error: `joint_1` `0.005347` rad
- command_target_xyz_m: `0.520000, -0.200000, 0.885001`
- final_feedback_xyz_m: `0.521976, -0.197660, 0.886018`
- final_cartesian_error_xyz_m: `-0.001975, -0.002340, -0.001017`
- final_cartesian_error_norm_m: `0.003227`
- final_xy_error_m: `0.003062`
- xy_error_min_m: `0.000248`
- xy_error_mean_m: `0.227091`
- xy_error_p95_m: `0.406794`
- xy_error_max_m: `0.413830`
- strict_xy_sample_count: `72`
- strict_xy_sample_fraction: `0.007147`
- final_xy_window_s: `1.000`
- final_xy_window_min_m: `0.000248`
- final_xy_window_mean_m: `0.003956`
- final_xy_window_max_m: `0.010988`

## Per Joint Error

| joint | max_abs_rad | p95_abs_rad | mean_abs_rad | final_error_rad | target_minus_feedback_rad | target_rad | final_feedback_rad |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| joint_1 | 0.006578 | 0.004522 | 0.002110 | -0.005347 | -0.005347 | -0.502569 | -0.497222 |
| joint_2 | 0.006411 | 0.003713 | 0.001433 | 0.000192 | 0.000192 | -0.992068 | -0.992260 |
| joint_3 | 0.011599 | 0.004905 | 0.001923 | 0.001150 | 0.001150 | 2.340682 | 2.339532 |
| joint_4 | 0.012608 | 0.007480 | 0.003154 | 0.000449 | 0.000449 | -0.112781 | -0.113230 |
| joint_5 | 0.014772 | 0.008973 | 0.003854 | 0.003141 | 0.003141 | -1.349980 | -1.353121 |
| joint_6 | 0.018824 | 0.013542 | 0.005504 | 0.004577 | 0.004577 | 0.024802 | 0.020225 |

Interpretation: this is an offline diagnostic over passive trajectory observer CSVs. It selects the MOVING_TO_START command by peg-tip target pose, not by command index, and does not alter controller behavior or safety gates.
