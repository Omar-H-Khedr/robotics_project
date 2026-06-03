# Approach Tracking Analysis

- input_dir: `diagnostics/research_baseline_insert_cartesian_descent_v1`
- tracking_source: `trajectory_controller_state_samples.csv`
- command_index: `1`
- command_receipt_stamp_s: `46.102`
- next_command_stamp_s: `61.711`
- approach_command_duration_s: `15.000`
- observed_approach_window_s: `15.609`
- post_command_hold_before_next_command_s: `0.605`
- samples: `3902`
- max_abs_position_error_rad: `0.010298`
- p95_max_abs_position_error_rad: `0.008323`
- mean_rms_position_error_rad: `0.002948`
- worst_joint_by_p95_error: `joint_6` `0.008302` rad
- worst_joint_by_final_error: `joint_4` `0.005648` rad

## Cartesian Peg Tip Error

- command_target_xyz_m: `0.520000, -0.200000, 0.830000`
- final_feedback_xyz_m: `0.518344, -0.199884, 0.831086`
- final_cartesian_error_xyz_m: `0.001656, -0.000116, -0.001086`
- final_cartesian_error_norm_m: `0.001984`
- missing_descent_m: `0.001086`

## Per Joint Error

| joint | max_abs_rad | p95_abs_rad | mean_abs_rad | final_sample_error_rad | target_minus_feedback_rad | target_rad | final_feedback_rad |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| joint_1 | 0.006249 | 0.004303 | 0.001935 | 0.000633 | 0.000633 | -0.502769 | -0.503401 |
| joint_2 | 0.007865 | 0.005040 | 0.002041 | 0.001221 | 0.001221 | -0.894078 | -0.895299 |
| joint_3 | 0.006158 | 0.003434 | 0.001475 | 0.000535 | 0.000535 | 2.311157 | 2.310622 |
| joint_4 | 0.008877 | 0.005902 | 0.002509 | -0.005648 | -0.005648 | -0.109020 | -0.103373 |
| joint_5 | 0.008567 | 0.005676 | 0.002627 | 0.001113 | 0.001113 | -1.335299 | -1.336412 |
| joint_6 | 0.010298 | 0.008302 | 0.003944 | 0.001745 | 0.001745 | 0.032262 | 0.030517 |

Interpretation: this is an offline diagnostic over the passive tracking observer CSVs. It does not alter controller behavior or task safety gates.
