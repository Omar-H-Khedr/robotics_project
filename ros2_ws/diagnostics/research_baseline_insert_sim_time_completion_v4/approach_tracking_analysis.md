# Approach Tracking Analysis

- input_dir: `diagnostics/research_baseline_insert_sim_time_completion_v4`
- tracking_source: `trajectory_controller_state_samples.csv`
- command_index: `1`
- command_receipt_stamp_s: `45.508`
- next_command_stamp_s: `57.608`
- approach_command_duration_s: `15.000`
- observed_approach_window_s: `12.100`
- post_command_hold_before_next_command_s: `0.000`
- samples: `3025`
- max_abs_position_error_rad: `0.010233`
- p95_max_abs_position_error_rad: `0.008434`
- mean_rms_position_error_rad: `0.002952`
- worst_joint_by_p95_error: `joint_6` `0.008405` rad
- worst_joint_by_final_error: `joint_6` `0.005579` rad

## Cartesian Peg Tip Error

- command_target_xyz_m: `0.520000, -0.200000, 0.830000`
- final_feedback_xyz_m: `0.523296, -0.199907, 0.840971`
- final_cartesian_error_xyz_m: `-0.003296, -0.000093, -0.010971`
- final_cartesian_error_norm_m: `0.011455`
- missing_descent_m: `0.010971`

## Per Joint Error

| joint | max_abs_rad | p95_abs_rad | mean_abs_rad | final_sample_error_rad | target_minus_feedback_rad | target_rad | final_feedback_rad |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| joint_1 | 0.006978 | 0.004444 | 0.002009 | -0.002713 | -0.002517 | -0.500565 | -0.498049 |
| joint_2 | 0.008037 | 0.004782 | 0.001950 | -0.001091 | 0.019922 | -0.894436 | -0.914358 |
| joint_3 | 0.006203 | 0.003721 | 0.001533 | -0.002183 | -0.009193 | 2.311161 | 2.320354 |
| joint_4 | 0.008974 | 0.005792 | 0.002518 | 0.003676 | 0.003877 | -0.110812 | -0.114688 |
| joint_5 | 0.008449 | 0.005683 | 0.002620 | 0.002537 | 0.006042 | -1.334100 | -1.340143 |
| joint_6 | 0.010233 | 0.008405 | 0.003963 | -0.005579 | -0.005811 | 0.019443 | 0.025254 |

Interpretation: this is an offline diagnostic over the passive tracking observer CSVs. It does not alter controller behavior or task safety gates.
