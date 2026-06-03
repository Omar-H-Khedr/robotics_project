# Approach Tracking Analysis

- input_dir: `diagnostics/research_baseline_staged_withdrawal_v1`
- tracking_source: `trajectory_controller_state_samples.csv`
- command_index: `1`
- command_receipt_stamp_s: `45.211`
- next_command_stamp_s: `57.307`
- approach_command_duration_s: `15.000`
- observed_approach_window_s: `12.096`
- post_command_hold_before_next_command_s: `0.000`
- samples: `3024`
- max_abs_position_error_rad: `0.010418`
- p95_max_abs_position_error_rad: `0.008308`
- mean_rms_position_error_rad: `0.002922`
- worst_joint_by_p95_error: `joint_6` `0.008284` rad
- worst_joint_by_final_error: `joint_6` `0.007126` rad

## Cartesian Peg Tip Error

- command_target_xyz_m: `0.520000, -0.200000, 0.830000`
- final_feedback_xyz_m: `0.520815, -0.202372, 0.841424`
- final_cartesian_error_xyz_m: `-0.000815, 0.002372, -0.011424`
- final_cartesian_error_norm_m: `0.011696`
- missing_descent_m: `0.011424`

## Per Joint Error

| joint | max_abs_rad | p95_abs_rad | mean_abs_rad | final_sample_error_rad | target_minus_feedback_rad | target_rad | final_feedback_rad |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| joint_1 | 0.007095 | 0.004062 | 0.001861 | 0.002318 | 0.002514 | -0.500782 | -0.503296 |
| joint_2 | 0.008284 | 0.004776 | 0.001929 | 0.001481 | 0.022325 | -0.895378 | -0.917703 |
| joint_3 | 0.005474 | 0.003344 | 0.001426 | -0.003863 | -0.010811 | 2.310488 | 2.321299 |
| joint_4 | 0.008632 | 0.005893 | 0.002569 | 0.000943 | 0.001143 | -0.115439 | -0.116582 |
| joint_5 | 0.008129 | 0.005745 | 0.002632 | -0.001026 | 0.002422 | -1.329694 | -1.332115 |
| joint_6 | 0.010418 | 0.008284 | 0.003930 | -0.007126 | -0.007347 | 0.015877 | 0.023224 |

Interpretation: this is an offline diagnostic over the passive tracking observer CSVs. It does not alter controller behavior or task safety gates.
