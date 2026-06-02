# Approach Tracking Analysis

- input_dir: `diagnostics/research_baseline_joint_effort_scale_2p0_v1`
- command_receipt_stamp_s: `43.627`
- next_command_stamp_s: `43.910`
- approach_command_duration_s: `15.000`
- observed_approach_window_s: `0.283`
- post_command_hold_before_next_command_s: `0.000`
- samples: `71`
- max_abs_position_error_rad: `0.049733`
- p95_max_abs_position_error_rad: `0.042579`
- mean_rms_position_error_rad: `0.011237`
- worst_joint_by_p95_error: `joint_6` `0.042579` rad
- worst_joint_by_final_error: `joint_6` `0.028977` rad

## Cartesian Peg Tip Error

- command_target_xyz_m: `0.520000, -0.200000, 0.830000`
- final_feedback_xyz_m: `0.520038, -0.200880, 0.890982`
- final_cartesian_error_xyz_m: `-0.000038, 0.000880, -0.060982`
- final_cartesian_error_norm_m: `0.060988`
- missing_descent_m: `-0.060982`

## Per Joint Error

| joint | max_abs_rad | p95_abs_rad | mean_abs_rad | final_sample_error_rad | target_minus_feedback_rad | target_rad | final_feedback_rad |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| joint_1 | 0.006923 | 0.005411 | 0.002642 | -0.002535 | 0.000433 | -0.520188 | -0.520621 |
| joint_2 | 0.003487 | 0.002967 | 0.001190 | 0.002319 | 0.099473 | -0.569309 | -0.668782 |
| joint_3 | 0.007029 | 0.006297 | 0.002971 | -0.004385 | -0.034505 | 2.247343 | 2.281848 |
| joint_4 | 0.027796 | 0.017246 | 0.007974 | -0.005396 | -0.003447 | -0.172363 | -0.168916 |
| joint_5 | 0.017283 | 0.015197 | 0.006121 | -0.010717 | 0.024438 | -1.597543 | -1.621981 |
| joint_6 | 0.049733 | 0.042579 | 0.023355 | 0.028977 | 0.029199 | 0.000808 | -0.028391 |

Interpretation: this is an offline diagnostic over the passive tracking observer CSVs. It does not alter controller behavior or task safety gates.
