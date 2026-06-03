# Approach Tracking Analysis

- input_dir: `diagnostics/research_baseline_approach_z_precondition_gate_v1`
- tracking_source: `trajectory_controller_state_samples.csv`
- command_index: `1`
- command_receipt_stamp_s: `43.673`
- next_command_stamp_s: `55.500`
- approach_command_duration_s: `15.000`
- observed_approach_window_s: `11.827`
- post_command_hold_before_next_command_s: `0.000`
- samples: `2955`
- max_abs_position_error_rad: `0.010250`
- p95_max_abs_position_error_rad: `0.008323`
- mean_rms_position_error_rad: `0.002937`
- worst_joint_by_p95_error: `joint_6` `0.008280` rad
- worst_joint_by_final_error: `joint_6` `0.007423` rad

## Cartesian Peg Tip Error

- command_target_xyz_m: `0.520000, -0.200000, 0.830000`
- final_feedback_xyz_m: `0.517594, -0.202990, 0.840723`
- final_cartesian_error_xyz_m: `0.002406, 0.002990, -0.010723`
- final_cartesian_error_norm_m: `0.011389`
- missing_descent_m: `0.010723`

## Per Joint Error

| joint | max_abs_rad | p95_abs_rad | mean_abs_rad | final_sample_error_rad | target_minus_feedback_rad | target_rad | final_feedback_rad |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| joint_1 | 0.006165 | 0.004228 | 0.001954 | 0.003434 | 0.003658 | -0.501466 | -0.505123 |
| joint_2 | 0.006649 | 0.004702 | 0.001860 | -0.003040 | 0.020280 | -0.894532 | -0.914812 |
| joint_3 | 0.005305 | 0.003498 | 0.001494 | -0.001569 | -0.009332 | 2.310996 | 2.320328 |
| joint_4 | 0.008690 | 0.005864 | 0.002526 | -0.003513 | -0.003284 | -0.111637 | -0.108353 |
| joint_5 | 0.008514 | 0.005843 | 0.002659 | -0.000515 | 0.003376 | -1.333505 | -1.336881 |
| joint_6 | 0.010250 | 0.008280 | 0.003940 | 0.007423 | 0.007148 | 0.023052 | 0.015905 |

Interpretation: this is an offline diagnostic over the passive tracking observer CSVs. It does not alter controller behavior or task safety gates.
