# Approach Tracking Analysis

- input_dir: `diagnostics/research_baseline_retreat_clearance_lift_v1`
- tracking_source: `trajectory_controller_state_samples.csv`
- command_index: `1`
- command_receipt_stamp_s: `48.115`
- next_command_stamp_s: `60.054`
- approach_command_duration_s: `15.000`
- observed_approach_window_s: `11.939`
- post_command_hold_before_next_command_s: `0.000`
- samples: `2985`
- max_abs_position_error_rad: `0.010440`
- p95_max_abs_position_error_rad: `0.008305`
- mean_rms_position_error_rad: `0.002932`
- worst_joint_by_p95_error: `joint_6` `0.008286` rad
- worst_joint_by_final_error: `` `0.000000` rad

## Cartesian Peg Tip Error

- command_target_xyz_m: `0.520000, -0.200000, 0.830000`
- final_feedback_xyz_m: `0.517888, -0.200970, 0.841587`
- final_cartesian_error_xyz_m: `0.002112, 0.000970, -0.011587`
- final_cartesian_error_norm_m: `0.011818`
- missing_descent_m: `0.011587`

## Per Joint Error

| joint | max_abs_rad | p95_abs_rad | mean_abs_rad | final_sample_error_rad | target_minus_feedback_rad | target_rad | final_feedback_rad |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| joint_1 | 0.006017 | 0.004021 | 0.001971 | 0.000000 | 0.002853 | -0.502248 | -0.505102 |
| joint_2 | 0.007939 | 0.004848 | 0.001923 | 0.000000 | 0.020551 | -0.894478 | -0.915029 |
| joint_3 | 0.005363 | 0.003484 | 0.001415 | 0.000000 | -0.006909 | 2.310944 | 2.317852 |
| joint_4 | 0.008430 | 0.005869 | 0.002534 | 0.000000 | -0.002576 | -0.113582 | -0.111006 |
| joint_5 | 0.008382 | 0.005814 | 0.002662 | 0.000000 | 0.003840 | -1.333651 | -1.337491 |
| joint_6 | 0.010440 | 0.008286 | 0.003934 | 0.000000 | 0.002094 | 0.024686 | 0.022592 |

Interpretation: this is an offline diagnostic over the passive tracking observer CSVs. It does not alter controller behavior or task safety gates.
