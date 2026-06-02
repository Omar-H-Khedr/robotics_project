# Approach Tracking Analysis

- input_dir: `diagnostics/research_baseline_approach_gain_3000_v1`
- command_receipt_stamp_s: `48.723`
- next_command_stamp_s: `92.805`
- approach_command_duration_s: `15.000`
- observed_approach_window_s: `44.082`
- post_command_hold_before_next_command_s: `29.081`
- samples: `11021`
- max_abs_position_error_rad: `0.111631`
- p95_max_abs_position_error_rad: `0.110990`
- mean_rms_position_error_rad: `0.037577`
- worst_joint_by_p95_error: `joint_2` `0.110990` rad
- worst_joint_by_final_error: `joint_2` `0.110880` rad

## Cartesian Peg Tip Error

- command_target_xyz_m: `0.520000, -0.200000, 0.830000`
- final_feedback_xyz_m: `0.511485, -0.182375, 0.899807`
- final_cartesian_error_xyz_m: `0.008515, -0.017625, -0.069807`
- final_cartesian_error_norm_m: `0.072500`
- missing_descent_m: `-0.069807`

## Per Joint Error

| joint | max_abs_rad | p95_abs_rad | mean_abs_rad | final_error_rad | target_rad | final_feedback_rad |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| joint_1 | 0.006128 | 0.001999 | 0.000872 | -0.000788 | -0.526200 | -0.525412 |
| joint_2 | 0.111631 | 0.110990 | 0.089108 | 0.110880 | -0.552738 | -0.663619 |
| joint_3 | 0.007021 | 0.002809 | 0.001107 | 0.001718 | 2.238869 | 2.237151 |
| joint_4 | 0.039887 | 0.017105 | 0.007662 | 0.004532 | -0.192657 | -0.197189 |
| joint_5 | 0.034808 | 0.021580 | 0.008806 | 0.000414 | -1.648699 | -1.649113 |
| joint_6 | 0.041436 | 0.015259 | 0.006575 | 0.010650 | -0.022223 | -0.032873 |

Interpretation: this is an offline diagnostic over the passive tracking observer CSVs. It does not alter controller behavior or task safety gates.
