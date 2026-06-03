# Approach Tracking Analysis

- input_dir: `diagnostics/research_baseline_insert_sideload_abort_v1`
- tracking_source: `trajectory_controller_state_samples.csv`
- command_index: `1`
- command_receipt_stamp_s: `42.901`
- next_command_stamp_s: `54.808`
- approach_command_duration_s: `15.000`
- observed_approach_window_s: `11.907`
- post_command_hold_before_next_command_s: `0.000`
- samples: `2976`
- max_abs_position_error_rad: `0.010423`
- p95_max_abs_position_error_rad: `0.008356`
- mean_rms_position_error_rad: `0.002923`
- worst_joint_by_p95_error: `joint_6` `0.008340` rad
- worst_joint_by_final_error: `joint_6` `0.007107` rad

## Cartesian Peg Tip Error

- command_target_xyz_m: `0.520000, -0.200000, 0.830000`
- final_feedback_xyz_m: `0.519738, -0.201547, 0.841587`
- final_cartesian_error_xyz_m: `0.000262, 0.001547, -0.011587`
- final_cartesian_error_norm_m: `0.011692`
- missing_descent_m: `0.011587`

## Per Joint Error

| joint | max_abs_rad | p95_abs_rad | mean_abs_rad | final_sample_error_rad | target_minus_feedback_rad | target_rad | final_feedback_rad |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| joint_1 | 0.006083 | 0.004142 | 0.001827 | 0.002300 | 0.002518 | -0.501561 | -0.504079 |
| joint_2 | 0.008456 | 0.004984 | 0.001946 | -0.001089 | 0.021639 | -0.894782 | -0.916422 |
| joint_3 | 0.005372 | 0.003474 | 0.001432 | -0.001394 | -0.008961 | 2.310813 | 2.319774 |
| joint_4 | 0.008013 | 0.005719 | 0.002516 | -0.002983 | -0.002758 | -0.111399 | -0.108641 |
| joint_5 | 0.008533 | 0.005784 | 0.002649 | -0.000849 | 0.002934 | -1.332298 | -1.335232 |
| joint_6 | 0.010423 | 0.008340 | 0.003936 | -0.007107 | -0.007379 | 0.023974 | 0.031353 |

Interpretation: this is an offline diagnostic over the passive tracking observer CSVs. It does not alter controller behavior or task safety gates.
