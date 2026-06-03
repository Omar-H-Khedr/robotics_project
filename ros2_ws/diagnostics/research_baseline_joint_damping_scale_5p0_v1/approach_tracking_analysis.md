# Approach Tracking Analysis

- input_dir: `diagnostics/research_baseline_joint_damping_scale_5p0_v1`
- tracking_source: `trajectory_controller_state_samples.csv`
- command_index: `1`
- command_receipt_stamp_s: `43.420`
- next_command_stamp_s: `52.860`
- approach_command_duration_s: `15.000`
- observed_approach_window_s: `9.440`
- post_command_hold_before_next_command_s: `0.000`
- samples: `2359`
- max_abs_position_error_rad: `0.010278`
- p95_max_abs_position_error_rad: `0.008192`
- mean_rms_position_error_rad: `0.002912`
- worst_joint_by_p95_error: `joint_6` `0.008161` rad
- worst_joint_by_final_error: `joint_6` `0.004515` rad

## Cartesian Peg Tip Error

- command_target_xyz_m: `0.520000, -0.200000, 0.830000`
- final_feedback_xyz_m: `0.521268, -0.202477, 0.849622`
- final_cartesian_error_xyz_m: `-0.001268, 0.002477, -0.019622`
- final_cartesian_error_norm_m: `0.019818`
- missing_descent_m: `0.019622`

## Per Joint Error

| joint | max_abs_rad | p95_abs_rad | mean_abs_rad | final_sample_error_rad | target_minus_feedback_rad | target_rad | final_feedback_rad |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| joint_1 | 0.006442 | 0.004029 | 0.001906 | 0.001418 | 0.001821 | -0.502085 | -0.503907 |
| joint_2 | 0.007974 | 0.004760 | 0.001857 | -0.004044 | 0.037407 | -0.893959 | -0.931366 |
| joint_3 | 0.005697 | 0.003424 | 0.001439 | -0.002348 | -0.015898 | 2.311318 | 2.327216 |
| joint_4 | 0.008232 | 0.005987 | 0.002558 | 0.001990 | 0.002401 | -0.110422 | -0.112823 |
| joint_5 | 0.008422 | 0.005807 | 0.002615 | -0.003190 | 0.003829 | -1.336052 | -1.339880 |
| joint_6 | 0.010278 | 0.008161 | 0.003923 | -0.004515 | -0.005031 | 0.027152 | 0.032183 |

Interpretation: this is an offline diagnostic over the passive tracking observer CSVs. It does not alter controller behavior or task safety gates.
