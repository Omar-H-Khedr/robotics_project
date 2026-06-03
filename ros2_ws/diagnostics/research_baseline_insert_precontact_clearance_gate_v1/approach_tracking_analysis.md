# Approach Tracking Analysis

- input_dir: `diagnostics/research_baseline_insert_precontact_clearance_gate_v1`
- tracking_source: `trajectory_controller_state_samples.csv`
- command_index: `1`
- command_receipt_stamp_s: `48.709`
- next_command_stamp_s: `63.203`
- approach_command_duration_s: `15.000`
- observed_approach_window_s: `14.494`
- post_command_hold_before_next_command_s: `0.000`
- samples: `3624`
- max_abs_position_error_rad: `0.010365`
- p95_max_abs_position_error_rad: `0.008284`
- mean_rms_position_error_rad: `0.002942`
- worst_joint_by_p95_error: `joint_6` `0.008244` rad
- worst_joint_by_final_error: `joint_6` `0.004979` rad

## Cartesian Peg Tip Error

- command_target_xyz_m: `0.520000, -0.200000, 0.830000`
- final_feedback_xyz_m: `0.518528, -0.201499, 0.830856`
- final_cartesian_error_xyz_m: `0.001472, 0.001499, -0.000856`
- final_cartesian_error_norm_m: `0.002268`
- missing_descent_m: `0.000856`

## Per Joint Error

| joint | max_abs_rad | p95_abs_rad | mean_abs_rad | final_sample_error_rad | target_minus_feedback_rad | target_rad | final_feedback_rad |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| joint_1 | 0.006246 | 0.004528 | 0.002055 | 0.001357 | 0.001396 | -0.502463 | -0.503859 |
| joint_2 | 0.006613 | 0.004677 | 0.001910 | -0.001828 | 0.002104 | -0.894943 | -0.897046 |
| joint_3 | 0.005380 | 0.003335 | 0.001389 | -0.000473 | -0.001796 | 2.310599 | 2.312395 |
| joint_4 | 0.008674 | 0.005875 | 0.002556 | -0.003836 | -0.003795 | -0.110463 | -0.106667 |
| joint_5 | 0.008508 | 0.005642 | 0.002635 | -0.000652 | -0.000003 | -1.331329 | -1.331326 |
| joint_6 | 0.010365 | 0.008244 | 0.003940 | 0.004979 | 0.004928 | 0.029665 | 0.024737 |

Interpretation: this is an offline diagnostic over the passive tracking observer CSVs. It does not alter controller behavior or task safety gates.
