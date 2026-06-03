# MOVING_TO_START Tracking Analysis

- input_dir: `diagnostics/research_baseline_insert_precontact_clearance_gate_v1`
- result: `OK`
- command_index: `0`
- tracking_source: `trajectory_controller_state_samples.csv`
- command_duration_s: `40.000`
- post_command_hold_before_next_command_s: `6.287`
- samples: `11571`
- p95_max_abs_position_error_rad: `0.013236`
- worst_joint_by_p95_error: `joint_6` `0.013225` rad
- worst_joint_by_final_error: `joint_5` `0.004764` rad
- command_target_xyz_m: `0.520000, -0.200000, 0.885001`
- final_feedback_xyz_m: `0.520780, -0.199872, 0.885595`
- final_cartesian_error_xyz_m: `-0.000780, -0.000128, -0.000595`
- final_cartesian_error_norm_m: `0.000989`
- final_xy_error_m: `0.000791`
- xy_error_min_m: `0.000092`
- xy_error_mean_m: `0.196805`
- xy_error_p95_m: `0.405323`
- xy_error_max_m: `0.414666`
- strict_xy_sample_count: `759`
- strict_xy_sample_fraction: `0.065595`
- final_xy_window_s: `1.000`
- final_xy_window_min_m: `0.000148`
- final_xy_window_mean_m: `0.001675`
- final_xy_window_max_m: `0.004421`

## Per Joint Error

| joint | max_abs_rad | p95_abs_rad | mean_abs_rad | final_error_rad | target_minus_feedback_rad | target_rad | final_feedback_rad |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| joint_1 | 0.006324 | 0.004484 | 0.002156 | -0.002193 | -0.002193 | -0.502569 | -0.500376 |
| joint_2 | 0.007093 | 0.004159 | 0.001559 | 0.002362 | 0.002362 | -0.992068 | -0.994430 |
| joint_3 | 0.010649 | 0.004957 | 0.001918 | -0.000068 | -0.000068 | 2.340682 | 2.340749 |
| joint_4 | 0.012540 | 0.007211 | 0.003046 | -0.001003 | -0.001003 | -0.112781 | -0.111778 |
| joint_5 | 0.014273 | 0.008644 | 0.003693 | -0.004764 | -0.004764 | -1.349980 | -1.345216 |
| joint_6 | 0.018736 | 0.013225 | 0.005280 | 0.003025 | 0.003025 | 0.024802 | 0.021777 |

Interpretation: this is an offline diagnostic over passive trajectory observer CSVs. It selects the MOVING_TO_START command by peg-tip target pose, not by command index, and does not alter controller behavior or safety gates.
