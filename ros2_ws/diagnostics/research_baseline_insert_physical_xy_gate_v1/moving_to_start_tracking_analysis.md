# MOVING_TO_START Tracking Analysis

- input_dir: `diagnostics/research_baseline_insert_physical_xy_gate_v1`
- result: `OK`
- command_index: `0`
- tracking_source: `trajectory_controller_state_samples.csv`
- command_duration_s: `40.000`
- post_command_hold_before_next_command_s: `7.713`
- samples: `11929`
- p95_max_abs_position_error_rad: `0.013038`
- worst_joint_by_p95_error: `joint_6` `0.012996` rad
- worst_joint_by_final_error: `joint_6` `0.007031` rad
- command_target_xyz_m: `0.520000, -0.200000, 0.885001`
- final_feedback_xyz_m: `0.521943, -0.199910, 0.884200`
- final_cartesian_error_xyz_m: `-0.001943, -0.000090, 0.000801`
- final_cartesian_error_norm_m: `0.002103`
- final_xy_error_m: `0.001945`
- xy_error_min_m: `0.000056`
- xy_error_mean_m: `0.189732`
- xy_error_p95_m: `0.401799`
- xy_error_max_m: `0.413500`
- strict_xy_sample_count: `1107`
- strict_xy_sample_fraction: `0.092799`
- final_xy_window_s: `1.000`
- final_xy_window_min_m: `0.000138`
- final_xy_window_mean_m: `0.002033`
- final_xy_window_max_m: `0.004675`

## Per Joint Error

| joint | max_abs_rad | p95_abs_rad | mean_abs_rad | final_error_rad | target_minus_feedback_rad | target_rad | final_feedback_rad |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| joint_1 | 0.006731 | 0.004526 | 0.002115 | -0.004502 | -0.004502 | -0.502569 | -0.498067 |
| joint_2 | 0.008274 | 0.004092 | 0.001520 | -0.000186 | -0.000186 | -0.992068 | -0.991882 |
| joint_3 | 0.010874 | 0.004762 | 0.001849 | -0.000833 | -0.000833 | 2.340682 | 2.341515 |
| joint_4 | 0.012509 | 0.007173 | 0.003033 | -0.000752 | -0.000752 | -0.112781 | -0.112029 |
| joint_5 | 0.014435 | 0.008509 | 0.003640 | -0.002518 | -0.002518 | -1.349980 | -1.347462 |
| joint_6 | 0.018661 | 0.012996 | 0.005243 | 0.007031 | 0.007031 | 0.024802 | 0.017771 |

Interpretation: this is an offline diagnostic over passive trajectory observer CSVs. It selects the MOVING_TO_START command by peg-tip target pose, not by command index, and does not alter controller behavior or safety gates.
