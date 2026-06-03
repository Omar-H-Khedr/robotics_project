# MOVING_TO_START Tracking Analysis

- input_dir: `diagnostics/research_baseline_staged_withdrawal_v1`
- result: `OK`
- command_index: `0`
- tracking_source: `trajectory_controller_state_samples.csv`
- command_duration_s: `40.000`
- post_command_hold_before_next_command_s: `2.599`
- samples: `10650`
- p95_max_abs_position_error_rad: `0.013315`
- worst_joint_by_p95_error: `joint_6` `0.013280` rad
- worst_joint_by_final_error: `joint_5` `0.002858` rad
- command_target_xyz_m: `0.520000, -0.200000, 0.885001`
- final_feedback_xyz_m: `0.519963, -0.200180, 0.884620`
- final_cartesian_error_xyz_m: `0.000037, 0.000179, 0.000381`
- final_cartesian_error_norm_m: `0.000423`
- final_xy_error_m: `0.000183`
- xy_error_min_m: `0.000070`
- xy_error_mean_m: `0.214138`
- xy_error_p95_m: `0.407466`
- xy_error_max_m: `0.415419`
- strict_xy_sample_count: `311`
- strict_xy_sample_fraction: `0.029202`
- final_xy_window_s: `1.000`
- final_xy_window_min_m: `0.000070`
- final_xy_window_mean_m: `0.001939`
- final_xy_window_max_m: `0.004368`

## Per Joint Error

| joint | max_abs_rad | p95_abs_rad | mean_abs_rad | final_error_rad | target_minus_feedback_rad | target_rad | final_feedback_rad |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| joint_1 | 0.006370 | 0.004560 | 0.002208 | -0.000085 | -0.000085 | -0.502569 | -0.502485 |
| joint_2 | 0.008229 | 0.004203 | 0.001572 | 0.000054 | 0.000054 | -0.992068 | -0.992122 |
| joint_3 | 0.009939 | 0.004951 | 0.001887 | 0.000296 | 0.000296 | 2.340682 | 2.340386 |
| joint_4 | 0.012367 | 0.007310 | 0.003087 | -0.001533 | -0.001533 | -0.112781 | -0.111248 |
| joint_5 | 0.014511 | 0.008718 | 0.003769 | -0.002858 | -0.002858 | -1.349980 | -1.347123 |
| joint_6 | 0.018717 | 0.013280 | 0.005347 | -0.000821 | -0.000821 | 0.024802 | 0.025623 |

Interpretation: this is an offline diagnostic over passive trajectory observer CSVs. It selects the MOVING_TO_START command by peg-tip target pose, not by command index, and does not alter controller behavior or safety gates.
