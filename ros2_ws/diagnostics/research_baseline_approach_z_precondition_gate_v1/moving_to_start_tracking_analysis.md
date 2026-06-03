# MOVING_TO_START Tracking Analysis

- input_dir: `diagnostics/research_baseline_approach_z_precondition_gate_v1`
- result: `OK`
- command_index: `0`
- tracking_source: `trajectory_controller_state_samples.csv`
- command_duration_s: `40.000`
- post_command_hold_before_next_command_s: `1.130`
- samples: `10282`
- p95_max_abs_position_error_rad: `0.013483`
- worst_joint_by_p95_error: `joint_6` `0.013477` rad
- worst_joint_by_final_error: `joint_6` `0.006837` rad
- command_target_xyz_m: `0.520000, -0.200000, 0.885001`
- final_feedback_xyz_m: `0.520075, -0.200808, 0.884704`
- final_cartesian_error_xyz_m: `-0.000075, 0.000808, 0.000297`
- final_cartesian_error_norm_m: `0.000864`
- final_xy_error_m: `0.000812`
- xy_error_min_m: `0.000130`
- xy_error_mean_m: `0.220932`
- xy_error_p95_m: `0.405622`
- xy_error_max_m: `0.415806`
- strict_xy_sample_count: `149`
- strict_xy_sample_fraction: `0.014491`
- final_xy_window_s: `1.000`
- final_xy_window_min_m: `0.000173`
- final_xy_window_mean_m: `0.002267`
- final_xy_window_max_m: `0.004883`

## Per Joint Error

| joint | max_abs_rad | p95_abs_rad | mean_abs_rad | final_error_rad | target_minus_feedback_rad | target_rad | final_feedback_rad |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| joint_1 | 0.006432 | 0.004511 | 0.002158 | 0.000082 | 0.000082 | -0.502569 | -0.502652 |
| joint_2 | 0.009085 | 0.004192 | 0.001533 | 0.000263 | 0.000263 | -0.992068 | -0.992330 |
| joint_3 | 0.010259 | 0.004991 | 0.001951 | -0.001124 | -0.001124 | 2.340682 | 2.341805 |
| joint_4 | 0.012274 | 0.007333 | 0.003123 | 0.004056 | 0.004056 | -0.112781 | -0.116837 |
| joint_5 | 0.014238 | 0.008807 | 0.003811 | -0.000647 | -0.000647 | -1.349980 | -1.349334 |
| joint_6 | 0.018834 | 0.013477 | 0.005440 | 0.006837 | 0.006837 | 0.024802 | 0.017965 |

Interpretation: this is an offline diagnostic over passive trajectory observer CSVs. It selects the MOVING_TO_START command by peg-tip target pose, not by command index, and does not alter controller behavior or safety gates.
