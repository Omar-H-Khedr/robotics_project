# Approach Tracking Analysis

- input_dir: `ros2_ws/diagnostics/research_baseline_slow_approach_descent_v1`
- command_receipt_stamp_s: `50.539`
- next_command_stamp_s: `94.534`
- approach_command_duration_s: `41.662`
- observed_approach_window_s: `43.995`
- post_command_hold_before_next_command_s: `2.327`
- samples: `10998`
- max_abs_position_error_rad: `0.108618`
- p95_max_abs_position_error_rad: `0.106741`
- mean_rms_position_error_rad: `0.022729`
- worst_joint_by_p95_error: `joint_2` `0.106741` rad
- worst_joint_by_final_error: `joint_2` `0.108545` rad

## Cartesian Peg Tip Error

- command_target_xyz_m: `0.520000, -0.200000, 0.830000`
- final_feedback_xyz_m: `0.510620, -0.183198, 0.897465`
- final_cartesian_error_xyz_m: `0.009380, -0.016802, -0.067465`
- final_cartesian_error_norm_m: `0.070156`
- missing_descent_m: `-0.067465`

## Per Joint Error

| joint | max_abs_rad | p95_abs_rad | mean_abs_rad | final_sample_error_rad | target_minus_feedback_rad | target_rad | final_feedback_rad |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| joint_1 | 0.008177 | 0.002334 | 0.001015 | -0.000734 | -0.000734 | -0.524815 | -0.524080 |
| joint_2 | 0.108618 | 0.106741 | 0.050161 | 0.108545 | 0.108545 | -0.556703 | -0.665248 |
| joint_3 | 0.007391 | 0.002813 | 0.001098 | 0.001060 | 0.001060 | 2.240983 | 2.239923 |
| joint_4 | 0.033039 | 0.017744 | 0.008026 | -0.002538 | -0.002538 | -0.187977 | -0.185439 |
| joint_5 | 0.038487 | 0.022580 | 0.009836 | -0.003844 | -0.003844 | -1.636479 | -1.632635 |
| joint_6 | 0.024086 | 0.014537 | 0.006331 | 0.012813 | 0.012813 | -0.016091 | -0.028904 |

Interpretation: this is an offline diagnostic over the passive tracking observer CSVs. It does not alter controller behavior or task safety gates.
