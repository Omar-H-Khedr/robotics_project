# MOVING_TO_START Tracking Analysis

- input_dir: `diagnostics/research_baseline_joint_damping_scale_2p0_v1`
- result: `OK`
- command_index: `0`
- command_duration_s: `40.000`
- post_command_hold_before_next_command_s: `22.315`
- samples: `15579`
- p95_max_abs_position_error_rad: `0.015812`
- worst_joint_by_p95_error: `joint_6` `0.014434` rad
- worst_joint_by_final_error: `joint_1` `0.007081` rad
- command_target_xyz_m: `0.520000, -0.200000, 0.885001`
- final_feedback_xyz_m: `0.514217, -0.199750, 0.883783`
- final_cartesian_error_xyz_m: `0.005783, -0.000250, 0.001218`
- final_cartesian_error_norm_m: `0.005915`
- final_xy_error_m: `0.005788`

## Per Joint Error

| joint | max_abs_rad | p95_abs_rad | mean_abs_rad | final_error_rad | target_minus_feedback_rad | target_rad | final_feedback_rad |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| joint_1 | 0.018082 | 0.010570 | 0.005187 | 0.007081 | 0.007081 | -0.502569 | -0.509651 |
| joint_2 | 0.016828 | 0.009507 | 0.003795 | -0.004713 | -0.004713 | -0.992068 | -0.987354 |
| joint_3 | 0.023570 | 0.011372 | 0.004542 | 0.005362 | 0.005362 | 2.340682 | 2.335320 |
| joint_4 | 0.019794 | 0.010521 | 0.004246 | -0.002697 | -0.002697 | -0.112781 | -0.110084 |
| joint_5 | 0.019767 | 0.010992 | 0.004583 | 0.000596 | 0.000596 | -1.349980 | -1.350577 |
| joint_6 | 0.028423 | 0.014434 | 0.005785 | 0.001431 | 0.001431 | 0.024802 | 0.023371 |

Interpretation: this is an offline diagnostic over passive trajectory observer CSVs. It selects the MOVING_TO_START command by peg-tip target pose, not by command index, and does not alter controller behavior or safety gates.
