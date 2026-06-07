# Endpoint Hold Dynamics Analysis

- input_dir: `diagnostics/research_baseline_search_damping10_gain3000_25hz_v1`
- result: `OK`
- tracking_source: `trajectory_controller_state_samples.csv`
- command_index: `0`
- samples: `5`
- hold_duration_s: `0.016`
- xy_error_min_m: `0.000562`
- xy_error_mean_m: `0.001132`
- xy_error_p95_m: `0.001624`
- xy_error_max_m: `0.001624`
- cartesian_error_p95_m: `0.001956`
- x_range_m: `0.002269`
- y_range_m: `0.001538`
- z_range_m: `0.002947`
- strict_xy_sample_count: `5`
- strict_xy_sample_fraction: `1.000000`
- strict_bin_count: `1`
- max_consecutive_strict_bins: `1`
- estimated_state_loop_hz: `25.0`
- largest_feedback_range_joint: `joint_6` `0.008485` rad

## Per Joint Hold Dynamics

| joint | target_rad | feedback_min_rad | feedback_max_rad | feedback_range_rad | mean_abs_error_rad | p95_abs_error_rad | final_error_rad |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| joint_1 | -0.502569 | -0.504343 | -0.501795 | 0.002547 | 0.000921 | 0.001773 | -0.000289 |
| joint_2 | -0.992068 | -0.992627 | -0.989889 | 0.002739 | 0.001202 | 0.002179 | -0.002179 |
| joint_3 | 2.340682 | 2.338634 | 2.340811 | 0.002178 | 0.000999 | 0.002048 | 0.000471 |
| joint_4 | -0.112781 | -0.113931 | -0.109029 | 0.004902 | 0.001351 | 0.003752 | 0.000605 |
| joint_5 | -1.349980 | -1.354344 | -1.346313 | 0.008031 | 0.002651 | 0.004363 | -0.003667 |
| joint_6 | 0.024802 | 0.019521 | 0.028006 | 0.008485 | 0.003755 | 0.005281 | 0.004452 |

Interpretation: this diagnostic uses passive trajectory observer data after the axis-align command duration has elapsed and before the next trajectory command. It does not alter controller behavior or safety gates.
