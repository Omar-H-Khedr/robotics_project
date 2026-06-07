# Endpoint Hold Dynamics Analysis

- input_dir: `diagnostics/research_baseline_single_gz_control_25hz_v1`
- result: `OK`
- tracking_source: `trajectory_controller_state_samples.csv`
- command_index: `0`
- samples: `1021`
- hold_duration_s: `4.080`
- xy_error_min_m: `0.000049`
- xy_error_mean_m: `0.002033`
- xy_error_p95_m: `0.003678`
- xy_error_max_m: `0.004960`
- cartesian_error_p95_m: `0.004513`
- x_range_m: `0.009079`
- y_range_m: `0.007749`
- z_range_m: `0.007704`
- strict_xy_sample_count: `532`
- strict_xy_sample_fraction: `0.521058`
- strict_bin_count: `3`
- max_consecutive_strict_bins: `2`
- estimated_state_loop_hz: `25.0`
- largest_feedback_range_joint: `joint_6` `0.020187` rad

## Per Joint Hold Dynamics

| joint | target_rad | feedback_min_rad | feedback_max_rad | feedback_range_rad | mean_abs_error_rad | p95_abs_error_rad | final_error_rad |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| joint_1 | -0.502569 | -0.508700 | -0.496154 | 0.012546 | 0.001993 | 0.004288 | -0.002052 |
| joint_2 | -0.992068 | -0.994823 | -0.985944 | 0.008879 | 0.001649 | 0.003993 | -0.001450 |
| joint_3 | 2.340682 | 2.336660 | 2.345814 | 0.009154 | 0.001420 | 0.003255 | -0.002655 |
| joint_4 | -0.112781 | -0.121908 | -0.104685 | 0.017223 | 0.002601 | 0.006020 | 0.005554 |
| joint_5 | -1.349980 | -1.357073 | -1.341575 | 0.015498 | 0.002671 | 0.005751 | -0.005601 |
| joint_6 | 0.024802 | 0.014736 | 0.034923 | 0.020187 | 0.003978 | 0.008469 | 0.002482 |

Interpretation: this diagnostic uses passive trajectory observer data after the axis-align command duration has elapsed and before the next trajectory command. It does not alter controller behavior or safety gates.
