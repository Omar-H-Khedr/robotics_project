# Endpoint Hold Dynamics Analysis

- input_dir: `diagnostics/research_baseline_staged_withdrawal_v1`
- result: `OK`
- tracking_source: `trajectory_controller_state_samples.csv`
- command_index: `0`
- samples: `651`
- hold_duration_s: `2.599`
- xy_error_min_m: `0.000070`
- xy_error_mean_m: `0.002213`
- xy_error_p95_m: `0.004007`
- xy_error_max_m: `0.005637`
- cartesian_error_p95_m: `0.004744`
- x_range_m: `0.010169`
- y_range_m: `0.006817`
- z_range_m: `0.007967`
- strict_xy_sample_count: `294`
- strict_xy_sample_fraction: `0.451613`
- strict_bin_count: `0`
- max_consecutive_strict_bins: `0`
- largest_feedback_range_joint: `joint_6` `0.019785` rad

## Per Joint Hold Dynamics

| joint | target_rad | feedback_min_rad | feedback_max_rad | feedback_range_rad | mean_abs_error_rad | p95_abs_error_rad | final_error_rad |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| joint_1 | -0.502569 | -0.508939 | -0.497424 | 0.011515 | 0.002377 | 0.004999 | -0.000085 |
| joint_2 | -0.992068 | -0.995750 | -0.984257 | 0.011493 | 0.001912 | 0.004824 | 0.000054 |
| joint_3 | 2.340682 | 2.336048 | 2.345511 | 0.009463 | 0.001348 | 0.003553 | 0.000296 |
| joint_4 | -0.112781 | -0.121439 | -0.104953 | 0.016486 | 0.002559 | 0.006236 | -0.001533 |
| joint_5 | -1.349980 | -1.357945 | -1.341948 | 0.015998 | 0.002649 | 0.005856 | -0.002858 |
| joint_6 | 0.024802 | 0.015276 | 0.035061 | 0.019785 | 0.003948 | 0.008153 | -0.000821 |

Interpretation: this diagnostic uses passive trajectory observer data after the axis-align command duration has elapsed and before the next trajectory command. It does not alter controller behavior or safety gates.
