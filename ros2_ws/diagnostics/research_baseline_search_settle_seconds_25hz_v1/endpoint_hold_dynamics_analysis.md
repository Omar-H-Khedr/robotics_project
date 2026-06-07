# Endpoint Hold Dynamics Analysis

- input_dir: `diagnostics/research_baseline_search_settle_seconds_25hz_v1`
- result: `OK`
- tracking_source: `trajectory_controller_state_samples.csv`
- command_index: `0`
- samples: `140`
- hold_duration_s: `0.556`
- xy_error_min_m: `0.000296`
- xy_error_mean_m: `0.001953`
- xy_error_p95_m: `0.003327`
- xy_error_max_m: `0.003956`
- cartesian_error_p95_m: `0.004520`
- x_range_m: `0.006703`
- y_range_m: `0.005484`
- z_range_m: `0.006508`
- strict_xy_sample_count: `72`
- strict_xy_sample_fraction: `0.514286`
- strict_bin_count: `0`
- max_consecutive_strict_bins: `0`
- estimated_state_loop_hz: `25.0`
- largest_feedback_range_joint: `joint_6` `0.018370` rad

## Per Joint Hold Dynamics

| joint | target_rad | feedback_min_rad | feedback_max_rad | feedback_range_rad | mean_abs_error_rad | p95_abs_error_rad | final_error_rad |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| joint_1 | -0.502569 | -0.507165 | -0.499368 | 0.007797 | 0.001884 | 0.004020 | -0.000734 |
| joint_2 | -0.992068 | -0.994785 | -0.986523 | 0.008262 | 0.001869 | 0.004437 | -0.002219 |
| joint_3 | 2.340682 | 2.338314 | 2.344798 | 0.006484 | 0.001197 | 0.002937 | 0.000860 |
| joint_4 | -0.112781 | -0.120032 | -0.107084 | 0.012948 | 0.002374 | 0.005614 | -0.001486 |
| joint_5 | -1.349980 | -1.358126 | -1.343075 | 0.015050 | 0.002824 | 0.006145 | 0.005238 |
| joint_6 | 0.024802 | 0.015417 | 0.033787 | 0.018370 | 0.003923 | 0.008595 | -0.002662 |

Interpretation: this diagnostic uses passive trajectory observer data after the axis-align command duration has elapsed and before the next trajectory command. It does not alter controller behavior or safety gates.
