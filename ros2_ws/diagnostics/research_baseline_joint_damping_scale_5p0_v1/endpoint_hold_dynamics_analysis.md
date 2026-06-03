# Endpoint Hold Dynamics Analysis

- input_dir: `diagnostics/research_baseline_joint_damping_scale_5p0_v1`
- result: `OK`
- tracking_source: `trajectory_controller_state_samples.csv`
- command_index: `0`
- samples: `278`
- hold_duration_s: `1.107`
- xy_error_min_m: `0.000088`
- xy_error_mean_m: `0.002058`
- xy_error_p95_m: `0.003559`
- xy_error_max_m: `0.004176`
- cartesian_error_p95_m: `0.004768`
- x_range_m: `0.008135`
- y_range_m: `0.007048`
- z_range_m: `0.009094`
- strict_xy_sample_count: `140`
- strict_xy_sample_fraction: `0.503597`
- strict_bin_count: `0`
- max_consecutive_strict_bins: `0`
- largest_feedback_range_joint: `joint_6` `0.018783` rad

## Per Joint Hold Dynamics

| joint | target_rad | feedback_min_rad | feedback_max_rad | feedback_range_rad | mean_abs_error_rad | p95_abs_error_rad | final_error_rad |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| joint_1 | -0.502569 | -0.507180 | -0.498339 | 0.008841 | 0.002101 | 0.003936 | 0.003551 |
| joint_2 | -0.992068 | -0.996060 | -0.984734 | 0.011326 | 0.001977 | 0.005267 | -0.001891 |
| joint_3 | 2.340682 | 2.336862 | 2.344350 | 0.007487 | 0.001299 | 0.003034 | -0.001360 |
| joint_4 | -0.112781 | -0.119893 | -0.105377 | 0.014516 | 0.002435 | 0.005491 | -0.001544 |
| joint_5 | -1.349980 | -1.357153 | -1.342319 | 0.014834 | 0.002692 | 0.005810 | 0.000712 |
| joint_6 | 0.024802 | 0.014954 | 0.033737 | 0.018783 | 0.003895 | 0.008185 | 0.005355 |

Interpretation: this diagnostic uses passive trajectory observer data after the axis-align command duration has elapsed and before the next trajectory command. It does not alter controller behavior or safety gates.
