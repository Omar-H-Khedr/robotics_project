# Endpoint Hold Dynamics Analysis

- input_dir: `diagnostics/research_baseline_insert_sim_time_completion_v4`
- result: `OK`
- tracking_source: `trajectory_controller_state_samples.csv`
- command_index: `0`
- samples: `766`
- hold_duration_s: `3.060`
- xy_error_min_m: `0.000083`
- xy_error_mean_m: `0.002109`
- xy_error_p95_m: `0.003778`
- xy_error_max_m: `0.005425`
- cartesian_error_p95_m: `0.004544`
- x_range_m: `0.009679`
- y_range_m: `0.007477`
- z_range_m: `0.007477`
- strict_xy_sample_count: `375`
- strict_xy_sample_fraction: `0.489556`
- strict_bin_count: `0`
- max_consecutive_strict_bins: `0`
- largest_feedback_range_joint: `joint_6` `0.019259` rad

## Per Joint Hold Dynamics

| joint | target_rad | feedback_min_rad | feedback_max_rad | feedback_range_rad | mean_abs_error_rad | p95_abs_error_rad | final_error_rad |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| joint_1 | -0.502569 | -0.508682 | -0.496473 | 0.012208 | 0.002077 | 0.004008 | -0.003145 |
| joint_2 | -0.992068 | -0.995515 | -0.985748 | 0.009767 | 0.001747 | 0.004044 | -0.002426 |
| joint_3 | 2.340682 | 2.336484 | 2.346248 | 0.009764 | 0.001583 | 0.003517 | -0.000477 |
| joint_4 | -0.112781 | -0.121709 | -0.104861 | 0.016848 | 0.002579 | 0.006229 | 0.002234 |
| joint_5 | -1.349980 | -1.358163 | -1.341607 | 0.016556 | 0.002679 | 0.005724 | -0.000871 |
| joint_6 | 0.024802 | 0.015186 | 0.034445 | 0.019259 | 0.003909 | 0.008163 | 0.004381 |

Interpretation: this diagnostic uses passive trajectory observer data after the axis-align command duration has elapsed and before the next trajectory command. It does not alter controller behavior or safety gates.
