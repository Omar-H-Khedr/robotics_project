# Endpoint Hold Dynamics Analysis

- input_dir: `diagnostics/research_baseline_insert_cartesian_descent_v1`
- result: `OK`
- tracking_source: `trajectory_controller_state_samples.csv`
- command_index: `0`
- samples: `817`
- hold_duration_s: `3.265`
- xy_error_min_m: `0.000048`
- xy_error_mean_m: `0.002000`
- xy_error_p95_m: `0.003612`
- xy_error_max_m: `0.005131`
- cartesian_error_p95_m: `0.004662`
- x_range_m: `0.009250`
- y_range_m: `0.006951`
- z_range_m: `0.008658`
- strict_xy_sample_count: `437`
- strict_xy_sample_fraction: `0.534884`
- strict_bin_count: `0`
- max_consecutive_strict_bins: `0`
- largest_feedback_range_joint: `joint_6` `0.019869` rad

## Per Joint Hold Dynamics

| joint | target_rad | feedback_min_rad | feedback_max_rad | feedback_range_rad | mean_abs_error_rad | p95_abs_error_rad | final_error_rad |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| joint_1 | -0.502569 | -0.508741 | -0.496156 | 0.012585 | 0.001875 | 0.004038 | 0.002526 |
| joint_2 | -0.992068 | -0.994947 | -0.985437 | 0.009510 | 0.001735 | 0.004407 | 0.001163 |
| joint_3 | 2.340682 | 2.337082 | 2.346012 | 0.008930 | 0.001484 | 0.003360 | 0.001731 |
| joint_4 | -0.112781 | -0.121047 | -0.104175 | 0.016872 | 0.002564 | 0.006113 | -0.000760 |
| joint_5 | -1.349980 | -1.357819 | -1.342276 | 0.015543 | 0.002631 | 0.005682 | 0.000602 |
| joint_6 | 0.024802 | 0.014624 | 0.034493 | 0.019869 | 0.003937 | 0.008377 | 0.000605 |

Interpretation: this diagnostic uses passive trajectory observer data after the axis-align command duration has elapsed and before the next trajectory command. It does not alter controller behavior or safety gates.
