# Endpoint Hold Dynamics Analysis

- input_dir: `diagnostics/research_baseline_approach_z_precondition_gate_v1`
- result: `OK`
- tracking_source: `trajectory_controller_state_samples.csv`
- command_index: `0`
- samples: `283`
- hold_duration_s: `1.128`
- xy_error_min_m: `0.000130`
- xy_error_mean_m: `0.002252`
- xy_error_p95_m: `0.004252`
- xy_error_max_m: `0.005295`
- cartesian_error_p95_m: `0.004761`
- x_range_m: `0.009183`
- y_range_m: `0.006769`
- z_range_m: `0.007608`
- strict_xy_sample_count: `126`
- strict_xy_sample_fraction: `0.445230`
- strict_bin_count: `0`
- max_consecutive_strict_bins: `0`
- largest_feedback_range_joint: `joint_6` `0.019993` rad

## Per Joint Hold Dynamics

| joint | target_rad | feedback_min_rad | feedback_max_rad | feedback_range_rad | mean_abs_error_rad | p95_abs_error_rad | final_error_rad |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| joint_1 | -0.502569 | -0.508047 | -0.497271 | 0.010776 | 0.002553 | 0.004921 | 0.000082 |
| joint_2 | -0.992068 | -0.994642 | -0.986437 | 0.008205 | 0.001322 | 0.003697 | 0.000263 |
| joint_3 | 2.340682 | 2.337489 | 2.344425 | 0.006936 | 0.001145 | 0.002761 | -0.001124 |
| joint_4 | -0.112781 | -0.120495 | -0.104016 | 0.016479 | 0.002768 | 0.006369 | 0.004056 |
| joint_5 | -1.349980 | -1.358032 | -1.342228 | 0.015805 | 0.002716 | 0.005851 | -0.000647 |
| joint_6 | 0.024802 | 0.014902 | 0.034895 | 0.019993 | 0.003990 | 0.008531 | 0.006837 |

Interpretation: this diagnostic uses passive trajectory observer data after the axis-align command duration has elapsed and before the next trajectory command. It does not alter controller behavior or safety gates.
