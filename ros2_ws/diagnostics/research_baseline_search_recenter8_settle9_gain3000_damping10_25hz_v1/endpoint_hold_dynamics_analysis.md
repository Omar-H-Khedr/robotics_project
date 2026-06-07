# Endpoint Hold Dynamics Analysis

- input_dir: `diagnostics/research_baseline_search_recenter8_settle9_gain3000_damping10_25hz_v1`
- result: `OK`
- tracking_source: `trajectory_controller_state_samples.csv`
- command_index: `0`
- samples: `25`
- hold_duration_s: `0.097`
- xy_error_min_m: `0.000257`
- xy_error_mean_m: `0.000958`
- xy_error_p95_m: `0.001621`
- xy_error_max_m: `0.001749`
- cartesian_error_p95_m: `0.002095`
- x_range_m: `0.002544`
- y_range_m: `0.002947`
- z_range_m: `0.003341`
- strict_xy_sample_count: `25`
- strict_xy_sample_fraction: `1.000000`
- strict_bin_count: `3`
- max_consecutive_strict_bins: `3`
- estimated_state_loop_hz: `25.0`
- largest_feedback_range_joint: `joint_5` `0.011116` rad

## Per Joint Hold Dynamics

| joint | target_rad | feedback_min_rad | feedback_max_rad | feedback_range_rad | mean_abs_error_rad | p95_abs_error_rad | final_error_rad |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| joint_1 | -0.502569 | -0.503942 | -0.501281 | 0.002661 | 0.000901 | 0.001288 | -0.000292 |
| joint_2 | -0.992068 | -0.993221 | -0.989311 | 0.003910 | 0.000839 | 0.002624 | -0.000243 |
| joint_3 | 2.340682 | 2.339016 | 2.343144 | 0.004128 | 0.000862 | 0.002292 | 0.000706 |
| joint_4 | -0.112781 | -0.117552 | -0.108318 | 0.009234 | 0.001704 | 0.004463 | 0.004771 |
| joint_5 | -1.349980 | -1.355322 | -1.344206 | 0.011116 | 0.002537 | 0.005342 | 0.004976 |
| joint_6 | 0.024802 | 0.018321 | 0.029246 | 0.010925 | 0.003587 | 0.006156 | -0.004443 |

Interpretation: this diagnostic uses passive trajectory observer data after the axis-align command duration has elapsed and before the next trajectory command. It does not alter controller behavior or safety gates.
