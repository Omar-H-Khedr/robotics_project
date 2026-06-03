# Endpoint Hold Dynamics Analysis

- input_dir: `diagnostics/research_baseline_insert_physical_xy_gate_v1`
- result: `OK`
- tracking_source: `trajectory_controller_state_samples.csv`
- command_index: `0`
- samples: `1929`
- hold_duration_s: `7.712`
- xy_error_min_m: `0.000056`
- xy_error_mean_m: `0.001923`
- xy_error_p95_m: `0.003681`
- xy_error_max_m: `0.005041`
- cartesian_error_p95_m: `0.004452`
- x_range_m: `0.009266`
- y_range_m: `0.006542`
- z_range_m: `0.008530`
- strict_xy_sample_count: `1084`
- strict_xy_sample_fraction: `0.561949`
- strict_bin_count: `0`
- max_consecutive_strict_bins: `0`
- largest_feedback_range_joint: `joint_6` `0.020909` rad

## Per Joint Hold Dynamics

| joint | target_rad | feedback_min_rad | feedback_max_rad | feedback_range_rad | mean_abs_error_rad | p95_abs_error_rad | final_error_rad |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| joint_1 | -0.502569 | -0.508728 | -0.497063 | 0.011665 | 0.001848 | 0.004183 | -0.004502 |
| joint_2 | -0.992068 | -0.995527 | -0.983794 | 0.011734 | 0.001854 | 0.004572 | -0.000186 |
| joint_3 | 2.340682 | 2.336732 | 2.346062 | 0.009330 | 0.001332 | 0.003201 | -0.000833 |
| joint_4 | -0.112781 | -0.120794 | -0.104790 | 0.016004 | 0.002578 | 0.005899 | -0.000752 |
| joint_5 | -1.349980 | -1.358348 | -1.341843 | 0.016505 | 0.002630 | 0.005748 | -0.002518 |
| joint_6 | 0.024802 | 0.014447 | 0.035356 | 0.020909 | 0.003950 | 0.008367 | 0.007031 |

Interpretation: this diagnostic uses passive trajectory observer data after the axis-align command duration has elapsed and before the next trajectory command. It does not alter controller behavior or safety gates.
