# Endpoint Hold Dynamics Analysis

- input_dir: `diagnostics/research_baseline_search_gain3000_settle_seconds_25hz_v1`
- result: `OK`
- tracking_source: `trajectory_controller_state_samples.csv`
- command_index: `0`
- samples: `23`
- hold_duration_s: `0.089`
- xy_error_min_m: `0.000324`
- xy_error_mean_m: `0.001609`
- xy_error_p95_m: `0.002809`
- xy_error_max_m: `0.003002`
- cartesian_error_p95_m: `0.003106`
- x_range_m: `0.004264`
- y_range_m: `0.004924`
- z_range_m: `0.002634`
- strict_xy_sample_count: `15`
- strict_xy_sample_fraction: `0.652174`
- strict_bin_count: `0`
- max_consecutive_strict_bins: `0`
- estimated_state_loop_hz: `25.0`
- largest_feedback_range_joint: `joint_4` `0.014353` rad

## Per Joint Hold Dynamics

| joint | target_rad | feedback_min_rad | feedback_max_rad | feedback_range_rad | mean_abs_error_rad | p95_abs_error_rad | final_error_rad |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| joint_1 | -0.502569 | -0.505576 | -0.498314 | 0.007262 | 0.001708 | 0.003880 | -0.002946 |
| joint_2 | -0.992068 | -0.992952 | -0.989800 | 0.003153 | 0.001089 | 0.002253 | -0.001266 |
| joint_3 | 2.340682 | 2.339281 | 2.343675 | 0.004394 | 0.001168 | 0.002860 | 0.001218 |
| joint_4 | -0.112781 | -0.118153 | -0.103800 | 0.014353 | 0.003024 | 0.008360 | 0.001668 |
| joint_5 | -1.349980 | -1.355220 | -1.347131 | 0.008089 | 0.002197 | 0.004133 | 0.005240 |
| joint_6 | 0.024802 | 0.017387 | 0.028852 | 0.011466 | 0.003653 | 0.007389 | -0.003436 |

Interpretation: this diagnostic uses passive trajectory observer data after the axis-align command duration has elapsed and before the next trajectory command. It does not alter controller behavior or safety gates.
