# Endpoint Hold Dynamics Analysis

- input_dir: `diagnostics/research_baseline_insert_sideload_abort_v1`
- result: `OK`
- tracking_source: `trajectory_controller_state_samples.csv`
- command_index: `0`
- samples: `74`
- hold_duration_s: `0.292`
- xy_error_min_m: `0.000248`
- xy_error_mean_m: `0.001826`
- xy_error_p95_m: `0.003348`
- xy_error_max_m: `0.004151`
- cartesian_error_p95_m: `0.004334`
- x_range_m: `0.006360`
- y_range_m: `0.005812`
- z_range_m: `0.006641`
- strict_xy_sample_count: `42`
- strict_xy_sample_fraction: `0.567568`
- strict_bin_count: `0`
- max_consecutive_strict_bins: `0`
- largest_feedback_range_joint: `joint_6` `0.019548` rad

## Per Joint Hold Dynamics

| joint | target_rad | feedback_min_rad | feedback_max_rad | feedback_range_rad | mean_abs_error_rad | p95_abs_error_rad | final_error_rad |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| joint_1 | -0.502569 | -0.506896 | -0.497222 | 0.009674 | 0.001605 | 0.004160 | -0.005347 |
| joint_2 | -0.992068 | -0.995324 | -0.986875 | 0.008449 | 0.002356 | 0.004642 | 0.000192 |
| joint_3 | 2.340682 | 2.337804 | 2.344587 | 0.006783 | 0.001450 | 0.003387 | 0.001150 |
| joint_4 | -0.112781 | -0.118776 | -0.108093 | 0.010683 | 0.002463 | 0.005237 | 0.000449 |
| joint_5 | -1.349980 | -1.356667 | -1.342299 | 0.014369 | 0.002758 | 0.005828 | 0.003141 |
| joint_6 | 0.024802 | 0.015258 | 0.034807 | 0.019548 | 0.003975 | 0.008204 | 0.004577 |

Interpretation: this diagnostic uses passive trajectory observer data after the axis-align command duration has elapsed and before the next trajectory command. It does not alter controller behavior or safety gates.
