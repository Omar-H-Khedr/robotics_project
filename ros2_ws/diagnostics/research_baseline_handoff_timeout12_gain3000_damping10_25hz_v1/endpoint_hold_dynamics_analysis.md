# Endpoint Hold Dynamics Analysis

- input_dir: `diagnostics/research_baseline_handoff_timeout12_gain3000_damping10_25hz_v1`
- result: `OK`
- tracking_source: `trajectory_controller_state_samples.csv`
- command_index: `0`
- samples: `49`
- hold_duration_s: `0.192`
- xy_error_min_m: `0.000111`
- xy_error_mean_m: `0.001075`
- xy_error_p95_m: `0.002108`
- xy_error_max_m: `0.002245`
- cartesian_error_p95_m: `0.002772`
- x_range_m: `0.004345`
- y_range_m: `0.003121`
- z_range_m: `0.004390`
- strict_xy_sample_count: `45`
- strict_xy_sample_fraction: `0.918367`
- strict_bin_count: `2`
- max_consecutive_strict_bins: `2`
- estimated_state_loop_hz: `25.0`
- largest_feedback_range_joint: `joint_6` `0.015774` rad

## Per Joint Hold Dynamics

| joint | target_rad | feedback_min_rad | feedback_max_rad | feedback_range_rad | mean_abs_error_rad | p95_abs_error_rad | final_error_rad |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| joint_1 | -0.502569 | -0.504255 | -0.501530 | 0.002725 | 0.000754 | 0.001545 | 0.001621 |
| joint_2 | -0.992068 | -0.993618 | -0.988635 | 0.004983 | 0.001361 | 0.003063 | -0.001611 |
| joint_3 | 2.340682 | 2.339558 | 2.342437 | 0.002879 | 0.000702 | 0.001357 | -0.000807 |
| joint_4 | -0.112781 | -0.116696 | -0.107491 | 0.009205 | 0.002471 | 0.004769 | 0.001005 |
| joint_5 | -1.349980 | -1.354929 | -1.345235 | 0.009695 | 0.002215 | 0.004504 | 0.001997 |
| joint_6 | 0.024802 | 0.017402 | 0.033176 | 0.015774 | 0.003746 | 0.007401 | -0.001145 |

Interpretation: this diagnostic uses passive trajectory observer data after the axis-align command duration has elapsed and before the next trajectory command. It does not alter controller behavior or safety gates.
