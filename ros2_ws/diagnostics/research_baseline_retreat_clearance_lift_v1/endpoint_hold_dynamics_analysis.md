# Endpoint Hold Dynamics Analysis

- input_dir: `diagnostics/research_baseline_retreat_clearance_lift_v1`
- result: `OK`
- tracking_source: `trajectory_controller_state_samples.csv`
- command_index: `0`
- samples: `1344`
- hold_duration_s: `5.372`
- xy_error_min_m: `0.000046`
- xy_error_mean_m: `0.002016`
- xy_error_p95_m: `0.003635`
- xy_error_max_m: `0.005267`
- cartesian_error_p95_m: `0.004662`
- x_range_m: `0.009464`
- y_range_m: `0.007415`
- z_range_m: `0.008451`
- strict_xy_sample_count: `685`
- strict_xy_sample_fraction: `0.509673`
- strict_bin_count: `0`
- max_consecutive_strict_bins: `0`
- largest_feedback_range_joint: `joint_6` `0.020450` rad

## Per Joint Hold Dynamics

| joint | target_rad | feedback_min_rad | feedback_max_rad | feedback_range_rad | mean_abs_error_rad | p95_abs_error_rad | final_error_rad |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| joint_1 | -0.502569 | -0.508836 | -0.496258 | 0.012578 | 0.001850 | 0.004028 | -0.002452 |
| joint_2 | -0.992068 | -0.995196 | -0.985006 | 0.010190 | 0.001945 | 0.004813 | 0.001909 |
| joint_3 | 2.340682 | 2.336378 | 2.346209 | 0.009831 | 0.001498 | 0.003437 | -0.000475 |
| joint_4 | -0.112781 | -0.121712 | -0.104647 | 0.017066 | 0.002569 | 0.005982 | 0.002462 |
| joint_5 | -1.349980 | -1.357282 | -1.341639 | 0.015643 | 0.002667 | 0.005800 | 0.001115 |
| joint_6 | 0.024802 | 0.014483 | 0.034934 | 0.020450 | 0.003943 | 0.008241 | -0.001680 |

Interpretation: this diagnostic uses passive trajectory observer data after the axis-align command duration has elapsed and before the next trajectory command. It does not alter controller behavior or safety gates.
