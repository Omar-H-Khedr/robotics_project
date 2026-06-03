# Endpoint Hold Dynamics Analysis

- input_dir: `diagnostics/research_baseline_controller_state_tracking_v2`
- result: `OK`
- tracking_source: `trajectory_controller_state_samples.csv`
- command_index: `0`
- samples: `5930`
- hold_duration_s: `23.716`
- xy_error_min_m: `0.000218`
- xy_error_mean_m: `0.009221`
- xy_error_p95_m: `0.015782`
- xy_error_max_m: `0.023077`
- cartesian_error_p95_m: `0.019016`
- x_range_m: `0.041273`
- y_range_m: `0.035843`
- z_range_m: `0.033742`
- strict_xy_sample_count: `140`
- strict_xy_sample_fraction: `0.023609`
- strict_bin_count: `0`
- max_consecutive_strict_bins: `0`
- largest_feedback_range_joint: `joint_1` `0.057121` rad

## Per Joint Hold Dynamics

| joint | target_rad | feedback_min_rad | feedback_max_rad | feedback_range_rad | mean_abs_error_rad | p95_abs_error_rad | final_error_rad |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| joint_1 | -0.502569 | -0.530103 | -0.472983 | 0.057121 | 0.009781 | 0.019452 | 0.010885 |
| joint_2 | -0.992068 | -1.004335 | -0.959995 | 0.044340 | 0.007394 | 0.017485 | 0.003953 |
| joint_3 | 2.340682 | 2.319723 | 2.367417 | 0.047693 | 0.007185 | 0.016488 | 0.004720 |
| joint_4 | -0.112781 | -0.140308 | -0.083773 | 0.056535 | 0.008001 | 0.020231 | -0.010125 |
| joint_5 | -1.349980 | -1.373775 | -1.326539 | 0.047236 | 0.006899 | 0.015965 | 0.012237 |
| joint_6 | 0.024802 | 0.007809 | 0.042330 | 0.034521 | 0.006001 | 0.013789 | -0.000046 |

Interpretation: this diagnostic uses passive trajectory observer data after the axis-align command duration has elapsed and before the next trajectory command. It does not alter controller behavior or safety gates.
