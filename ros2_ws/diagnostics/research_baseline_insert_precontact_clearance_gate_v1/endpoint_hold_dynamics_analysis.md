# Endpoint Hold Dynamics Analysis

- input_dir: `diagnostics/research_baseline_insert_precontact_clearance_gate_v1`
- result: `OK`
- tracking_source: `trajectory_controller_state_samples.csv`
- command_index: `0`
- samples: `1572`
- hold_duration_s: `6.284`
- xy_error_min_m: `0.000092`
- xy_error_mean_m: `0.002126`
- xy_error_p95_m: `0.003887`
- xy_error_max_m: `0.005397`
- cartesian_error_p95_m: `0.004541`
- x_range_m: `0.009502`
- y_range_m: `0.007930`
- z_range_m: `0.007836`
- strict_xy_sample_count: `737`
- strict_xy_sample_fraction: `0.468830`
- strict_bin_count: `0`
- max_consecutive_strict_bins: `0`
- largest_feedback_range_joint: `joint_6` `0.019904` rad

## Per Joint Hold Dynamics

| joint | target_rad | feedback_min_rad | feedback_max_rad | feedback_range_rad | mean_abs_error_rad | p95_abs_error_rad | final_error_rad |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| joint_1 | -0.502569 | -0.508498 | -0.496549 | 0.011950 | 0.002152 | 0.004662 | -0.002193 |
| joint_2 | -0.992068 | -0.995378 | -0.985933 | 0.009444 | 0.001656 | 0.004195 | 0.002362 |
| joint_3 | 2.340682 | 2.336449 | 2.346721 | 0.010271 | 0.001514 | 0.003451 | -0.000068 |
| joint_4 | -0.112781 | -0.121325 | -0.104628 | 0.016697 | 0.002501 | 0.005807 | -0.001003 |
| joint_5 | -1.349980 | -1.357082 | -1.341605 | 0.015477 | 0.002628 | 0.005768 | -0.004764 |
| joint_6 | 0.024802 | 0.014732 | 0.034636 | 0.019904 | 0.003950 | 0.008288 | 0.003025 |

Interpretation: this diagnostic uses passive trajectory observer data after the axis-align command duration has elapsed and before the next trajectory command. It does not alter controller behavior or safety gates.
