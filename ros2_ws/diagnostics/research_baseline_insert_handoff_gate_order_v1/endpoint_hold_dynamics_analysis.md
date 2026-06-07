# Endpoint Hold Dynamics Analysis

- input_dir: `diagnostics/research_baseline_insert_handoff_gate_order_v1`
- result: `OK`
- tracking_source: `trajectory_controller_state_samples.csv`
- command_index: `0`
- samples: `169`
- hold_duration_s: `0.672`
- xy_error_min_m: `0.000100`
- xy_error_mean_m: `0.001744`
- xy_error_p95_m: `0.003438`
- xy_error_max_m: `0.004717`
- cartesian_error_p95_m: `0.004249`
- x_range_m: `0.007803`
- y_range_m: `0.005776`
- z_range_m: `0.007150`
- strict_xy_sample_count: `108`
- strict_xy_sample_fraction: `0.639053`
- strict_bin_count: `0`
- max_consecutive_strict_bins: `0`
- estimated_state_loop_hz: `25.0`
- largest_feedback_range_joint: `joint_6` `0.019148` rad

## Per Joint Hold Dynamics

| joint | target_rad | feedback_min_rad | feedback_max_rad | feedback_range_rad | mean_abs_error_rad | p95_abs_error_rad | final_error_rad |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| joint_1 | -0.502569 | -0.508111 | -0.497792 | 0.010320 | 0.001552 | 0.004290 | 0.000038 |
| joint_2 | -0.992068 | -0.995326 | -0.986007 | 0.009319 | 0.001680 | 0.003817 | 0.002454 |
| joint_3 | 2.340682 | 2.337740 | 2.345168 | 0.007429 | 0.001402 | 0.003292 | -0.001936 |
| joint_4 | -0.112781 | -0.119879 | -0.104855 | 0.015024 | 0.002787 | 0.006851 | 0.000754 |
| joint_5 | -1.349980 | -1.356319 | -1.342793 | 0.013526 | 0.002730 | 0.006022 | -0.002082 |
| joint_6 | 0.024802 | 0.014741 | 0.033889 | 0.019148 | 0.003924 | 0.008282 | 0.000769 |

Interpretation: this diagnostic uses passive trajectory observer data after the axis-align command duration has elapsed and before the next trajectory command. It does not alter controller behavior or safety gates.
