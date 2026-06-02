# Approach Tracking Analysis

- input_dir: `diagnostics/research_baseline_search_fail_closed_v2`
- command_receipt_stamp_s: `49.864`
- next_command_stamp_s: `93.820`
- approach_command_duration_s: `15.000`
- observed_approach_window_s: `43.956`
- post_command_hold_before_next_command_s: `28.952`
- samples: `10987`
- max_abs_position_error_rad: `0.109348`
- p95_max_abs_position_error_rad: `0.108733`
- mean_rms_position_error_rad: `0.036725`
- worst_joint_by_p95_error: `joint_2` `0.108733` rad
- worst_joint_by_final_error: `joint_2` `0.107360` rad

## Cartesian Peg Tip Error

- command_target_xyz_m: `0.520000, -0.200000, 0.830000`
- final_feedback_xyz_m: `0.512892, -0.183042, 0.899840`
- final_cartesian_error_xyz_m: `0.007108, -0.016958, -0.069840`
- final_cartesian_error_norm_m: `0.072220`
- missing_descent_m: `-0.069840`

## Per Joint Error

| joint | max_abs_rad | p95_abs_rad | mean_abs_rad | final_error_rad | target_rad | final_feedback_rad |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| joint_1 | 0.005131 | 0.001934 | 0.000845 | -0.000638 | -0.527417 | -0.526779 |
| joint_2 | 0.109348 | 0.108733 | 0.087150 | 0.107360 | -0.555841 | -0.663200 |
| joint_3 | 0.008244 | 0.002575 | 0.001053 | 0.001397 | 2.240152 | 2.238755 |
| joint_4 | 0.031155 | 0.016518 | 0.007276 | 0.011167 | -0.198544 | -0.209711 |
| joint_5 | 0.036416 | 0.022012 | 0.009287 | 0.013594 | -1.638703 | -1.652297 |
| joint_6 | 0.023028 | 0.014348 | 0.006233 | 0.007967 | -0.005358 | -0.013325 |

Interpretation: this is an offline diagnostic over the passive tracking observer CSVs. It does not alter controller behavior or task safety gates.
