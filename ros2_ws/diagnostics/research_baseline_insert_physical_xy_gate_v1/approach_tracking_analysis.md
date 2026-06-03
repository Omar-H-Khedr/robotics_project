# Approach Tracking Analysis

- input_dir: `diagnostics/research_baseline_insert_physical_xy_gate_v1`
- tracking_source: `trajectory_controller_state_samples.csv`
- command_index: `1`
- command_receipt_stamp_s: `50.423`
- next_command_stamp_s: `62.602`
- approach_command_duration_s: `15.000`
- observed_approach_window_s: `12.179`
- post_command_hold_before_next_command_s: `0.000`
- samples: `3045`
- max_abs_position_error_rad: `0.010219`
- p95_max_abs_position_error_rad: `0.008407`
- mean_rms_position_error_rad: `0.002954`
- worst_joint_by_p95_error: `joint_6` `0.008392` rad
- worst_joint_by_final_error: `joint_6` `0.006702` rad

## Cartesian Peg Tip Error

- command_target_xyz_m: `0.520000, -0.200000, 0.830000`
- final_feedback_xyz_m: `0.518982, -0.200932, 0.840791`
- final_cartesian_error_xyz_m: `0.001018, 0.000932, -0.010791`
- final_cartesian_error_norm_m: `0.010879`
- missing_descent_m: `0.010791`

## Per Joint Error

| joint | max_abs_rad | p95_abs_rad | mean_abs_rad | final_sample_error_rad | target_minus_feedback_rad | target_rad | final_feedback_rad |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| joint_1 | 0.006018 | 0.004349 | 0.001955 | -0.000464 | -0.000275 | -0.499781 | -0.499507 |
| joint_2 | 0.006339 | 0.004522 | 0.001827 | -0.001068 | 0.019771 | -0.895372 | -0.915143 |
| joint_3 | 0.005728 | 0.003595 | 0.001494 | -0.000237 | -0.007182 | 2.310601 | 2.317783 |
| joint_4 | 0.008683 | 0.006019 | 0.002620 | -0.005429 | -0.005235 | -0.108936 | -0.103701 |
| joint_5 | 0.008663 | 0.005670 | 0.002639 | -0.001097 | 0.002350 | -1.329775 | -1.332125 |
| joint_6 | 0.010219 | 0.008392 | 0.003956 | 0.006702 | 0.006480 | 0.018322 | 0.011842 |

Interpretation: this is an offline diagnostic over the passive tracking observer CSVs. It does not alter controller behavior or task safety gates.
