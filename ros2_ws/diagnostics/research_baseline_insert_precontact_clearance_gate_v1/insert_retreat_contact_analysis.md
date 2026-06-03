# Insert / Retreat Contact Analysis

- input_dir: `diagnostics/research_baseline_insert_precontact_clearance_gate_v1`
- tracking_source: `trajectory_controller_state_samples.csv`
- command_index: `2`
- command_receipt_stamp_s: `63.203`
- next_command_stamp_s: `63.718`
- insert_command_duration_s: `20.000`
- observed_window_s: `0.515`
- samples: `128`
- max_abs_position_error_rad: `0.009368`
- p95_max_abs_position_error_rad: `0.008059`
- mean_rms_position_error_rad: `0.002998`

## Cartesian Peg Tip

- command_target_xyz_m: `0.520004, -0.200001, 0.790004`
- final_feedback_xyz_m: `0.521952, -0.200499, 0.826544`
- final_cartesian_error_xyz_m: `-0.001948, 0.000498, -0.036540`
- final_cartesian_error_norm_m: `0.036595`
- missing_descent_to_target_m: `0.036540`
- hole_top_z_m: `0.810000`
- min_feedback_z_m: `0.825876`
- max_physical_depth_m: `0.000000`
- final_physical_depth_m: `0.000000`

## Contact By State

- contact_state_samples.csv unavailable or empty

## Wrench By State

| state | samples | max_abs_fz_n | max_force_norm_n | mean_xy_error_m |
| --- | ---: | ---: | ---: | ---: |
| ABORT | 2460 | 125.907529 | 200.446259 | 0.118591 |
| APPROACH | 1220 | 129.397658 | 204.882497 | 0.002113 |
| DONE | 2898 | 126.753893 | 182.176724 | 0.409188 |
| INSERT | 50 | 109.307240 | 181.134548 | 0.001924 |
| MOVING_TO_START | 4610 | 126.640378 | 205.051445 | 0.196909 |
| SEARCH | 230 | 123.084811 | 205.090642 | 0.002191 |
| UNKNOWN | 251 | 121.353317 | 181.708571 | 0.409631 |

Interpretation: this is an offline diagnostic over passive observer CSVs. It does not publish commands or change task safety gates.
