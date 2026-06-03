# Insert / Retreat Contact Analysis

- input_dir: `diagnostics/research_baseline_insert_cartesian_descent_v1`
- tracking_source: `trajectory_controller_state_samples.csv`
- command_index: `2`
- command_receipt_stamp_s: `61.711`
- next_command_stamp_s: `62.008`
- insert_command_duration_s: `20.000`
- observed_window_s: `0.297`
- samples: `75`
- max_abs_position_error_rad: `0.009622`
- p95_max_abs_position_error_rad: `0.007322`
- mean_rms_position_error_rad: `0.002617`

## Cartesian Peg Tip

- command_target_xyz_m: `0.520000, -0.200000, 0.790000`
- final_feedback_xyz_m: `0.522436, -0.201105, 0.828234`
- final_cartesian_error_xyz_m: `-0.002436, 0.001105, -0.038234`
- final_cartesian_error_norm_m: `0.038327`
- missing_descent_to_target_m: `0.038234`
- hole_top_z_m: `0.810000`
- min_feedback_z_m: `0.827124`
- max_physical_depth_m: `0.000000`
- final_physical_depth_m: `0.000000`

## Contact By State

- contact_state_samples.csv unavailable or empty

## Wrench By State

| state | samples | max_abs_fz_n | max_force_norm_n | mean_xy_error_m |
| --- | ---: | ---: | ---: | ---: |
| ABORT | 2470 | 130.547196 | 205.964963 | 0.119145 |
| APPROACH | 1170 | 130.638117 | 210.620984 | 0.002107 |
| DONE | 3692 | 128.391794 | 187.167429 | 0.409218 |
| INSERT | 30 | 110.301512 | 175.470887 | 0.001597 |
| MOVING_TO_START | 4296 | 129.565946 | 199.660665 | 0.211179 |
| SEARCH | 390 | 125.325476 | 201.486693 | 0.002039 |
| UNKNOWN | 305 | 111.241309 | 177.865051 | 0.409622 |

Interpretation: this is an offline diagnostic over passive observer CSVs. It does not publish commands or change task safety gates.
