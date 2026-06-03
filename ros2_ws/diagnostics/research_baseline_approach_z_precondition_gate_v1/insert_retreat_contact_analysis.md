# Insert / Retreat Contact Analysis

- input_dir: `diagnostics/research_baseline_approach_z_precondition_gate_v1`
- tracking_source: `trajectory_controller_state_samples.csv`
- command_index: `2`
- command_receipt_stamp_s: `55.500`
- next_command_stamp_s: `65.826`
- insert_command_duration_s: `20.000`
- observed_window_s: `10.326`
- samples: `2578`
- max_abs_position_error_rad: `0.010354`
- p95_max_abs_position_error_rad: `0.008407`
- mean_rms_position_error_rad: `0.002915`

## Cartesian Peg Tip

- command_target_xyz_m: `0.520004, -0.200001, 0.790008`
- final_feedback_xyz_m: `0.518227, -0.201338, 0.813985`
- final_cartesian_error_xyz_m: `0.001777, 0.001337, -0.023977`
- final_cartesian_error_norm_m: `0.024080`
- missing_descent_to_target_m: `0.023977`
- hole_top_z_m: `0.810000`
- min_feedback_z_m: `0.811899`
- max_physical_depth_m: `0.000000`
- final_physical_depth_m: `0.000000`

## Contact By State

| state | first_stamp_s | samples | positive_samples | max_contact_force_n | top_collision_pair |
| --- | ---: | ---: | ---: | ---: | --- |
| RETREAT | 67.281 | 7776 | 7776 | 1970.434828 | `lbr_iisy6_r1300::ft_sensor_link::ft_sensor_link_fixed_joint_lump__grasped_peg_collision_2 <-> target_plate::plate_link::target_plate_collision` |

## Wrench By State

| state | samples | max_abs_fz_n | max_force_norm_n | mean_xy_error_m |
| --- | ---: | ---: | ---: | ---: |
| APPROACH | 1178 | 127.543219 | 208.030911 | 0.002042 |
| IDLE | 81 | 116.365116 | 175.929637 | 0.409227 |
| INSERT | 1035 | 125.636943 | 209.244996 | 0.002775 |
| MOVING_TO_START | 4116 | 128.729012 | 207.792381 | 0.221566 |
| RETREAT | 691 | 594.283889 | 949.257744 | 0.018550 |
| SEARCH | 5 | 94.361819 | 105.167668 | 0.002661 |
| UNKNOWN | 166 | 100.296453 | 168.150419 | 0.409489 |

Interpretation: this is an offline diagnostic over passive observer CSVs. It does not publish commands or change task safety gates.
