# Withdrawal Contact Timing Analysis

- input_dir: `diagnostics/research_baseline_insert_cartesian_descent_v1`
- tracking_source: `trajectory_controller_state_samples.csv`
- insert_command_index: `2`
- positive_contact_samples: `0`

## Command Windows

| index | label | receipt_stamp_s | next_command_stamp_s | duration_s | point_count | target_xyz_m |
| ---: | --- | ---: | ---: | ---: | ---: | --- |
| 0 | MOVING_TO_START | 2.835 | 46.102 | 40.000 | 20 | `0.520000, -0.200000, 0.885001` |
| 1 | APPROACH | 46.102 | 61.711 | 15.000 | 8 | `0.520000, -0.200000, 0.830000` |
| 2 | INSERT | 61.711 | 62.008 | 20.000 | 6 | `0.520000, -0.200000, 0.790000` |
| 3 | RETREAT_1 | 62.008 | none | 25.000 | 16 | `0.800000, 0.099056, 1.039872` |

## Positive Contact By Active Command

| command | samples | first_stamp_s | last_stamp_s | max_force_n | max_depth_m | max_xy_error_m | z_range_m | top_collision_pair |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- | --- |

## Highest Force Events

| stamp_s | command | source | force_n | depth_m | xy_error_m | peg_tip_xyz_m | collision_pairs |
| ---: | --- | --- | ---: | ---: | ---: | --- | --- |

Interpretation: this is an offline diagnostic. It uses recorded command, contact, and tracking CSVs only; it does not publish robot commands or change safety gates.
