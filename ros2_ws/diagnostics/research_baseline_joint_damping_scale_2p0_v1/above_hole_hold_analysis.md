# Above-Hole Hold Analysis

- input_dir: `diagnostics/research_baseline_joint_damping_scale_2p0_v1`
- state: `MOVING_TO_START`
- strict_xy_m: `0.002000`
- required_stable_ticks: `5`
- state_loop_hz: `10.0`
- moving_to_start_samples: `6234`
- strict_samples: `215`
- best_continuous_strict_samples: `3`
- best_continuous_strict_duration_s: `0.020000`
- estimated_state_loop_best_stable_ticks: `2`
- estimated_state_loop_gate_passed: `False`
- min_xy_error_m: `0.000153`
- min_xy_stamp_s: `43.360`
- final_moving_xy_error_m: `0.006362`
- mean_moving_xy_error_m: `0.148320`

Interpretation: this is an offline diagnostic over passive observer CSVs. It estimates whether the recorded above-hole hold would satisfy the task controller's strict 2 mm XY gate for five 10 Hz state-loop ticks. It does not publish commands or alter task safety gates.
