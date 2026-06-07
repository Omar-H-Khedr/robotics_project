# Above-Hole Hold Analysis

- input_dir: `diagnostics/research_baseline_search_recenter8_settle9_gain3000_damping10_25hz_v1`
- state: `MOVING_TO_START`
- strict_xy_m: `0.002000`
- required_stable_ticks: `5`
- state_loop_hz: `25.0`
- moving_to_start_samples: `4005`
- strict_samples: `17`
- best_continuous_strict_samples: `7`
- best_continuous_strict_duration_s: `0.060000`
- estimated_state_loop_best_stable_ticks: `2`
- estimated_state_loop_gate_passed: `False`
- min_xy_error_m: `0.000236`
- min_xy_stamp_s: `42.750`
- final_moving_xy_error_m: `0.000257`
- mean_moving_xy_error_m: `0.228634`

Interpretation: this is an offline diagnostic over passive observer CSVs. It estimates whether the recorded above-hole hold would satisfy the task controller's strict 2 mm XY gate for five configured-cadence state-loop ticks. It does not publish commands or alter task safety gates.
