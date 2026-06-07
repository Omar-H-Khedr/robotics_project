# Above-Hole Hold Analysis

- input_dir: `diagnostics/research_baseline_search_gain3000_settle_seconds_25hz_v1`
- state: `MOVING_TO_START`
- strict_xy_m: `0.002000`
- required_stable_ticks: `5`
- state_loop_hz: `25.0`
- moving_to_start_samples: `4000`
- strict_samples: `19`
- best_continuous_strict_samples: `5`
- best_continuous_strict_duration_s: `0.040000`
- estimated_state_loop_best_stable_ticks: `2`
- estimated_state_loop_gate_passed: `False`
- min_xy_error_m: `0.000766`
- min_xy_stamp_s: `42.560`
- final_moving_xy_error_m: `0.001017`
- mean_moving_xy_error_m: `0.228384`

Interpretation: this is an offline diagnostic over passive observer CSVs. It estimates whether the recorded above-hole hold would satisfy the task controller's strict 2 mm XY gate for five configured-cadence state-loop ticks. It does not publish commands or alter task safety gates.
