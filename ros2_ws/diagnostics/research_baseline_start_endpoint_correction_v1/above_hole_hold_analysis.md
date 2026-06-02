# Above-Hole Hold Analysis

- input_dir: `diagnostics/research_baseline_start_endpoint_correction_v1`
- state: `MOVING_TO_START`
- strict_xy_m: `0.002000`
- required_stable_ticks: `5`
- state_loop_hz: `10.0`
- moving_to_start_samples: `6147`
- strict_samples: `58`
- best_continuous_strict_samples: `2`
- best_continuous_strict_duration_s: `0.008000`
- estimated_state_loop_best_stable_ticks: `1`
- estimated_state_loop_gate_passed: `False`
- min_xy_error_m: `0.000173`
- min_xy_stamp_s: `42.451`
- final_moving_xy_error_m: `0.003925`
- mean_moving_xy_error_m: `0.150799`

Interpretation: this is an offline diagnostic over passive observer CSVs. It estimates whether the recorded above-hole hold would satisfy the task controller's strict 2 mm XY gate for five 10 Hz state-loop ticks. It does not publish commands or alter task safety gates.
