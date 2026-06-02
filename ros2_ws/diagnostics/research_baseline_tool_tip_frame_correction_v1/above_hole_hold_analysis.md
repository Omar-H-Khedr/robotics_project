# Above-Hole Hold Analysis

- input_dir: `diagnostics/research_baseline_tool_tip_frame_correction_v1`
- state: `MOVING_TO_START`
- strict_xy_m: `0.002000`
- required_stable_ticks: `5`
- state_loop_hz: `10.0`
- moving_to_start_samples: `6138`
- strict_samples: `52`
- best_continuous_strict_samples: `2`
- best_continuous_strict_duration_s: `0.010000`
- estimated_state_loop_best_stable_ticks: `1`
- estimated_state_loop_gate_passed: `False`
- min_xy_error_m: `0.000037`
- min_xy_stamp_s: `59.230`
- final_moving_xy_error_m: `0.013978`
- mean_moving_xy_error_m: `0.149465`

Interpretation: this is an offline diagnostic over passive observer CSVs. It estimates whether the recorded above-hole hold would satisfy the task controller's strict 2 mm XY gate for five 10 Hz state-loop ticks. It does not publish commands or alter task safety gates.
