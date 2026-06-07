# SEARCH Gate Trace Analysis

- input: `diagnostics/research_baseline_velocity_state_500hz_repeat_v1/trial_02_tracking/search_gate_trace.csv`
- physical_clearance_m: `0.001000`
- required_ticks: `8`
- rows: `0`
- final_decision: `None`
- final_xy_error_m: `none`
- max_convergence_ticks_after: `0`
- passed_online_gate: `False`

| Decision | Rows |
| --- | ---: |

| Window | Ticks | Start tick | End tick | Min XY m | Max XY m |
| --- | ---: | ---: | ---: | ---: | ---: |
| ready inside clearance | 0 | None | None | none | none |
| all trace inside clearance | 0 | None | None | none | none |

Interpretation: `max_convergence_ticks_after` is the online state-machine counter recorded by `admittance_insertion_node`. It is more authoritative than passive observer replay for deciding whether SEARCH was allowed to enter INSERT.
