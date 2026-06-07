# SEARCH Gate Trace Analysis

- input: `diagnostics/research_baseline_clean_python_shutdown_recenter8_gain3000_damping10_25hz_v3/search_gate_trace.csv`
- physical_clearance_m: `0.001000`
- required_ticks: `8`
- rows: `1125`
- final_decision: `timeout_abort`
- final_xy_error_m: `0.001285`
- max_convergence_ticks_after: `6`
- passed_online_gate: `False`

| Decision | Rows |
| --- | ---: |
| post_settle_count | 2 |
| recenter_command | 4 |
| settling_count | 134 |
| settling_reset | 984 |
| timeout_abort | 1 |

| Window | Ticks | Start tick | End tick | Min XY m | Max XY m |
| --- | ---: | ---: | ---: | ---: | ---: |
| ready inside clearance | 6 | 38 | 43 | 0.000546 | 0.000909 |
| all trace inside clearance | 6 | 38 | 43 | 0.000546 | 0.000909 |

Interpretation: `max_convergence_ticks_after` is the online state-machine counter recorded by `admittance_insertion_node`. It is more authoritative than passive observer replay for deciding whether SEARCH was allowed to enter INSERT.
