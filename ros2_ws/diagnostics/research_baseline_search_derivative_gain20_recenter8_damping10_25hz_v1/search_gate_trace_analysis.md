# SEARCH Gate Trace Analysis

- input: `diagnostics/research_baseline_search_derivative_gain20_recenter8_damping10_25hz_v1/search_gate_trace.csv`
- physical_clearance_m: `0.001000`
- required_ticks: `8`
- rows: `1125`
- final_decision: `timeout_abort`
- final_xy_error_m: `0.000623`
- max_convergence_ticks_after: `4`
- passed_online_gate: `False`

| Decision | Rows |
| --- | ---: |
| post_settle_count | 2 |
| recenter_command | 4 |
| settling_count | 119 |
| settling_reset | 999 |
| timeout_abort | 1 |

| Window | Ticks | Start tick | End tick | Min XY m | Max XY m |
| --- | ---: | ---: | ---: | ---: | ---: |
| ready inside clearance | 4 | 113 | 116 | 0.000178 | 0.000926 |
| all trace inside clearance | 5 | 874 | 878 | 0.000602 | 0.000948 |

Interpretation: `max_convergence_ticks_after` is the online state-machine counter recorded by `admittance_insertion_node`. It is more authoritative than passive observer replay for deciding whether SEARCH was allowed to enter INSERT.
