# XY Stability Analysis

- input_dir: `diagnostics/research_baseline_insert_handoff_gate_order_v1`
- sample_source: `wrench_state_samples.csv`
- total_valid_samples: `8914`
- state_loop_hz: `25.0`

| State | Samples | Min XY m | Mean XY m | P95 XY m | Final XY m | Z range m | Best 1mm ticks | Best 2mm ticks |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| APPROACH | 1168 | 0.000041 | 0.002127 | 0.003923 | 0.001891 | 0.840813..0.888313 | 4 | 8 |
| MOVING_TO_START | 4063 | 0.000100 | 0.224278 | 0.406909 | 0.001810 | 0.858419..1.041879 | 3 | 6 |
| SEARCH | 3403 | 0.000069 | 0.002296 | 0.004227 | 0.001886 | 0.815629..0.845068 | 3 | 7 |
| UNKNOWN | 280 | 0.404730 | 0.409530 | 0.411950 | 0.407111 | 1.033027..1.041854 | 0 | 0 |

Interpretation: this is an offline passive-log diagnostic. The state-loop window estimate samples the observer stream at the task controller cadence recorded in trial_outcome.json when available, or the explicit --state-loop-hz override. This keeps sustained-clearance tick evidence honest when running non-default cadence diagnostics.
