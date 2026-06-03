# XY Stability Analysis

- input_dir: `diagnostics/research_baseline_search_derivative_gain_v3`
- sample_source: `wrench_state_samples.csv`
- total_valid_samples: `20265`
- state_loop_hz: `10.0`

| State | Samples | Min XY m | Mean XY m | P95 XY m | Final XY m | Z range m | Best 1mm ticks | Best 2mm ticks |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| ABORT | 2470 | 0.000242 | 0.112434 | 0.367009 | 0.397579 | 0.819121..1.025410 | 2 | 4 |
| APPROACH | 1210 | 0.000019 | 0.002380 | 0.004809 | 0.001821 | 0.840586..0.888643 | 2 | 4 |
| DONE | 7845 | 0.397873 | 0.409243 | 0.412184 | 0.407414 | 1.022450..1.042651 | 0 | 0 |
| IDLE | 62 | 0.405786 | 0.409302 | 0.411765 | 0.411804 | 1.033389..1.041498 | 0 | 0 |
| MOVING_TO_START | 4040 | 0.000422 | 0.228276 | 0.409122 | 0.001920 | 0.858039..1.040911 | 0 | 1 |
| SEARCH | 4500 | 0.000043 | 0.002393 | 0.004621 | 0.002974 | 0.817526..0.843894 | 3 | 10 |
| UNKNOWN | 138 | 0.406939 | 0.409638 | 0.410751 | 0.409835 | 1.033687..1.041937 | 0 | 0 |

Interpretation: this is an offline passive-log diagnostic. The state-loop window estimate samples the observer stream at 10 Hz, matching the task controller cadence closely enough to decide whether a sustained gate is plausible before changing controller behavior.
