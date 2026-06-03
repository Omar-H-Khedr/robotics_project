# XY Stability Analysis

- input_dir: `diagnostics/research_baseline_search_centered_hold_v1`
- sample_source: `wrench_state_samples.csv`
- total_valid_samples: `20912`
- state_loop_hz: `10.0`

| State | Samples | Min XY m | Mean XY m | P95 XY m | Final XY m | Z range m | Best 1mm ticks | Best 2mm ticks |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| ABORT | 2460 | 0.000804 | 0.125661 | 0.370046 | 0.395072 | 0.835228..1.024642 | 0 | 1 |
| APPROACH | 1180 | 0.000082 | 0.002192 | 0.004238 | 0.001109 | 0.841635..0.887802 | 2 | 13 |
| DONE | 7682 | 0.395517 | 0.409259 | 0.412146 | 0.407009 | 1.023672..1.042588 | 0 | 0 |
| MOVING_TO_START | 4840 | 0.000063 | 0.187411 | 0.402237 | 0.001532 | 0.858784..1.042666 | 2 | 4 |
| SEARCH | 4500 | 0.000062 | 0.003157 | 0.005951 | 0.004772 | 0.837012..0.849746 | 2 | 6 |
| UNKNOWN | 250 | 0.406117 | 0.409551 | 0.411957 | 0.408282 | 1.034113..1.041759 | 0 | 0 |

Interpretation: this is an offline passive-log diagnostic. The state-loop window estimate samples the observer stream at 10 Hz, matching the task controller cadence closely enough to decide whether a sustained gate is plausible before changing controller behavior.
