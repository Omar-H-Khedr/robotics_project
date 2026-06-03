# XY Stability Analysis

- input_dir: `diagnostics/research_baseline_search_recenter_on_coarse_band_v1`
- sample_source: `wrench_state_samples.csv`
- total_valid_samples: `19199`
- state_loop_hz: `10.0`

| State | Samples | Min XY m | Mean XY m | P95 XY m | Final XY m | Z range m | Best 1mm ticks | Best 2mm ticks |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| ABORT | 2470 | 0.000114 | 0.111887 | 0.367311 | 0.396503 | 0.815710..1.025269 | 1 | 4 |
| APPROACH | 1190 | 0.000062 | 0.002287 | 0.004299 | 0.003853 | 0.839321..0.887747 | 2 | 4 |
| DONE | 5769 | 0.395957 | 0.409264 | 0.412284 | 0.408998 | 1.023935..1.042632 | 0 | 0 |
| MOVING_TO_START | 4920 | 0.000064 | 0.188614 | 0.406417 | 0.001807 | 0.858275..1.041843 | 2 | 6 |
| SEARCH | 4500 | 0.000044 | 0.002975 | 0.006085 | 0.003714 | 0.814882..0.844184 | 4 | 9 |
| UNKNOWN | 350 | 0.404504 | 0.409502 | 0.411777 | 0.407358 | 1.033562..1.041835 | 0 | 0 |

Interpretation: this is an offline passive-log diagnostic. The state-loop window estimate samples the observer stream at 10 Hz, matching the task controller cadence closely enough to decide whether a sustained gate is plausible before changing controller behavior.
