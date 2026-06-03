# XY Stability Analysis

- input_dir: `diagnostics/research_baseline_search_derivative_gain_v2`
- sample_source: `wrench_state_samples.csv`
- total_valid_samples: `20212`
- state_loop_hz: `10.0`

| State | Samples | Min XY m | Mean XY m | P95 XY m | Final XY m | Z range m | Best 1mm ticks | Best 2mm ticks |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| ABORT | 2470 | 0.000054 | 0.112289 | 0.367802 | 0.400541 | 0.812910..1.026730 | 2 | 3 |
| APPROACH | 1180 | 0.000079 | 0.002202 | 0.004111 | 0.001470 | 0.840126..0.886898 | 3 | 6 |
| DONE | 7042 | 0.397256 | 0.409270 | 0.412166 | 0.407928 | 1.024655..1.042997 | 0 | 0 |
| MOVING_TO_START | 4740 | 0.000013 | 0.190308 | 0.402645 | 0.002309 | 0.858503..1.041894 | 2 | 6 |
| SEARCH | 4500 | 0.000005 | 0.002317 | 0.004263 | 0.002667 | 0.813727..0.845665 | 3 | 7 |
| UNKNOWN | 280 | 0.404996 | 0.409310 | 0.411412 | 0.406221 | 1.029323..1.042371 | 0 | 0 |

Interpretation: this is an offline passive-log diagnostic. The state-loop window estimate samples the observer stream at 10 Hz, matching the task controller cadence closely enough to decide whether a sustained gate is plausible before changing controller behavior.
