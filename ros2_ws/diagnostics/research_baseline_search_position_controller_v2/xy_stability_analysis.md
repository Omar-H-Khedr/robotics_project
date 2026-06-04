# XY Stability Analysis

- input_dir: `diagnostics/research_baseline_search_position_controller_v2`
- sample_source: `wrench_state_samples.csv`
- total_valid_samples: `19723`
- state_loop_hz: `10.0`

| State | Samples | Min XY m | Mean XY m | P95 XY m | Final XY m | Z range m | Best 1mm ticks | Best 2mm ticks |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| ABORT | 1160 | 0.000062 | 0.117114 | 0.367654 | 0.399294 | 0.825249..1.022876 | 1 | 3 |
| APPROACH | 590 | 0.000243 | 0.002414 | 0.004478 | 0.000255 | 0.838819..0.886469 | 2 | 4 |
| DONE | 10433 | 0.397488 | 0.409276 | 0.412211 | 0.413033 | 1.024171..1.042480 | 0 | 0 |
| MOVING_TO_START | 2780 | 0.000060 | 0.158251 | 0.397199 | 0.001603 | 0.858180..1.038906 | 2 | 5 |
| SEARCH | 4500 | 0.000038 | 0.002285 | 0.004609 | 0.003757 | 0.811853..0.843057 | 4 | 10 |
| UNKNOWN | 260 | 0.404076 | 0.409344 | 0.411149 | 0.407443 | 1.033643..1.041678 | 0 | 0 |

Interpretation: this is an offline passive-log diagnostic. The state-loop window estimate samples the observer stream at 10 Hz, matching the task controller cadence closely enough to decide whether a sustained gate is plausible before changing controller behavior.
