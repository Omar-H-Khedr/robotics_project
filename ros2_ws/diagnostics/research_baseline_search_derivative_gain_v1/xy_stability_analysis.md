# XY Stability Analysis

- input_dir: `diagnostics/research_baseline_search_derivative_gain_v1`
- sample_source: `wrench_state_samples.csv`
- total_valid_samples: `19898`
- state_loop_hz: `10.0`

| State | Samples | Min XY m | Mean XY m | P95 XY m | Final XY m | Z range m | Best 1mm ticks | Best 2mm ticks |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| ABORT | 2470 | 0.000002 | 0.105346 | 0.364346 | 0.400219 | 0.808262..1.023333 | 1 | 2 |
| APPROACH | 1190 | 0.000082 | 0.002151 | 0.004039 | 0.002812 | 0.837983..0.887186 | 1 | 5 |
| DONE | 7078 | 0.395799 | 0.409263 | 0.412182 | 0.410923 | 1.023873..1.042634 | 0 | 0 |
| MOVING_TO_START | 4400 | 0.000153 | 0.207250 | 0.407670 | 0.002606 | 0.858668..1.042381 | 2 | 7 |
| SEARCH | 4500 | 0.000008 | 0.002375 | 0.004468 | 0.003823 | 0.809471..0.845252 | 3 | 7 |
| UNKNOWN | 260 | 0.405304 | 0.409459 | 0.411650 | 0.410656 | 1.035105..1.041645 | 0 | 0 |

Interpretation: this is an offline passive-log diagnostic. The state-loop window estimate samples the observer stream at 10 Hz, matching the task controller cadence closely enough to decide whether a sustained gate is plausible before changing controller behavior.
