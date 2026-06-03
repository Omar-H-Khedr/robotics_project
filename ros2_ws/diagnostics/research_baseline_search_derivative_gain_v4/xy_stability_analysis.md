# XY Stability Analysis

- input_dir: `diagnostics/research_baseline_search_derivative_gain_v4`
- sample_source: `wrench_state_samples.csv`
- total_valid_samples: `20254`
- state_loop_hz: `10.0`

| State | Samples | Min XY m | Mean XY m | P95 XY m | Final XY m | Z range m | Best 1mm ticks | Best 2mm ticks |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| ABORT | 2469 | 0.000376 | 0.111615 | 0.366547 | 0.400324 | 0.816921..1.025252 | 1 | 1 |
| APPROACH | 1181 | 0.000040 | 0.002209 | 0.004137 | 0.001913 | 0.841119..0.886689 | 2 | 7 |
| DONE | 7784 | 0.396075 | 0.409264 | 0.412185 | 0.408208 | 1.022676..1.042453 | 0 | 0 |
| MOVING_TO_START | 4060 | 0.000049 | 0.224111 | 0.406445 | 0.002126 | 0.857943..1.042333 | 1 | 4 |
| SEARCH | 4500 | 0.000028 | 0.002510 | 0.004776 | 0.000342 | 0.816366..0.846016 | 3 | 7 |
| UNKNOWN | 260 | 0.404291 | 0.409402 | 0.411367 | 0.408083 | 1.034507..1.041509 | 0 | 0 |

Interpretation: this is an offline passive-log diagnostic. The state-loop window estimate samples the observer stream at 10 Hz, matching the task controller cadence closely enough to decide whether a sustained gate is plausible before changing controller behavior.
