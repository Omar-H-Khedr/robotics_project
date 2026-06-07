# XY Stability Analysis

- input_dir: `diagnostics/research_baseline_search_derivative_gain20_recenter8_damping10_25hz_v1`
- sample_source: `wrench_state_samples.csv`
- total_valid_samples: `12458`
- state_loop_hz: `25.0`

| State | Samples | Min XY m | Mean XY m | P95 XY m | Final XY m | Z range m | Best 1mm ticks | Best 2mm ticks |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| ABORT | 2456 | 0.000005 | 0.118179 | 0.368281 | 0.398048 | 0.825633..1.023487 | 3 | 14 |
| APPROACH | 1164 | 0.000048 | 0.001244 | 0.002298 | 0.001942 | 0.842890..0.886042 | 5 | 43 |
| DONE | 58 | 0.396833 | 0.404528 | 0.410010 | 0.409074 | 1.023951..1.039754 | 0 | 0 |
| MOVING_TO_START | 4004 | 0.000431 | 0.226876 | 0.406388 | 0.000874 | 0.860484..1.040868 | 1 | 4 |
| SEARCH | 4500 | 0.000008 | 0.001249 | 0.002397 | 0.001034 | 0.825167..0.845256 | 6 | 33 |
| UNKNOWN | 276 | 0.407339 | 0.409535 | 0.410679 | 0.410208 | 1.036221..1.041287 | 0 | 0 |

Interpretation: this is an offline passive-log diagnostic. The state-loop window estimate samples the observer stream at the task controller cadence recorded in trial_outcome.json when available, or the explicit --state-loop-hz override. This keeps sustained-clearance tick evidence honest when running non-default cadence diagnostics.
