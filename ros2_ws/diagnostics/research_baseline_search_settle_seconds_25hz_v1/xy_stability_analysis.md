# XY Stability Analysis

- input_dir: `diagnostics/research_baseline_search_settle_seconds_25hz_v1`
- sample_source: `wrench_state_samples.csv`
- total_valid_samples: `8665`
- state_loop_hz: `25.0`

| State | Samples | Min XY m | Mean XY m | P95 XY m | Final XY m | Z range m | Best 1mm ticks | Best 2mm ticks |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| APPROACH | 1172 | 0.000134 | 0.002121 | 0.004049 | 0.001505 | 0.839983..0.887126 | 3 | 11 |
| MOVING_TO_START | 4036 | 0.000406 | 0.225509 | 0.407823 | 0.001117 | 0.858818..1.041225 | 2 | 2 |
| SEARCH | 3181 | 0.000052 | 0.002298 | 0.004260 | 0.001588 | 0.822695..0.845273 | 4 | 6 |
| UNKNOWN | 276 | 0.404000 | 0.409473 | 0.411242 | 0.411863 | 1.034642..1.041645 | 0 | 0 |

Interpretation: this is an offline passive-log diagnostic. The state-loop window estimate samples the observer stream at the task controller cadence recorded in trial_outcome.json when available, or the explicit --state-loop-hz override. This keeps sustained-clearance tick evidence honest when running non-default cadence diagnostics.
