# XY Stability Analysis

- input_dir: `diagnostics/research_baseline_search_damping10_gain3000_25hz_v1`
- sample_source: `wrench_state_samples.csv`
- total_valid_samples: `8309`
- state_loop_hz: `25.0`

| State | Samples | Min XY m | Mean XY m | P95 XY m | Final XY m | Z range m | Best 1mm ticks | Best 2mm ticks |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| ABORT | 2281 | 0.000025 | 0.106786 | 0.332091 | 0.356302 | 0.839199..0.978620 | 3 | 10 |
| APPROACH | 1184 | 0.000041 | 0.001268 | 0.002328 | 0.000295 | 0.841975..0.886090 | 10 | 39 |
| INSERT | 604 | 0.000052 | 0.001188 | 0.002226 | 0.002123 | 0.839815..0.844695 | 4 | 56 |
| MOVING_TO_START | 3996 | 0.000812 | 0.228456 | 0.408005 | 0.001759 | 0.860488..1.040438 | 0 | 3 |
| UNKNOWN | 244 | 0.407920 | 0.409662 | 0.411026 | 0.409375 | 1.037231..1.041148 | 0 | 0 |

Interpretation: this is an offline passive-log diagnostic. The state-loop window estimate samples the observer stream at the task controller cadence recorded in trial_outcome.json when available, or the explicit --state-loop-hz override. This keeps sustained-clearance tick evidence honest when running non-default cadence diagnostics.
