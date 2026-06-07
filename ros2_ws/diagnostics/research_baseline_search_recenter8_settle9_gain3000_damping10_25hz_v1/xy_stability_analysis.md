# XY Stability Analysis

- input_dir: `diagnostics/research_baseline_search_recenter8_settle9_gain3000_damping10_25hz_v1`
- sample_source: `wrench_state_samples.csv`
- total_valid_samples: `11246`
- state_loop_hz: `25.0`

| State | Samples | Min XY m | Mean XY m | P95 XY m | Final XY m | Z range m | Best 1mm ticks | Best 2mm ticks |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| ABORT | 2452 | 0.000095 | 0.117535 | 0.367423 | 0.397584 | 0.825958..1.021912 | 3 | 13 |
| APPROACH | 1180 | 0.000029 | 0.001263 | 0.002427 | 0.001679 | 0.840908..0.886095 | 5 | 43 |
| DONE | 1854 | 0.397605 | 0.409294 | 0.411207 | 0.409378 | 1.022120..1.041698 | 0 | 0 |
| INSERT | 1208 | 0.000019 | 0.001281 | 0.002559 | 0.001085 | 0.827744..0.832737 | 5 | 33 |
| MOVING_TO_START | 4005 | 0.000236 | 0.228634 | 0.408628 | 0.000257 | 0.860909..1.040332 | 1 | 2 |
| SEARCH | 264 | 0.000026 | 0.001156 | 0.002064 | 0.000735 | 0.830683..0.844132 | 3 | 29 |
| UNKNOWN | 283 | 0.406593 | 0.409537 | 0.410983 | 0.408873 | 1.035964..1.041075 | 0 | 0 |

Interpretation: this is an offline passive-log diagnostic. The state-loop window estimate samples the observer stream at the task controller cadence recorded in trial_outcome.json when available, or the explicit --state-loop-hz override. This keeps sustained-clearance tick evidence honest when running non-default cadence diagnostics.
