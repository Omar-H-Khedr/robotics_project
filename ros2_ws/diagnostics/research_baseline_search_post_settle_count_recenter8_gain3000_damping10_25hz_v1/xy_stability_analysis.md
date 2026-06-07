# XY Stability Analysis

- input_dir: `diagnostics/research_baseline_search_post_settle_count_recenter8_gain3000_damping10_25hz_v1`
- sample_source: `wrench_state_samples.csv`
- total_valid_samples: `9159`
- state_loop_hz: `25.0`

| State | Samples | Min XY m | Mean XY m | P95 XY m | Final XY m | Z range m | Best 1mm ticks | Best 2mm ticks |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| ABORT | 2452 | 0.000051 | 0.126029 | 0.370365 | 0.395904 | 0.839198..1.023647 | 5 | 30 |
| APPROACH | 1176 | 0.000067 | 0.001273 | 0.002543 | 0.000942 | 0.842576..0.886677 | 5 | 37 |
| DONE | 59 | 0.395991 | 0.404245 | 0.409840 | 0.409999 | 1.023711..1.040949 | 0 | 0 |
| INSERT | 1208 | 0.000054 | 0.001216 | 0.002365 | 0.000557 | 0.839504..0.845127 | 9 | 53 |
| MOVING_TO_START | 4005 | 0.000442 | 0.226831 | 0.406499 | 0.000478 | 0.860288..1.040359 | 1 | 5 |
| UNKNOWN | 259 | 0.407158 | 0.409612 | 0.410724 | 0.409087 | 1.036182..1.041355 | 0 | 0 |

Interpretation: this is an offline passive-log diagnostic. The state-loop window estimate samples the observer stream at the task controller cadence recorded in trial_outcome.json when available, or the explicit --state-loop-hz override. This keeps sustained-clearance tick evidence honest when running non-default cadence diagnostics.
