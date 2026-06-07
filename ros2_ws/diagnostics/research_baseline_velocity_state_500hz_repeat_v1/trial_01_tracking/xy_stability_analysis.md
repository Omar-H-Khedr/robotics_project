# XY Stability Analysis

- input_dir: `diagnostics/research_baseline_velocity_state_500hz_repeat_v1/trial_01_tracking`
- sample_source: `wrench_state_samples.csv`
- total_valid_samples: `10447`
- state_loop_hz: `25.0`

| State | Samples | Min XY m | Mean XY m | P95 XY m | Final XY m | Z range m | Best 1mm ticks | Best 2mm ticks |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| APPROACH | 1168 | 0.000012 | 0.000462 | 0.000886 | 0.000413 | 0.843433..0.886242 | 175 | 292 |
| DONE | 43 | 0.398123 | 0.404654 | 0.409611 | 0.409371 | 1.025012..1.039915 | 0 | 0 |
| INSERT | 2536 | 0.000011 | 0.000639 | 0.001162 | 0.000251 | 0.788910..0.844427 | 78 | 634 |
| MOVING_TO_START | 4006 | 0.000234 | 0.228773 | 0.408981 | 0.000587 | 0.862857..1.040277 | 3 | 5 |
| RETREAT | 2468 | 0.000003 | 0.094073 | 0.361000 | 0.398235 | 0.789016..1.024602 | 23 | 347 |
| UNKNOWN | 226 | 0.408480 | 0.409589 | 0.410193 | 0.409309 | 1.038559..1.040274 | 0 | 0 |

Interpretation: this is an offline passive-log diagnostic. The state-loop window estimate samples the observer stream at the task controller cadence recorded in trial_outcome.json when available, or the explicit --state-loop-hz override. This keeps sustained-clearance tick evidence honest when running non-default cadence diagnostics.
