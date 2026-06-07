# XY Stability Analysis

- input_dir: `diagnostics/research_baseline_clean_python_shutdown_recenter8_gain3000_damping10_25hz_v3`
- sample_source: `wrench_state_samples.csv`
- total_valid_samples: `12427`
- state_loop_hz: `25.0`

| State | Samples | Min XY m | Mean XY m | P95 XY m | Final XY m | Z range m | Best 1mm ticks | Best 2mm ticks |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| ABORT | 2460 | 0.000028 | 0.118640 | 0.370093 | 0.399357 | 0.826530..1.026330 | 4 | 35 |
| APPROACH | 1140 | 0.000050 | 0.001228 | 0.002362 | 0.001968 | 0.842342..0.886753 | 7 | 34 |
| DONE | 59 | 0.397130 | 0.405528 | 0.410397 | 0.410397 | 1.025031..1.040548 | 0 | 0 |
| MOVING_TO_START | 3996 | 0.000397 | 0.227759 | 0.406619 | 0.000609 | 0.860614..1.041064 | 1 | 2 |
| SEARCH | 4500 | 0.000012 | 0.001286 | 0.002451 | 0.000659 | 0.825742..0.846026 | 9 | 39 |
| UNKNOWN | 272 | 0.407252 | 0.409514 | 0.410529 | 0.409749 | 1.037338..1.041118 | 0 | 0 |

Interpretation: this is an offline passive-log diagnostic. The state-loop window estimate samples the observer stream at the task controller cadence recorded in trial_outcome.json when available, or the explicit --state-loop-hz override. This keeps sustained-clearance tick evidence honest when running non-default cadence diagnostics.
