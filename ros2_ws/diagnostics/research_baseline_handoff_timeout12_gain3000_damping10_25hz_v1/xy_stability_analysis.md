# XY Stability Analysis

- input_dir: `diagnostics/research_baseline_handoff_timeout12_gain3000_damping10_25hz_v1`
- sample_source: `wrench_state_samples.csv`
- total_valid_samples: `10490`
- state_loop_hz: `25.0`

| State | Samples | Min XY m | Mean XY m | P95 XY m | Final XY m | Z range m | Best 1mm ticks | Best 2mm ticks |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| ABORT | 546 | 0.000099 | 0.001888 | 0.003175 | 0.002530 | 0.821834..0.848746 | 3 | 14 |
| APPROACH | 1172 | 0.000020 | 0.001197 | 0.002286 | 0.001918 | 0.842207..0.886520 | 7 | 35 |
| MOVING_TO_START | 3992 | 0.000059 | 0.225787 | 0.405938 | 0.000441 | 0.860720..1.040252 | 1 | 4 |
| SEARCH | 4500 | 0.000029 | 0.001458 | 0.002933 | 0.001360 | 0.822286..0.844725 | 5 | 46 |
| UNKNOWN | 280 | 0.406737 | 0.409531 | 0.410705 | 0.410320 | 1.037320..1.040838 | 0 | 0 |

Interpretation: this is an offline passive-log diagnostic. The state-loop window estimate samples the observer stream at the task controller cadence recorded in trial_outcome.json when available, or the explicit --state-loop-hz override. This keeps sustained-clearance tick evidence honest when running non-default cadence diagnostics.
