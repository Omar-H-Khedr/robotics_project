# XY Stability Analysis

- input_dir: `diagnostics/research_baseline_done_shutdown_recenter8_settle9_gain3000_damping10_25hz_v1`
- sample_source: `wrench_state_samples.csv`
- total_valid_samples: `8943`
- state_loop_hz: `25.0`

| State | Samples | Min XY m | Mean XY m | P95 XY m | Final XY m | Z range m | Best 1mm ticks | Best 2mm ticks |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| APPROACH | 1156 | 0.000011 | 0.001205 | 0.002352 | 0.000829 | 0.842947..0.886871 | 8 | 33 |
| MOVING_TO_START | 4006 | 0.000513 | 0.227128 | 0.406553 | 0.001429 | 0.860089..1.039237 | 3 | 6 |
| SEARCH | 3507 | 0.000011 | 0.001199 | 0.002264 | 0.001312 | 0.821986..0.846169 | 7 | 54 |
| UNKNOWN | 274 | 0.406593 | 0.409494 | 0.410798 | 0.408721 | 1.035964..1.041212 | 0 | 0 |

Interpretation: this is an offline passive-log diagnostic. The state-loop window estimate samples the observer stream at the task controller cadence recorded in trial_outcome.json when available, or the explicit --state-loop-hz override. This keeps sustained-clearance tick evidence honest when running non-default cadence diagnostics.
