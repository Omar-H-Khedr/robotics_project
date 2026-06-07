# XY Stability Analysis

- input_dir: `diagnostics/research_baseline_search_gain3000_settle_seconds_25hz_v1`
- sample_source: `wrench_state_samples.csv`
- total_valid_samples: `8561`
- state_loop_hz: `25.0`

| State | Samples | Min XY m | Mean XY m | P95 XY m | Final XY m | Z range m | Best 1mm ticks | Best 2mm ticks |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| ABORT | 2452 | 0.000078 | 0.126089 | 0.369646 | 0.399739 | 0.837363..1.022891 | 3 | 5 |
| APPROACH | 1144 | 0.000052 | 0.002223 | 0.004223 | 0.000194 | 0.842122..0.886939 | 3 | 9 |
| DONE | 685 | 0.396737 | 0.408752 | 0.412053 | 0.407973 | 1.022439..1.042346 | 0 | 0 |
| INSERT | 12 | 0.000347 | 0.001946 | 0.003026 | 0.000735 | 0.839170..0.845828 | 1 | 1 |
| MOVING_TO_START | 4000 | 0.000766 | 0.228384 | 0.407538 | 0.001017 | 0.858318..1.041463 | 0 | 2 |
| UNKNOWN | 268 | 0.405019 | 0.409523 | 0.412150 | 0.406031 | 1.033696..1.041797 | 0 | 0 |

Interpretation: this is an offline passive-log diagnostic. The state-loop window estimate samples the observer stream at the task controller cadence recorded in trial_outcome.json when available, or the explicit --state-loop-hz override. This keeps sustained-clearance tick evidence honest when running non-default cadence diagnostics.
