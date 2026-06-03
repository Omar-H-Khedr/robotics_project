# XY Stability Analysis

- input_dir: `diagnostics/research_baseline_search_post_command_stability_gate_v1_repeat2`
- sample_source: `wrench_state_samples.csv`
- total_valid_samples: `21852`
- state_loop_hz: `10.0`

| State | Samples | Min XY m | Mean XY m | P95 XY m | Final XY m | Z range m | Best 1mm ticks | Best 2mm ticks |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| ABORT | 2470 | 0.000103 | 0.111376 | 0.367963 | 0.400038 | 0.814364..1.025831 | 3 | 6 |
| APPROACH | 1190 | 0.000030 | 0.002444 | 0.005282 | 0.001861 | 0.839852..0.886838 | 2 | 4 |
| DONE | 8422 | 0.398274 | 0.409255 | 0.412089 | 0.411302 | 1.021718..1.042733 | 0 | 0 |
| IDLE | 80 | 0.403351 | 0.409307 | 0.411946 | 0.408917 | 1.034706..1.042141 | 0 | 0 |
| MOVING_TO_START | 5070 | 0.000052 | 0.181293 | 0.405745 | 0.000925 | 0.859043..1.041115 | 2 | 5 |
| SEARCH | 4500 | 0.000025 | 0.002371 | 0.004688 | 0.002798 | 0.814844..0.845316 | 4 | 13 |
| UNKNOWN | 120 | 0.405830 | 0.409594 | 0.412052 | 0.411080 | 1.035631..1.041421 | 0 | 0 |

Interpretation: this is an offline passive-log diagnostic. The state-loop window estimate samples the observer stream at 10 Hz, matching the task controller cadence closely enough to decide whether a sustained gate is plausible before changing controller behavior.
