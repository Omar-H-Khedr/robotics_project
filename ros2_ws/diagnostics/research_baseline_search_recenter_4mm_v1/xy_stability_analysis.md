# XY Stability Analysis

- input_dir: `diagnostics/research_baseline_search_recenter_4mm_v1`
- sample_source: `wrench_state_samples.csv`
- total_valid_samples: `20347`
- state_loop_hz: `10.0`

| State | Samples | Min XY m | Mean XY m | P95 XY m | Final XY m | Z range m | Best 1mm ticks | Best 2mm ticks |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| ABORT | 2460 | 0.000069 | 0.125915 | 0.369990 | 0.394569 | 0.834576..1.026384 | 1 | 7 |
| APPROACH | 1231 | 0.000087 | 0.002042 | 0.003786 | 0.000896 | 0.837392..0.888461 | 3 | 10 |
| DONE | 11077 | 0.393567 | 0.409253 | 0.412199 | 0.411470 | 1.021882..1.042976 | 0 | 0 |
| INSERT | 40 | 0.000254 | 0.001813 | 0.003896 | 0.002664 | 0.834636..0.843987 | 3 | 4 |
| MOVING_TO_START | 5279 | 0.000081 | 0.175671 | 0.406573 | 0.002820 | 0.858642..1.041882 | 3 | 8 |
| UNKNOWN | 260 | 0.405163 | 0.409578 | 0.411271 | 0.411495 | 1.033702..1.042218 | 0 | 0 |

Interpretation: this is an offline passive-log diagnostic. The state-loop window estimate samples the observer stream at 10 Hz, matching the task controller cadence closely enough to decide whether a sustained gate is plausible before changing controller behavior.
