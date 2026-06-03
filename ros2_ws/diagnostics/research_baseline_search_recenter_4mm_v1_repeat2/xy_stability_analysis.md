# XY Stability Analysis

- input_dir: `diagnostics/research_baseline_search_recenter_4mm_v1_repeat2`
- sample_source: `wrench_state_samples.csv`
- total_valid_samples: `21611`
- state_loop_hz: `10.0`

| State | Samples | Min XY m | Mean XY m | P95 XY m | Final XY m | Z range m | Best 1mm ticks | Best 2mm ticks |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| ABORT | 2470 | 0.000129 | 0.112079 | 0.367826 | 0.399792 | 0.817841..1.025403 | 2 | 2 |
| APPROACH | 1181 | 0.000113 | 0.002130 | 0.003931 | 0.003570 | 0.838847..0.888645 | 4 | 11 |
| DONE | 8641 | 0.397852 | 0.409270 | 0.412172 | 0.411977 | 1.024506..1.042644 | 0 | 0 |
| IDLE | 68 | 0.406017 | 0.409652 | 0.412902 | 0.412322 | 1.035454..1.041467 | 0 | 0 |
| MOVING_TO_START | 4620 | 0.000080 | 0.198241 | 0.405298 | 0.001987 | 0.858697..1.042544 | 3 | 5 |
| SEARCH | 4499 | 0.000057 | 0.002402 | 0.004616 | 0.001682 | 0.820221..0.845884 | 4 | 8 |
| UNKNOWN | 132 | 0.405353 | 0.409621 | 0.412324 | 0.409581 | 1.035358..1.041382 | 0 | 0 |

Interpretation: this is an offline passive-log diagnostic. The state-loop window estimate samples the observer stream at 10 Hz, matching the task controller cadence closely enough to decide whether a sustained gate is plausible before changing controller behavior.
