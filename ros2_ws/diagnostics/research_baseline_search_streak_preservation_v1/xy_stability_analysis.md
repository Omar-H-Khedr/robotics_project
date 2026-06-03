# XY Stability Analysis

- input_dir: `diagnostics/research_baseline_search_streak_preservation_v1`
- sample_source: `wrench_state_samples.csv`
- total_valid_samples: `20240`
- state_loop_hz: `10.0`

| State | Samples | Min XY m | Mean XY m | P95 XY m | Final XY m | Z range m | Best 1mm ticks | Best 2mm ticks |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| ABORT | 2470 | 0.000069 | 0.118688 | 0.370258 | 0.396785 | 0.822581..1.025933 | 2 | 4 |
| APPROACH | 1230 | 0.000066 | 0.002089 | 0.003792 | 0.001585 | 0.838901..0.887968 | 2 | 9 |
| DONE | 7430 | 0.398501 | 0.409258 | 0.412172 | 0.409253 | 1.024627..1.042596 | 0 | 0 |
| MOVING_TO_START | 4303 | 0.000120 | 0.217827 | 0.406882 | 0.001450 | 0.858593..1.042054 | 0 | 7 |
| SEARCH | 4500 | 0.000059 | 0.002276 | 0.004225 | 0.002779 | 0.823072..0.843175 | 3 | 8 |
| UNKNOWN | 307 | 0.404466 | 0.409619 | 0.412039 | 0.412039 | 1.033559..1.042023 | 0 | 0 |

Interpretation: this is an offline passive-log diagnostic. The state-loop window estimate samples the observer stream at 10 Hz, matching the task controller cadence closely enough to decide whether a sustained gate is plausible before changing controller behavior.
