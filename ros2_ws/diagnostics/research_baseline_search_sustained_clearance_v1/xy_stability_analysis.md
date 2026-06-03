# XY Stability Analysis

- input_dir: `diagnostics/research_baseline_search_sustained_clearance_v1`
- sample_source: `wrench_state_samples.csv`
- total_valid_samples: `14813`
- state_loop_hz: `10.0`

| State | Samples | Min XY m | Mean XY m | P95 XY m | Final XY m | Z range m | Best 1mm ticks | Best 2mm ticks |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| ABORT | 2460 | 0.000449 | 0.110703 | 0.364895 | 0.394361 | 0.817961..1.022437 | 0 | 1 |
| APPROACH | 1250 | 0.000134 | 0.002459 | 0.004795 | 0.000795 | 0.837247..0.886559 | 2 | 4 |
| DONE | 2153 | 0.395314 | 0.409126 | 0.412284 | 0.411366 | 1.018438..1.042327 | 0 | 0 |
| MOVING_TO_START | 4220 | 0.000130 | 0.217933 | 0.408882 | 0.001482 | 0.859390..1.041672 | 1 | 4 |
| SEARCH | 4500 | 0.000037 | 0.003108 | 0.005935 | 0.004354 | 0.818132..0.842639 | 2 | 6 |
| UNKNOWN | 230 | 0.405358 | 0.409566 | 0.412037 | 0.410225 | 1.033645..1.042118 | 0 | 0 |

Interpretation: this is an offline passive-log diagnostic. The state-loop window estimate samples the observer stream at 10 Hz, matching the task controller cadence closely enough to decide whether a sustained gate is plausible before changing controller behavior.
