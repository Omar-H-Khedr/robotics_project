# XY Stability Analysis

- input_dir: `diagnostics/research_baseline_search_position_controller_v1`
- sample_source: `wrench_state_samples.csv`
- total_valid_samples: `19711`
- state_loop_hz: `10.0`

| State | Samples | Min XY m | Mean XY m | P95 XY m | Final XY m | Z range m | Best 1mm ticks | Best 2mm ticks |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| ABORT | 1170 | 0.000097 | 0.112427 | 0.369030 | 0.399897 | 0.818589..1.024429 | 2 | 6 |
| APPROACH | 600 | 0.000028 | 0.002122 | 0.003703 | 0.002758 | 0.836973..0.887210 | 1 | 5 |
| DONE | 11061 | 0.398621 | 0.409307 | 0.412250 | 0.407300 | 1.026096..1.043046 | 0 | 0 |
| MOVING_TO_START | 2106 | 0.000335 | 0.215068 | 0.406134 | 0.000971 | 0.857572..1.038926 | 0 | 3 |
| SEARCH | 4500 | 0.000059 | 0.002366 | 0.004622 | 0.001782 | 0.810185..0.841315 | 2 | 12 |
| UNKNOWN | 274 | 0.404274 | 0.409296 | 0.411286 | 0.408109 | 1.033438..1.041777 | 0 | 0 |

Interpretation: this is an offline passive-log diagnostic. The state-loop window estimate samples the observer stream at 10 Hz, matching the task controller cadence closely enough to decide whether a sustained gate is plausible before changing controller behavior.
