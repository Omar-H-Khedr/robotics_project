# XY Stability Analysis

- input_dir: `diagnostics/research_baseline_search_feedback_compensated_recenter_v1`
- sample_source: `wrench_state_samples.csv`
- total_valid_samples: `15265`
- state_loop_hz: `10.0`

| State | Samples | Min XY m | Mean XY m | P95 XY m | Final XY m | Z range m | Best 1mm ticks | Best 2mm ticks |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| ABORT | 2460 | 0.000148 | 0.111407 | 0.364934 | 0.397041 | 0.817789..1.022561 | 1 | 2 |
| APPROACH | 1140 | 0.000087 | 0.002048 | 0.003688 | 0.000420 | 0.843041..0.886380 | 2 | 14 |
| DONE | 2655 | 0.394277 | 0.409127 | 0.412291 | 0.413114 | 1.021338..1.042756 | 0 | 0 |
| MOVING_TO_START | 4183 | 0.000091 | 0.218729 | 0.406975 | 0.001074 | 0.857728..1.040193 | 2 | 4 |
| SEARCH | 4500 | 0.000040 | 0.002472 | 0.004900 | 0.003319 | 0.817850..0.847419 | 4 | 11 |
| UNKNOWN | 327 | 0.405731 | 0.409543 | 0.411615 | 0.409397 | 1.033561..1.042209 | 0 | 0 |

Interpretation: this is an offline passive-log diagnostic. The state-loop window estimate samples the observer stream at 10 Hz, matching the task controller cadence closely enough to decide whether a sustained gate is plausible before changing controller behavior.
