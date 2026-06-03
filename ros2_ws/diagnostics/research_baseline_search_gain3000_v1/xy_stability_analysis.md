# XY Stability Analysis

- input_dir: `diagnostics/research_baseline_search_gain3000_v1`
- sample_source: `wrench_state_samples.csv`
- total_valid_samples: `20974`
- state_loop_hz: `10.0`

| State | Samples | Min XY m | Mean XY m | P95 XY m | Final XY m | Z range m | Best 1mm ticks | Best 2mm ticks |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| ABORT | 2470 | 0.000245 | 0.112433 | 0.367364 | 0.399075 | 0.817147..1.025345 | 1 | 2 |
| APPROACH | 1140 | 0.000092 | 0.002112 | 0.003931 | 0.003256 | 0.842351..0.887368 | 2 | 6 |
| DONE | 8154 | 0.397518 | 0.409236 | 0.412132 | 0.411366 | 1.023123..1.042763 | 0 | 0 |
| MOVING_TO_START | 4444 | 0.000085 | 0.204687 | 0.405158 | 0.001764 | 0.857741..1.042067 | 3 | 8 |
| SEARCH | 4500 | 0.000026 | 0.003125 | 0.006329 | 0.002071 | 0.816972..0.845593 | 2 | 4 |
| UNKNOWN | 266 | 0.406128 | 0.409551 | 0.411501 | 0.411383 | 1.032833..1.042105 | 0 | 0 |

Interpretation: this is an offline passive-log diagnostic. The state-loop window estimate samples the observer stream at 10 Hz, matching the task controller cadence closely enough to decide whether a sustained gate is plausible before changing controller behavior.
