# XY Stability Analysis

- input_dir: `diagnostics/research_baseline_single_gz_control_25hz_v1`
- sample_source: `wrench_state_samples.csv`
- total_valid_samples: `7032`
- state_loop_hz: `25.0`

| State | Samples | Min XY m | Mean XY m | P95 XY m | Final XY m | Z range m | Best 1mm ticks | Best 2mm ticks |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| APPROACH | 1188 | 0.000093 | 0.002170 | 0.004057 | 0.002262 | 0.840082..0.886490 | 3 | 5 |
| MOVING_TO_START | 4448 | 0.000116 | 0.209287 | 0.408079 | 0.001494 | 0.857795..1.041980 | 2 | 12 |
| SEARCH | 1164 | 0.000084 | 0.002450 | 0.004769 | 0.001735 | 0.826245..0.846176 | 2 | 8 |
| UNKNOWN | 232 | 0.404356 | 0.409451 | 0.411301 | 0.410546 | 1.034111..1.041906 | 0 | 0 |

Interpretation: this is an offline passive-log diagnostic. The state-loop window estimate samples the observer stream at the task controller cadence recorded in trial_outcome.json when available, or the explicit --state-loop-hz override. This keeps sustained-clearance tick evidence honest when running non-default cadence diagnostics.
