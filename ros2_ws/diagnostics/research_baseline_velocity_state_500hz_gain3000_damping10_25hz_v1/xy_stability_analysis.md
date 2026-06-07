# XY Stability Analysis

- input_dir: `diagnostics/research_baseline_velocity_state_500hz_gain3000_damping10_25hz_v1`
- sample_source: `wrench_state_samples.csv`
- total_valid_samples: `10484`
- state_loop_hz: `25.0`

| State | Samples | Min XY m | Mean XY m | P95 XY m | Final XY m | Z range m | Best 1mm ticks | Best 2mm ticks |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| APPROACH | 1156 | 0.000026 | 0.000490 | 0.000979 | 0.000226 | 0.844334..0.885541 | 107 | 289 |
| DONE | 60 | 0.398218 | 0.405988 | 0.410000 | 0.410000 | 1.024435..1.040099 | 0 | 0 |
| INSERT | 2532 | 0.000013 | 0.000560 | 0.001055 | 0.000848 | 0.788842..0.844790 | 95 | 633 |
| MOVING_TO_START | 3996 | 0.000194 | 0.228131 | 0.408296 | 0.000194 | 0.862907..1.040024 | 1 | 3 |
| RETREAT | 2468 | 0.000013 | 0.093888 | 0.361413 | 0.398355 | 0.788591..1.024912 | 113 | 346 |
| UNKNOWN | 272 | 0.408169 | 0.409655 | 0.410169 | 0.409246 | 1.038033..1.040416 | 0 | 0 |

Interpretation: this is an offline passive-log diagnostic. The state-loop window estimate samples the observer stream at the task controller cadence recorded in trial_outcome.json when available, or the explicit --state-loop-hz override. This keeps sustained-clearance tick evidence honest when running non-default cadence diagnostics.
