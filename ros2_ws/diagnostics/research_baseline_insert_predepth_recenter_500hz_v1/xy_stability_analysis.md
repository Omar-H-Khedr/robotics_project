# XY Stability Analysis

- input_dir: `diagnostics/research_baseline_insert_predepth_recenter_500hz_v1`
- sample_source: `wrench_state_samples.csv`
- total_valid_samples: `11001`
- state_loop_hz: `25.0`

| State | Samples | Min XY m | Mean XY m | P95 XY m | Final XY m | Z range m | Best 1mm ticks | Best 2mm ticks |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| APPROACH | 1172 | 0.000010 | 0.000490 | 0.000970 | 0.000343 | 0.843527..0.886106 | 108 | 293 |
| DONE | 59 | 0.398619 | 0.406279 | 0.410013 | 0.409618 | 1.025300..1.040309 | 0 | 0 |
| INSERT | 3040 | 0.000011 | 0.000575 | 0.001138 | 0.000348 | 0.788827..0.844402 | 62 | 760 |
| MOVING_TO_START | 4000 | 0.000173 | 0.228515 | 0.408662 | 0.000493 | 0.862475..1.039762 | 3 | 3 |
| RETREAT | 2468 | 0.000019 | 0.094229 | 0.361824 | 0.398456 | 0.788633..1.025388 | 15 | 346 |
| UNKNOWN | 262 | 0.408097 | 0.409629 | 0.410219 | 0.408757 | 1.038187..1.040401 | 0 | 0 |

Interpretation: this is an offline passive-log diagnostic. The state-loop window estimate samples the observer stream at the task controller cadence recorded in trial_outcome.json when available, or the explicit --state-loop-hz override. This keeps sustained-clearance tick evidence honest when running non-default cadence diagnostics.
