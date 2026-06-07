# XY Stability Analysis

- input_dir: `diagnostics/research_baseline_velocity_state_500hz_repeat_v1/trial_02_tracking`
- sample_source: `wrench_state_samples.csv`
- total_valid_samples: `8277`
- state_loop_hz: `25.0`

| State | Samples | Min XY m | Mean XY m | P95 XY m | Final XY m | Z range m | Best 1mm ticks | Best 2mm ticks |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| ABORT | 2452 | 0.000032 | 0.125823 | 0.370460 | 0.399254 | 0.840728..1.024183 | 12 | 162 |
| APPROACH | 1176 | 0.000018 | 0.000497 | 0.000969 | 0.000445 | 0.843208..0.886064 | 65 | 294 |
| DONE | 48 | 0.397987 | 0.403586 | 0.408305 | 0.408305 | 1.024579..1.038088 | 0 | 0 |
| INSERT | 288 | 0.000020 | 0.000540 | 0.001012 | 0.000917 | 0.841699..0.844437 | 46 | 72 |
| MOVING_TO_START | 4005 | 0.000235 | 0.228431 | 0.408825 | 0.000326 | 0.862839..1.040247 | 2 | 4 |
| UNKNOWN | 308 | 0.408590 | 0.409641 | 0.410191 | 0.409705 | 1.038282..1.040397 | 0 | 0 |

Interpretation: this is an offline passive-log diagnostic. The state-loop window estimate samples the observer stream at the task controller cadence recorded in trial_outcome.json when available, or the explicit --state-loop-hz override. This keeps sustained-clearance tick evidence honest when running non-default cadence diagnostics.
