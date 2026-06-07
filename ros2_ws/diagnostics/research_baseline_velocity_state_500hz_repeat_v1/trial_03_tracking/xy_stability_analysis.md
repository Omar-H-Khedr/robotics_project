# XY Stability Analysis

- input_dir: `diagnostics/research_baseline_velocity_state_500hz_repeat_v1/trial_03_tracking`
- sample_source: `wrench_state_samples.csv`
- total_valid_samples: `8401`
- state_loop_hz: `25.0`

| State | Samples | Min XY m | Mean XY m | P95 XY m | Final XY m | Z range m | Best 1mm ticks | Best 2mm ticks |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| ABORT | 2453 | 0.000340 | 0.125951 | 0.370384 | 0.397874 | 0.835302..1.023994 | 3 | 253 |
| APPROACH | 1160 | 0.000001 | 0.000499 | 0.000981 | 0.000953 | 0.844022..0.885607 | 67 | 290 |
| DONE | 25 | 0.398223 | 0.400795 | 0.403054 | 0.402737 | 1.024165..1.031144 | 0 | 0 |
| IDLE | 68 | 0.408443 | 0.409560 | 0.410235 | 0.409493 | 1.038526..1.040343 | 0 | 0 |
| INSERT | 511 | 0.000015 | 0.000623 | 0.001188 | 0.000304 | 0.836002..0.844643 | 17 | 128 |
| MOVING_TO_START | 4052 | 0.000360 | 0.230903 | 0.409527 | 0.000672 | 0.862972..1.040384 | 2 | 5 |
| UNKNOWN | 132 | 0.408568 | 0.409625 | 0.410144 | 0.409996 | 1.038447..1.040343 | 0 | 0 |

Interpretation: this is an offline passive-log diagnostic. The state-loop window estimate samples the observer stream at the task controller cadence recorded in trial_outcome.json when available, or the explicit --state-loop-hz override. This keeps sustained-clearance tick evidence honest when running non-default cadence diagnostics.
