# XY Stability Analysis

- input_dir: `diagnostics/research_baseline_current_joint_handoff_recenter8_gain3000_damping10_25hz_v1`
- sample_source: `wrench_state_samples.csv`
- total_valid_samples: `10497`
- state_loop_hz: `25.0`

| State | Samples | Min XY m | Mean XY m | P95 XY m | Final XY m | Z range m | Best 1mm ticks | Best 2mm ticks |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| ABORT | 545 | 0.000147 | 0.001853 | 0.003524 | 0.001212 | 0.822680..0.850218 | 7 | 27 |
| APPROACH | 1152 | 0.000040 | 0.001199 | 0.002290 | 0.001015 | 0.843231..0.886698 | 6 | 32 |
| MOVING_TO_START | 4004 | 0.000371 | 0.227058 | 0.406453 | 0.000941 | 0.859682..1.039534 | 3 | 3 |
| SEARCH | 4500 | 0.000007 | 0.001326 | 0.002552 | 0.000502 | 0.822884..0.845925 | 9 | 40 |
| UNKNOWN | 296 | 0.407241 | 0.409532 | 0.410658 | 0.408363 | 1.037118..1.041155 | 0 | 0 |

Interpretation: this is an offline passive-log diagnostic. The state-loop window estimate samples the observer stream at the task controller cadence recorded in trial_outcome.json when available, or the explicit --state-loop-hz override. This keeps sustained-clearance tick evidence honest when running non-default cadence diagnostics.
