# XY Stability Analysis

- input_dir: `diagnostics/research_baseline_search_gate_trace_recenter8_gain3000_damping10_25hz_v1`
- sample_source: `wrench_state_samples.csv`
- total_valid_samples: `9114`
- state_loop_hz: `25.0`

| State | Samples | Min XY m | Mean XY m | P95 XY m | Final XY m | Z range m | Best 1mm ticks | Best 2mm ticks |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| ABORT | 2451 | 0.000073 | 0.135116 | 0.373213 | 0.397617 | 0.839340..1.024388 | 5 | 12 |
| APPROACH | 1168 | 0.000041 | 0.001142 | 0.002239 | 0.000880 | 0.842290..0.886803 | 6 | 39 |
| DONE | 59 | 0.397842 | 0.404378 | 0.409305 | 0.409011 | 1.023922..1.039146 | 0 | 0 |
| IDLE | 46 | 0.406737 | 0.409257 | 0.410705 | 0.409000 | 1.037357..1.040838 | 0 | 0 |
| INSERT | 1204 | 0.000036 | 0.001165 | 0.002250 | 0.001588 | 0.840509..0.845897 | 4 | 30 |
| MOVING_TO_START | 4024 | 0.000124 | 0.226686 | 0.406784 | 0.000626 | 0.860632..1.041236 | 1 | 6 |
| UNKNOWN | 162 | 0.407594 | 0.409621 | 0.410448 | 0.410576 | 1.037529..1.040442 | 0 | 0 |

Interpretation: this is an offline passive-log diagnostic. The state-loop window estimate samples the observer stream at the task controller cadence recorded in trial_outcome.json when available, or the explicit --state-loop-hz override. This keeps sustained-clearance tick evidence honest when running non-default cadence diagnostics.
