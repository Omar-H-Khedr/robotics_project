# Research Baseline Repeat Validation

- Trials completed: 5
- Physical successes: 5
- Success rate: 1.0
- Timeouts: 0
- Safety aborts: 0
- Side-load aborts: 0
- CSV: `diagnostics/research_baseline_final_sideload_retry_500hz_repeat_v2/repeat_trials.csv`

Physical success requires measured insertion depth, contact evidence, and no safety abort. This report does not convert a single run into a robustness claim.

| Trial | Outcome | Success | Failed phase | Depth m | Final XY m | Max pre-depth XY m | Peak raw Fz N | Contact N | Insert contact N | Pre-depth recenters | Recenter budget resets | Entry descents | Capture descents | Shallow side-load recoveries | Final side-load retries | Timeout | Safety abort | Side-load abort |
| ---: | --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- | --- | --- |
| 1 | `SUCCESS` | `True` | `` | 0.0200 | 0.0002 | 0.0014 | 96.92 | 65.08 | 54.84 | 0 | 0 | 1 | 1 | 0 | 0 | `False` | `False` | `False` |
| 2 | `SUCCESS` | `True` | `` | 0.0202 | 0.0007 | 0.0014 | 101.45 | 68.80 | 58.14 | 0 | 0 | 1 | 1 | 0 | 0 | `False` | `False` | `False` |
| 3 | `SUCCESS` | `True` | `` | 0.0203 | 0.0006 | 0.0021 | 97.21 | 70.74 | 62.81 | 2 | 0 | 1 | 3 | 0 | 0 | `False` | `False` | `False` |
| 4 | `SUCCESS` | `True` | `` | 0.0206 | 0.0008 | 0.0015 | 97.56 | 66.33 | 54.75 | 1 | 0 | 1 | 2 | 0 | 0 | `False` | `False` | `False` |
| 5 | `SUCCESS` | `True` | `` | 0.0198 | 0.0002 | 0.0017 | 97.41 | 74.67 | 50.24 | 1 | 0 | 2 | 1 | 0 | 0 | `False` | `False` | `False` |
