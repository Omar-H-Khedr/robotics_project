# Research Baseline Repeat Validation

- Trials completed: 3
- Physical successes: 1
- Success rate: 0.3333
- Timeouts: 0
- Safety aborts: 0
- CSV: `diagnostics/research_baseline_insert_predepth_recenter_500hz_repeat_v3/repeat_trials.csv`

Physical success requires measured insertion depth, contact evidence, and no safety abort. This report does not convert a single run into a robustness claim.

## Configuration

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run experiment_manager research_baseline_repeat_validator \
  --trials 3 \
  --timeout-s 300 \
  --output-dir diagnostics/research_baseline_insert_predepth_recenter_500hz_repeat_v3 \
  --per-trial-tracking-logs \
  --extra-launch-arg use_gui:=false \
  --extra-launch-arg control_rate:=25.0 \
  --extra-launch-arg position_gain:=3000.0 \
  --extra-launch-arg position_derivative_gain:=10.0 \
  --extra-launch-arg joint_damping_scale:=10.0 \
  --extra-launch-arg inject_velocity_state:=true \
  --extra-launch-arg velocity_state_controller_config_path:=config/research_baseline_velocity_state_500hz.yaml \
  --extra-launch-arg search_recenter_duration_s:=8.0 \
  --extra-launch-arg search_settle_duration_s:=9.0 \
  --extra-launch-arg insert_handoff_timeout_s:=12.0
```

The earlier 180 s repeat attempts were too short for the bounded recenter path:
recenter can add another handoff and descent window. The task node also now
writes final outcome JSON immediately on ABORT, so aborted trials are
classified by task evidence instead of becoming harness `NO_OUTCOME` rows.

## Trial Results

| Trial | Outcome | Depth m | Final XY m | Recenter attempts | Peak raw Fz N | Insert contact N | Reason |
| ---: | --- | ---: | ---: | ---: | ---: | ---: | --- |
| 1 | `ABORTED` | `0.0021` | `0.0012` | `1` | `94.83` | `41.27` | side-loaded at depth `0.0019 m`, XY `0.0012 m` for 3 ticks |
| 2 | `SUCCESS` | `0.0206` | `0.0006` | `2` | `97.09` | `45.53` | full cycle completed |
| 3 | `ABORTED` | `0.0040` | `0.0013` | `2` | `99.28` | `44.33` | side-loaded at depth `0.0038 m`, XY `0.0013 m` for 3 ticks |

All three trials bypassed SEARCH. The repeat result is therefore not evidence
that SEARCH-entering runs are robust.

## Decision

Keep the bounded recenter and ABORT outcome logging changes because they are
safer and more auditable than the previous behavior. Do not claim robust
peg-in-hole success: the repeat success rate remains `1/3`, and the remaining
failure mode is shallow inserted-depth side-load drift after one or two
bounded recenter attempts.
