# Research Baseline Repeat Validation

- Trials completed: 5
- Physical successes: 4
- Success rate: 0.8
- Timeouts: 0
- Safety aborts: 0
- Side-load aborts: 0
- CSV: `diagnostics/research_baseline_shallow_sideload_withdraw_500hz_repeat_v1/repeat_trials.csv`

Physical success requires measured insertion depth, contact evidence, and no safety abort. This report does not convert a single run into a robustness claim.

| Trial | Outcome | Success | Failed phase | Depth m | Final XY m | Peak raw Fz N | Contact N | Insert contact N | Pre-depth recenters | Shallow side-load recoveries | Timeout | Safety abort | Side-load abort |
| ---: | --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- | --- | --- |
| 1 | `SUCCESS` | `True` | `` | 0.0202 | 0.0004 | 99.39 | 65.00 | 47.41 | 1 | 0 | `False` | `False` | `False` |
| 2 | `SUCCESS` | `True` | `` | 0.0201 | 0.0002 | 99.15 | 68.01 | 46.05 | 2 | 1 | `False` | `False` | `False` |
| 3 | `SUCCESS` | `True` | `` | 0.0205 | 0.0003 | 99.46 | 77.54 | 53.17 | 1 | 0 | `False` | `False` | `False` |
| 4 | `ABORTED` | `False` | `INSERT` | 0.0000 | 0.0021 | 93.71 | 64.88 | 43.27 | 2 | 0 | `False` | `False` | `False` |
| 5 | `SUCCESS` | `True` | `` | 0.0207 | 0.0007 | 95.97 | 75.02 | 54.15 | 1 | 0 | `False` | `False` | `False` |

## Interpretation

This milestone improves repeat evidence from `1/3` in
`research_baseline_insert_predepth_recenter_500hz_repeat_v3` to `4/5` physical
successes with no timeouts and no safety aborts. The safety gates were not
weakened: physical radial clearance remained `0.0010 m`, the handoff stability
gate remained `8` ticks, and force thresholds were unchanged.

Trial 2 exercised the new shallow inserted side-load recovery: the controller
withdrew vertically after a shallow side-load event, restarted INSERT handoff,
and then completed a measured insertion-depth event. Trials 1, 3, and 5 did
not need shallow side-load withdrawal and also reached measured insertion
depth.

Trial 4 is the remaining blocker. It failed closed before meaningful insertion
depth after two bounded pre-depth recenters. The failure reason was no-contact
XY drift (`0.0021 m`) above the fixed `0.0010 m` clearance, not an inserted
side-load failure, timeout, force safety abort, contact-estimation failure, or
hole/peg geometry success case. SEARCH was bypassed in all five trials, so this
repeat does not validate SEARCH-entering robustness.

## Exact Command

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
rm -rf diagnostics/research_baseline_shallow_sideload_withdraw_500hz_repeat_v1
ros2 run experiment_manager research_baseline_repeat_validator \
  --trials 5 \
  --timeout-s 420 \
  --output-dir diagnostics/research_baseline_shallow_sideload_withdraw_500hz_repeat_v1 \
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

Next control blocker: reduce or prevent no-contact pre-depth XY drift after
repeated INSERT handoff recenters while preserving the fixed physical clearance
gate. SEARCH-entering robustness remains unresolved because these repeats did
not enter SEARCH.
