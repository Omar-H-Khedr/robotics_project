# research_baseline_velocity_state_500hz_repeat_v1

Date: 2026-06-07

## Purpose

Repeat-validate the 500 Hz velocity-state diagnostic configuration after the
single retained insertion-depth event in
`research_baseline_velocity_state_500hz_gain3000_damping10_25hz_v1`.

This validation used fresh Gazebo launches per trial and per-trial tracking
directories. The fixed physical gates were not relaxed:

- physical radial clearance: `0.0010 m`
- SEARCH/INSERT handoff stability: `8` task ticks
- meaningful insertion depth threshold before no-contact side-load abort:
  `0.0010 m`

## Command

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run experiment_manager research_baseline_repeat_validator \
  --trials 3 \
  --timeout-s 260 \
  --output-dir diagnostics/research_baseline_velocity_state_500hz_repeat_v1 \
  --per-trial-tracking-logs \
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

## Result

- trials completed: `3`
- physical successes by harness criteria: `1`
- success rate: `0.3333`
- timeouts: `0`
- safety aborts: `0`
- repeated validation verdict: failed robustness validation

| Trial | Outcome | Success | Failed phase | Depth m | Peak raw Fz N | Sustained contact N | Max Cartesian error m | Timeout | Safety abort | Reason |
|---:|---|---|---|---:|---:|---:|---:|---|---|---|
| 1 | SUCCESS | true | none | 0.0202 | 99.99 | 72.47 | 0.014003 | false | false | Full cycle completed. Insertion depth 0.020m, contact 49.9N during INSERT. |
| 2 | ABORTED | false | INSERT | 0.0000 | 96.58 | 64.80 | 0.013540 | false | false | INSERT aborted: no-contact XY error 0.0012m exceeds physical clearance 0.0010m before meaningful insertion depth 0.0010m for 3 ticks after descent command start. |
| 3 | ABORTED | false | INSERT | 0.0000 | 98.21 | 74.93 | 0.014477 | false | false | INSERT aborted: no-contact XY error 0.0010m exceeds physical clearance 0.0010m before meaningful insertion depth 0.0010m for 3 ticks after descent command start. |

## Per-Trial Tracking Evidence

All three trials bypassed SEARCH because APPROACH ended inside the physical
clearance gate; each per-trial `search_gate_trace.csv` had `0` rows. This
repeat therefore does not validate SEARCH stability.

All three trials recorded `0` positive Gazebo contact-topic samples in
`contact_state_summary.md`. Contact evidence in the task outcome is therefore
wrench-derived/task-side, not confirmed by Gazebo contact-topic rows.

Compact per-trial analyzer outputs are retained under:

- `trial_01_tracking/*_summary.md` and `*_analysis.{md,json}`
- `trial_02_tracking/*_summary.md` and `*_analysis.{md,json}`
- `trial_03_tracking/*_summary.md` and `*_analysis.{md,json}`

Key passive metrics:

| Trial | INSERT samples | INSERT mean XY m | INSERT p95 XY m | INSERT final XY m | Hold-like best feedback 1 mm ticks | Max centered-hold p95 XY drift m | Contact-topic positives |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 1 | 2536 | 0.000639 | 0.001162 | 0.000251 | 59 | 0.000912 | 0 |
| 2 | 288 | 0.000540 | 0.001012 | 0.000917 | 34 | 0.000958 | 0 |
| 3 | 511 | 0.000623 | 0.001188 | 0.000304 | 38 | 0.000928 | 0 |

## Decision

The 500 Hz velocity-state configuration is not robust enough to become the
validated baseline. It produced one successful repeat, but two of three trials
aborted safely during INSERT before meaningful depth because no-contact XY
drift crossed the `0.0010 m` physical clearance gate after descent command
start.

Keep the configuration and the per-trial tracking support as diagnostic tools.
Do not claim robust autonomous peg-in-hole success. The next control work
should target deterministic INSERT handoff/descent centering under the fixed
physical clearance gate, then rerun repeated validation.

Known limitations:

- SEARCH was bypassed in all three trials, so SEARCH robustness remains
  unresolved.
- Gazebo contact-topic observability still reported zero positive samples.
- `ros_gz_bridge` and `gzserver` still exited with `-11` during launch
  teardown.
- Trial 1 launch process return code was `-15` because the harness terminated
  the launch after reading the valid DONE outcome; this did not change the task
  outcome JSON.
