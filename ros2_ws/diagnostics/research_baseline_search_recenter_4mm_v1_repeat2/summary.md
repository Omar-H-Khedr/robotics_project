# Search Recenter 4 mm V1 Repeat 2

Date: 2026-06-03

## Change

SEARCH recentering uses a bounded `0.0040 m` near-center band. If feedback is
inside that band but has not sustained the physical `0.0010 m` clearance, the
controller publishes a centered no-contact hold at current peg Z rather than
immediately advancing to the next spiral offset. INSERT still requires
`8` consecutive control ticks inside the physical `0.0010 m` clearance.

## Validation

Runtime command:

```bash
mkdir -p /tmp/ros2_logs /tmp/gz_home
export ROS_LOG_DIR=/tmp/ros2_logs
export HOME=/tmp/gz_home
source /opt/ros/jazzy/setup.bash
cd /home/omar/code/robotics_project/ros2_ws
source install/setup.bash
timeout 430s ros2 launch thesis_bringup research_baseline.launch.py use_gui:=false joint_damping_scale:=5.0 tracking_log_dir:=diagnostics/research_baseline_search_recenter_4mm_v1_repeat2
```

The wrapper exited with timeout code `124` after the task had already reported
its final outcome and observer summaries had flushed.

## Runtime Outcome

- outcome: `ABORTED`
- reason: `SEARCH timeout (45s). XY error 0.0017m remains above tolerance.`
- insertion depth: `0.0000 m`
- max raw `|Fz|`: `130.5 N`
- max raw force norm: `211.5 N`
- task contact estimate: `87.5 N`
- final reported pre-insertion XY error: `0.0036 m`
- phase sequence: `MOVING_TO_START OK`, `APPROACH OK`, `SEARCH FAIL`
- INSERT was not entered.
- The console log showed six SEARCH recenter attempts before timeout.

## Passive Observer Evidence

- `contact_state_summary.md`: `0` contact-topic samples.
- `trajectory_tracking_summary.md`: `10` observed commands, `53591` JTC
  controller-state samples, p95 controller-state max abs position error
  `0.011373 rad`.
- `wrench_state_summary.md`: `21612` wrench samples, max raw `|Fz|`
  `130.549946 N`, max force norm `211.531302 N`.
- `xy_stability_analysis.md`: SEARCH mean XY `0.002402 m`, final SEARCH XY
  `0.001682 m`, best estimated `0.0010 m` clearance window `4` controller
  ticks, best estimated `0.0020 m` window `8` controller ticks.

## Decision

Keep the widened recenter trigger as a bounded improvement, not a success. It
reduced mean/final SEARCH XY compared with the 2 mm recenter run, but still did
not meet the required `8` consecutive control ticks inside physical clearance.
The next milestone should target the remaining handoff/hold oscillation rather
than loosening the `0.0010 m` gate.
