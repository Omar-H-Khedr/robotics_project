# research_baseline_damping_2p0_gain_1500_v1

Status: rejected as canonical controller setting.

This diagnostic tested whether the useful part of the rejected `joint_damping_scale:=2.0`
run could be combined with a moderate global position gain increase:

```bash
ros2 launch thesis_bringup research_baseline.launch.py use_gui:=false joint_damping_scale:=2.0 position_gain:=1500 tracking_log_dir:=diagnostics/research_baseline_damping_2p0_gain_1500_v1
```

Safety gates were unchanged. The run remained in `MOVING_TO_START` and never
entered descent, search, or insertion.

## Runtime Result

- Outcome: `ABORTED`
- Reason: `MOVING_TO_START timeout/failure (120.0s). cart_err=0.006m, xy_err=0.006m, joint_err=0.009rad, stable=0/5, tolerance=0.050m`
- Insertion depth: `0.0000 m`
- Reported max raw `|Fz|`: `166.5 N`
- Reported max raw force norm: `268.1 N`
- Contact-topic samples: `0`

## Passive Evidence

- Above-hole hold analyzer:
  - strict XY gate: `0.002 m`
  - required state-loop hold: `5` ticks at `10 Hz`
  - estimated best stable ticks: `2`
  - estimated gate passed: `False`
  - minimum recorded XY error: `0.000052 m`
  - final `MOVING_TO_START` XY error: `0.004072 m`
- Wrench observer:
  - `MOVING_TO_START` max abs Fz: `166.459344 N`
  - `MOVING_TO_START` max force norm: `250.860262 N`
- Contact observer:
  - samples: `0`
  - positive contact samples: `0`
- Trajectory observer:
  - observed commands: `1`
  - p95 max absolute joint error: `0.016012 rad`
  - final max absolute joint error: `0.009539 rad`
  - the selector-based moving-to-start analyzer returned `NO_AXIS_ALIGN_COMMAND_CAPTURED`, so this run cannot be used as reliable command-attributed start-tracking evidence.

## Conclusion

The combined damping/gain setting produced a transient sub-millimetre XY crossing
but still failed the preserved five-tick strict no-contact hold gate. The source
tree keeps the canonical defaults unchanged. This result reinforces that the
blocker is sustained above-hole hold stability, not mere reachability of the
target pose.
