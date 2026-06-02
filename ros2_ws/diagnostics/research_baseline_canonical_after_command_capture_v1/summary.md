# research_baseline_canonical_after_command_capture_v1

Status: canonical validation failed safely before descent.

This run validates the canonical baseline after the trajectory command-capture
instrumentation fix. It used default controller settings:

```bash
ros2 launch thesis_bringup research_baseline.launch.py use_gui:=false tracking_log_dir:=diagnostics/research_baseline_canonical_after_command_capture_v1
```

No gain, damping, effort, target, or safety-gate overrides were used.

## Runtime Result

- Outcome: `ABORTED`
- Reason: `MOVING_TO_START timeout/failure (120.0s). cart_err=0.010m, xy_err=0.010m, joint_err=0.015rad, stable=0/5, tolerance=0.050m`
- Insertion depth: `0.0000 m`
- Reported max raw `|Fz|`: `172.6 N`
- Reported max raw force norm: `270.0 N`
- Descent/search/insertion: not entered

## Observer Evidence

Trajectory command capture:

- discovery wait satisfied: `2/2` subscribers
- observed commands: `2` (`MOVING_TO_START` and abort retreat)
- selector-based command attribution: `OK`
- selected command index: `0`
- selected command duration: `40.0 s`
- selected command target: `[0.520000, -0.200000, 0.885001] m`
- post-command hold before retreat: `22.093 s`

Command-attributed tracking:

- p95 max joint error: `0.024627 rad`
- worst joint by p95 error: `joint_3` at `0.021407 rad`
- final command-attributed Cartesian norm error: `0.003636 m`
- final command-attributed XY error: `0.003578 m`

Above-hole hold analyzer:

- strict XY gate: `0.002 m`
- required hold: `5` ticks at `10 Hz`
- estimated best stable ticks: `1`
- estimated gate passed: `False`
- minimum recorded XY error: `0.000394 m`
- final `MOVING_TO_START` XY error in wrench observer rows: `0.012677 m`

Wrench/contact observers:

- `MOVING_TO_START` max abs Fz: `169.297536 N`
- `MOVING_TO_START` max force norm: `269.970106 N`
- contact-topic samples: `0`
- positive contact samples: `0`

## Conclusion

The first-command capture fix worked, and the canonical run now has usable
command-attributed `MOVING_TO_START` tracking evidence. The task still fails
honestly before descent because the peg tip only crosses the strict 2 mm
above-hole XY gate transiently. The current blocker remains sustained no-contact
above-hole hold stability, not missing command capture and not target
reachability.
