# Research Baseline Endpoint Hold Dynamics Analyzer V1

Date: 2026-06-03

## Purpose

Add a reusable passive diagnostic that isolates the endpoint hold window after
the axis-align MOVING_TO_START trajectory duration has elapsed and before the
next trajectory command. The analyzer prefers JTC controller-state tracking
when present and reports Cartesian oscillation, strict-gate occupancy, estimated
10 Hz stable bins, and per-joint hold ranges.

No runtime behavior, targets, gains, geometry, or safety gates were changed.

## Commands

```bash
python3 -m py_compile src/thesis_bringup/thesis_bringup/endpoint_hold_dynamics_analyzer.py
colcon build --symlink-install --packages-select thesis_bringup
ros2 run thesis_bringup endpoint_hold_dynamics_analyzer diagnostics/research_baseline_controller_state_tracking_v2
```

## Evidence

- `diagnostics/research_baseline_controller_state_tracking_v2/endpoint_hold_dynamics_analysis.md`
- `diagnostics/research_baseline_controller_state_tracking_v2/endpoint_hold_dynamics_analysis.json`

## Result

- tracking source: `trajectory_controller_state_samples.csv`
- endpoint hold samples: `5930`
- endpoint hold duration: `23.716 s`
- XY error mean: `0.009221 m`
- XY error p95: `0.015782 m`
- XY error max: `0.023077 m`
- X range: `0.041273 m`
- Y range: `0.035843 m`
- Z range: `0.033742 m`
- strict XY samples: `140`
- strict XY sample fraction: `0.023609`
- strict 10 Hz bins: `0`
- max consecutive strict 10 Hz bins: `0`
- largest joint feedback range: `joint_1`, `0.057121 rad`

## Conclusion

The above-hole endpoint is not a small static bias. It oscillates across a
multi-centimeter Cartesian range during the post-command hold window, and no
estimated 0.1 s state-machine bin remains entirely inside the 2 mm XY gate.
The next stabilization work should address endpoint hold dynamics or damping
authority directly. The safety gate remains valid and should not be relaxed.
