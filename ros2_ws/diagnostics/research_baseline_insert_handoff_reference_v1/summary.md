# Insert Handoff Reference Diagnostic

Milestone: `research_baseline_insert_handoff_reference_v1`

Date: 2026-06-03

## Purpose

Determine whether the immediate INSERT clearance failure is caused by an
off-center JTC reference or by feedback/plant drift away from an otherwise
centered reference.

This is an offline diagnostic over existing passive CSV evidence. It does not
publish commands, change robot behavior, loosen gates, or claim insertion
success.

## Commands

```bash
python3 -m py_compile src/thesis_bringup/thesis_bringup/insert_handoff_reference_analyzer.py
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run thesis_bringup insert_handoff_reference_analyzer diagnostics/research_baseline_insert_cartesian_descent_v1
colcon build --symlink-install --packages-select thesis_bringup
```

The same analyzer was also run on
`diagnostics/research_baseline_insert_precontact_clearance_gate_v1`.

## Evidence Files

- `precontact_clearance_gate_insert_handoff_reference_analysis.md`
- `precontact_clearance_gate_insert_handoff_reference_analysis.json`
- `cartesian_descent_insert_handoff_reference_analysis.md`
- `cartesian_descent_insert_handoff_reference_analysis.json`

## Results

For `research_baseline_insert_precontact_clearance_gate_v1`:

- selected INSERT command index `2`;
- command point count `1`;
- pre-command final reference XY error `0.000011 m`;
- pre-command final feedback XY error `0.002101 m`;
- initial reference and feedback XY error `0.001125 m`;
- first reference and feedback clearance violation `0.005 s` after command receipt;
- max feedback XY error in the first `0.5 s` was `0.005525 m`.

For rejected `research_baseline_insert_cartesian_descent_v1`:

- selected INSERT command index `2`;
- command point count `6`;
- pre-command final reference XY error `0.000000 m`;
- pre-command final feedback XY error `0.001660 m`;
- initial reference and feedback XY error `0.000458 m`;
- reference stayed inside physical clearance for the analyzed `0.5 s` window;
- feedback violated physical clearance `0.005 s` after command receipt;
- max reference XY error in the first `0.5 s` was `0.000510 m`;
- max feedback XY error in the first `0.5 s` was `0.004264 m`;
- max Cartesian reference-feedback error was `0.004571 m`.

## Interpretation

The rejected multi-waypoint Cartesian descent kept the JTC reference centered
inside the `0.0010 m` physical radial clearance, but controller feedback still
left clearance within `0.005 s` and before meaningful insertion depth.

This evidence points away from a simple final-target or waypoint-count error.
The next implementation should stabilize the INSERT handoff/feedback behavior
or add a bounded pre-insert settling mechanism while preserving the physical
clearance gate.

## Limitations

- This milestone analyzes existing failed trials only.
- No new Gazebo runtime was needed because the diagnostic uses the raw
  controller-state CSVs already captured for the two failed INSERT runs.
- No physical insertion success is claimed.
