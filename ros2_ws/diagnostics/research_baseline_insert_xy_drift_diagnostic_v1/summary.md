# Insert XY Drift Diagnostic

Milestone: `research_baseline_insert_xy_drift_diagnostic_v1`

Date: 2026-06-03

## Purpose

Diagnose the current INSERT blocker before changing motion behavior. The
question was whether the side-loaded insertion failure is caused only by late
contact/extraction, or whether the peg-tip XY pose already violates the
physical 25 mm peg / 27 mm hole radial clearance during the no-contact INSERT
handoff and early descent.

## Change

Added the offline analyzer:

```bash
ros2 run thesis_bringup insert_xy_drift_analyzer <diagnostics_dir>
```

The analyzer reads `trajectory_commands.csv` and the preferred
`trajectory_controller_state_samples.csv`, reconstructs peg-tip Cartesian
feedback with the project kinematics, selects the INSERT command by FK target,
and reports:

- pre-command XY/depth boundary samples;
- first physical-clearance violation;
- first meaningful insertion depth;
- first side-loaded depth event;
- correlated nearest F/T and contact-topic values.

It does not publish commands or alter safety gates.

## Validation Commands

```bash
python3 -m py_compile \
  src/thesis_bringup/thesis_bringup/insert_xy_drift_analyzer.py \
  src/thesis_bringup/setup.py

source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select thesis_bringup

source install/setup.bash
ros2 run thesis_bringup insert_xy_drift_analyzer \
  diagnostics/research_baseline_insert_sideload_abort_v1
ros2 run thesis_bringup insert_xy_drift_analyzer \
  diagnostics/research_baseline_insert_physical_xy_gate_v1
```

All commands completed.

## Evidence

Current side-load abort run:

- analysis: `../research_baseline_insert_sideload_abort_v1/insert_xy_drift_analysis.md`
- INSERT command: index `2`, one trajectory point, observed `12.810 s`
- pre-command final XY error: `0.001569 m`
- initial command-window XY error: `0.002539 m`
- first meaningful depth: `0.001316 m` at XY error `0.002488 m`
- first side-load event: same sample as first meaningful depth
- max XY error during observed INSERT: `0.008116 m`

Prior physical-XY-gate run:

- analysis: `../research_baseline_insert_physical_xy_gate_v1/insert_xy_drift_analysis.md`
- INSERT command: index `2`, one trajectory point, observed `23.100 s`
- pre-command final XY error: `0.001380 m`
- initial command-window XY error: `0.001151 m`
- first meaningful depth: `0.001103 m` at XY error `0.000323 m`
- first side-load event: `0.001274 m` depth at XY error `0.002153 m`
- max XY error during observed INSERT: `0.005838 m`

## Interpretation

This is not physical success. The evidence shows that the controller feedback
can exceed the `0.001 m` physical radial clearance before or during early
INSERT, even when the task-level pre-insertion value is near the allowed band.
The next implementation should preserve the side-load abort and add an
explicit no-contact INSERT clearance gate before descent, then address the
single-point INSERT path/control behavior that lets XY drift recur during
descent.
