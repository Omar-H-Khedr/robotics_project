# Hold Window Reference Analyzer

Milestone: `research_baseline_hold_window_reference_analyzer_v1`

Status: `completed_diagnostic`

## Purpose

Add a passive offline analyzer for trajectory command windows. The analyzer
reconstructs peg-tip Cartesian reference and feedback from
`trajectory_controller_state_samples.csv`, groups samples by
`trajectory_commands.csv`, and reports whether single-point hold-like commands
keep feedback inside the `0.0010 m` physical radial clearance.

The analyzer does not publish commands and does not change task safety gates.

## Commands

```bash
python3 -m py_compile src/thesis_bringup/thesis_bringup/hold_window_reference_analyzer.py
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select thesis_bringup
source install/setup.bash
ros2 run thesis_bringup hold_window_reference_analyzer diagnostics/research_baseline_search_recenter_4mm_v1
ros2 run thesis_bringup hold_window_reference_analyzer diagnostics/research_baseline_search_recenter_4mm_v1_repeat2
```

## Result

- Syntax check passed.
- Targeted `colcon build --packages-select thesis_bringup` passed.
- `diagnostics/research_baseline_search_recenter_4mm_v1/hold_window_reference_analysis.md`
  was generated.
- `diagnostics/research_baseline_search_recenter_4mm_v1_repeat2/hold_window_reference_analysis.md`
  was generated.

## Findings

For `research_baseline_search_recenter_4mm_v1`, the direct INSERT handoff hold
had a centered target, but feedback achieved only `3` estimated task ticks
inside `0.0010 m`.

For `research_baseline_search_recenter_4mm_v1_repeat2`, seven hold-like
commands were observed. The best feedback window across those holds was only
`2` estimated task ticks inside `0.0010 m`, while centered references often had
longer inside-clearance windows.

## Interpretation

The current blocker is not just a bad success metric or an obviously wrong
center target. The task can command centered or near-centered hold references,
but controller feedback does not remain inside the physical radial clearance
for the required `8` consecutive task ticks. The next implementation should
target feedback stabilization or command sequencing while preserving the
strict physical gate.
