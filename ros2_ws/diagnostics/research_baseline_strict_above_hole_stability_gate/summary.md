# Research Baseline Strict Above-Hole Stability Gate

Date: 2026-06-02

## Purpose

Prevent descent from a transient above-hole crossing. The previous primitive-collision run showed that `MOVING_TO_START` could proceed from a single degraded XY-good sample while the robot was still moving; by the first `APPROACH` tick, the peg had drifted laterally off the insertion axis.

This milestone preserves the safety policy: never descend toward the fixture unless the above-hole pose is actually stable.

## Implementation

- Removed the degraded `MOVING_TO_START` proceed path.
- `MOVING_TO_START` now requires the existing strict stable condition for `STABILIZE_TICKS` before transition:
  - joint error within `JOINT_TOLERANCE`;
  - Cartesian error within `CARTESIAN_TOLERANCE`;
  - XY error within the 2 mm `APPROACH_START_XY_TOLERANCE`.
- On timeout, the node records a complete phase failure with Cartesian error, XY error, joint error, and stability counter.
- Added a Cartesian-sampled approach trajectory helper for future descent validation, but the validation run did not reach `APPROACH` after the stricter gate was enforced.

## Verification Commands

```bash
cd /home/omar/code/robotics_project/ros2_ws
python3 -m py_compile src/kuka_task_control/kuka_task_control/admittance_insertion_node.py
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select kuka_task_control thesis_bringup
timeout 120s ros2 launch thesis_bringup research_baseline.launch.py use_gui:=false
```

## Results

- Python syntax check passed.
- Targeted build passed for `kuka_task_control` and `thesis_bringup`.
- Headless launch spawned the robot and activated controllers.
- The run did not descend into `APPROACH`.
- `MOVING_TO_START` timed out after 90 s and transitioned to `ABORT`, then `DONE`, with a complete outcome report.

Final runtime evidence:

```text
MOVING_TO_START timeout/failure (90.0s).
cart_err=0.022m, xy_err=0.018m, joint_err=0.034rad, stable=0/5
Outcome: ABORTED
Depth: 0.0000m
Max Fz: 707.9N
Contact: 680.4N
```

## Interpretation

This is the correct safety behavior for the observed tracking quality. The peg came close in Cartesian distance but did not hold the 2 mm XY gate for the required stable window. Descent was therefore blocked.

The next blocker is not insertion search or learned control. It is stable controller/physics tracking at the above-hole target, including the high free-space F/T spike observed during this no-contact phase.
