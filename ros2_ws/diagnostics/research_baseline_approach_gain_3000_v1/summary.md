# research_baseline_approach_gain_3000_v1

Date: 2026-06-02

## Purpose

Test whether the `APPROACH` descent failure is caused by insufficient Gazebo position-controller authority. This diagnostic used the existing launch override:

```bash
position_gain:=3000
```

The default retained value remains `1000`. This experiment was rejected and no code/config change was retained.

## Commands

```bash
cd /home/omar/code/robotics_project/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
timeout 240s ros2 launch thesis_bringup research_baseline.launch.py use_gui:=false position_gain:=3000 tracking_log_dir:=diagnostics/research_baseline_approach_gain_3000_v1
```

## Result

- Gazebo confirmed `position_proportional_gain` was set to `3000`.
- Trial outcome: `ABORTED`
- Final reason: `APPROACH timeout/degraded failure (90.0s). cart_err=0.073m, joint_err=0.110rad, tolerance=0.050m`
- `MOVING_TO_START`: success, duration `92.6 s`, `cart_error_m=0.012672`, `joint_error_rad=0.062941`
- Initial above-hole XY error: `0.0014 m`
- `APPROACH`: failed by timeout, `cart_error_m=0.073242`, `joint_error_rad=0.110010`
- Insertion depth: `0.0000 m`
- Peak raw `|Fz|`: `842.85 N`
- Peak raw force norm: `890.27 N`
- Max estimated contact force: `429.65 N`
- SEARCH did not run.

## Tracking Evidence

- approach command duration: `15.000 s`;
- trajectory observer max joint error: `0.111631 rad`;
- trajectory observer p95 max joint error: `0.110895 rad`;
- trajectory observer final max joint error after retreat: `0.005192 rad`.

The higher gain did not reduce the approach joint lag. It slightly improved time to the strict above-hole gate but worsened the approach error and increased peak raw wrench.

## Decision

Rejected. Do not retain `position_gain:=3000` as the canonical launch setting. The next investigation should focus on the model/controller dynamics that keep `joint_2` about 0.11 rad away from the approach target during descent.
