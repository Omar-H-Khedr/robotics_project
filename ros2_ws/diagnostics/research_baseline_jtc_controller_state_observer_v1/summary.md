# research_baseline_jtc_controller_state_observer_v1

Status: validated instrumentation fix.

The trajectory tracking observer previously subscribed to
`/joint_trajectory_controller/state`, which produced `jtc_state_samples: 0` in
all recent summaries. The ROS 2 Jazzy joint trajectory controller publishes its
`control_msgs/msg/JointTrajectoryControllerState` stream on the controller
private `controller_state` topic, which resolves here to:

```text
/joint_trajectory_controller/controller_state
```

The canonical launch and observer default now use that topic.

## Validation

```bash
python3 -m py_compile src/thesis_bringup/thesis_bringup/trajectory_tracking_observer.py src/thesis_bringup/launch/research_baseline.launch.py
colcon build --symlink-install --packages-select thesis_bringup
ros2 launch thesis_bringup research_baseline.launch.py use_gui:=false tracking_log_dir:=diagnostics/research_baseline_jtc_controller_state_observer_v1
```

The runtime launch was intentionally bounded by a 60 s timeout, so this is not
task-completion or insertion evidence.

## Evidence

`trajectory_tracking_summary.md` reports:

- state topic: `/joint_trajectory_controller/controller_state`
- observed commands: `1`
- JTC state samples: `4186`
- joint-state-derived tracking samples: `3829`
- p95 max absolute joint error: `0.026664 rad`
- final max absolute joint error: `0.019792 rad`

This fixes an observer instrumentation gap only. It does not change control
commands, safety gates, gains, damping, task targets, or success criteria.
