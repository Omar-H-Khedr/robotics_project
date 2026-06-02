# Research Baseline F/T Mount Effort-Limit Validation

Date: 2026-06-02

## Purpose

Validate whether the previous zero-range `ft_sensor_joint` limit of `effort=1`
was contributing to nonphysical raw wrench spikes during no-contact
`MOVING_TO_START`.

The joint remains a zero-range revolute joint because `gz sdf -p` collapses URDF
fixed joints into the upstream link, removing the named joint needed for the
joint-level Gazebo force-torque sensor.

## Change Under Test

- `ft_sensor_joint` remains `type="revolute"` with lower/upper limits at `0`.
- Limit changed from `effort=1`, `velocity=0` to `effort=10000`,
  `velocity=100`.
- `spawn_robot_sdf.py` documentation now records why the joint is preserved as a
  zero-range measurement joint.
- The active world comment now identifies the KUKA LBR iisy 6 R1300 workcell.

## Validation

Syntax/model checks:

```bash
python3 -m py_compile src/thesis_bringup/thesis_bringup/spawn_robot_sdf.py
source install/setup.bash
xacro src/peg_in_hole_description/urdf/lbr_iisy6_r1300_research_gripper.urdf.xacro mode:=gazebo > /tmp/lbr_iisy6_effort_limit.urdf
gz sdf -p /tmp/lbr_iisy6_effort_limit.urdf > /tmp/lbr_iisy6_effort_limit.sdf
rg -n "ft_sensor_joint|effort|velocity|lower|upper" /tmp/lbr_iisy6_effort_limit.sdf
```

Targeted build:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select peg_in_hole_description thesis_bringup kuka_task_control
```

Runtime:

```bash
mkdir -p /tmp/ros2_logs /tmp/gz_home diagnostics/research_baseline_ft_mount_effort_limit
export ROS_LOG_DIR=/tmp/ros2_logs
export HOME=/tmp/gz_home
source install/setup.bash
timeout 150s ros2 launch thesis_bringup research_baseline.launch.py use_gui:=false tracking_log_dir:=diagnostics/research_baseline_ft_mount_effort_limit
```

## Runtime Result

The launch spawned `lbr_iisy6_r1300`, loaded the force-torque sensor, activated
`joint_state_broadcaster` and `joint_trajectory_controller`, and started the
passive trajectory, wrench, and contact observers.

The task still aborted safely in `MOVING_TO_START`:

- outcome: `ABORTED`;
- reason: `Hard force abort: raw wrench exceeded 1000.0N in state MOVING_TO_START`;
- max `|Fz|`: `612.25 N`;
- max force norm: `1765.41 N`;
- insertion depth: `0.0000 m`;
- final phase Cartesian error: `0.010751 m`;
- gravity baseline valid: `true`.

Observer summaries:

- wrench samples: `7310`;
- `MOVING_TO_START` max `|Fz|`: `612.246577 N`;
- `MOVING_TO_START` max force norm: `1765.412969 N`;
- contact-topic samples: `0`;
- positive contact-topic samples: `0`;
- trajectory max absolute joint-position error: `0.048581 rad`;
- trajectory p95 max absolute joint-position error: `0.028216 rad`.

## Conclusion

Raising the artificial measurement-joint effort/velocity limits reduced but did
not eliminate the raw wrench spike. Compared with the previous
contact-correlation validation (`max |Fz|=1396.8 N`, `max |F|=2624.1 N`), this
run reduced peak `|Fz|` and peak force norm, but the preserved global hard-force
abort still triggered correctly.

This is not insertion success. The next blocker is to localize whether the late
`MOVING_TO_START` force norm spike comes from uninstrumented collision pairs,
tool/peg/table geometry proximity, F/T joint semantics, or controller/physics
dynamics near the above-hole target.
