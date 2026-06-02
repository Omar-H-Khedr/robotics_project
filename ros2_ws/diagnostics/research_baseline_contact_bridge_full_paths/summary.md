# Research Baseline Full-Path Contact Bridge Validation

Date: 2026-06-02

## Purpose

Correct the canonical research baseline contact observability. Previous runs
bridged simplified Gazebo contact topic names and observed zero samples, while
Gazebo logs showed contact sensors publishing on fully scoped sensor paths.

The active peg is fixed into the robot model, not spawned as the standalone
`cylindrical_peg`, so a robot-mounted peg contact sensor is required for the
canonical baseline.

## Changes

- `contact_bridge.yaml` now maps fully scoped Gazebo contact sensor paths to
  stable ROS topics:
  - `/gazebo/contacts/peg`;
  - `/gazebo/contacts/hole`;
  - `/gazebo/contacts/target`.
- `spawn_robot_sdf.py` injects `peg_contact_sensor` onto the converted
  `ft_sensor_link`, referencing the lumped grasped-peg collision.

## Validation

Syntax and SDF injection checks:

```bash
python3 -m py_compile src/thesis_bringup/thesis_bringup/spawn_robot_sdf.py src/thesis_bringup/thesis_bringup/contact_state_observer.py
source install/setup.bash
xacro src/peg_in_hole_description/urdf/lbr_iisy6_r1300_research_gripper.urdf.xacro mode:=gazebo > /tmp/lbr_iisy6_contact_probe.urdf
gz sdf -p /tmp/lbr_iisy6_contact_probe.urdf > /tmp/lbr_iisy6_contact_probe_raw.sdf
```

The injected SDF contained `peg_contact_sensor` on `ft_sensor_link` and
referenced the lumped collision
`ft_sensor_link_fixed_joint_lump__grasped_peg_collision_2`.

Targeted build:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select thesis_bringup
```

Runtime:

```bash
mkdir -p /tmp/ros2_logs /tmp/gz_home diagnostics/research_baseline_contact_bridge_full_paths
export ROS_LOG_DIR=/tmp/ros2_logs
export HOME=/tmp/gz_home
source install/setup.bash
timeout 150s ros2 launch thesis_bringup research_baseline.launch.py use_gui:=false tracking_log_dir:=diagnostics/research_baseline_contact_bridge_full_paths
```

## Runtime Result

The launch created all three full-path bridges:

- `/world/peg_in_hole_world/model/lbr_iisy6_r1300/link/ft_sensor_link/sensor/peg_contact_sensor/contact` -> `/gazebo/contacts/peg`;
- `/world/peg_in_hole_world/model/hole_fixture/link/fixture_link/sensor/hole_contact_sensor/contact` -> `/gazebo/contacts/hole`;
- `/world/peg_in_hole_world/model/target_plate/link/plate_link/sensor/target_contact_sensor/contact` -> `/gazebo/contacts/target`.

Gazebo also reported all three contact sensors publishing.

The task still aborted safely in `MOVING_TO_START`:

- outcome: `ABORTED`;
- reason: `Hard force abort: raw wrench exceeded 1000.0N in state MOVING_TO_START`;
- max `|Fz|`: `939.36 N`;
- max force norm: `1748.50 N`;
- insertion depth: `0.0000 m`;
- final phase Cartesian error: `0.009909 m`;
- phase duration: `78.3 s`.

Contact observer result:

- contact samples: `50`;
- positive contact samples: `50`;
- max contact force: `3440.570902 N`;
- `MOVING_TO_START` peg contact samples: `10/10`, max force
  `2427.307742 N`;
- `MOVING_TO_START` target contact samples: `10/10`, max force
  `2427.307742 N`;
- no hole contact samples were recorded.

Wrench and tracking summaries:

- `MOVING_TO_START` max `|Fz|`: `939.358852 N`;
- `MOVING_TO_START` max force norm: `1748.498723 N`;
- min XY error observed by wrench observer: `0.000334 m`;
- trajectory max absolute joint-position error: `0.048369 rad`;
- trajectory p95 max absolute joint-position error: `0.027238 rad`.

## Conclusion

The previous zero-contact result was an observability gap. With full-path
bridging and a robot-mounted peg contact sensor, the raw wrench abort is now
correlated with peg-target contact during `MOVING_TO_START`.

This is not insertion success. The next blocker is geometric/trajectory safety
near the above-hole target: the peg can contact the target plate before the
strict no-contact stability gate admits descent. The next change should adjust
the no-contact start pose, peg-tip clearance, or free-space approach geometry so
`MOVING_TO_START` remains physically above and clear of the fixture while
preserving the existing hard-force abort.
