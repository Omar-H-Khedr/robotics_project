# peg_in_hole_description

`peg_in_hole_description` owns the simulated assembly scene for the safe adaptive peg-in-hole research framework. It provides the canonical Gazebo workcell, task geometry, fixture models, and frame placeholders used by later control, perception, safety, and experiment packages.

## Research Purpose

The scene is intentionally simple and reviewable: primitive SDF geometry, explicit dimensions, stable frame names, and conservative contact settings. This makes it suitable for experiment design where clearance, insertion depth, fixture pose, contact behavior, sensing, and controller assumptions must be traceable.

## Geometry Assumptions

- KUKA model: real-scale `lbr_iisy3_r760`; the robot is not scaled in the task scene.
- End-effector attachment: the upstream KUKA iisy macro provides `flange` as the ROS-Industrial attachment frame and `tool0` as the all-zero tool frame.
- Phase 2 research gripper: a fixed/passive two-finger parallel gripper is attached to `flange`; it adds `gripper_palm`, `gripper_left_finger`, `gripper_right_finger`, and `gripper_tcp`.
- Work table: `0.80 m x 0.60 m` top, `0.05 m` top thickness, `0.75 m` surface height.
- Robot stand: static `robot_pedestal`, spawned at `[0.80, -0.75, 0.0]`, raises the KUKA base to the table-surface baseline height.
- Robot spawn in the unified baseline: base frame on the pedestal at `[0.80, -0.75, 0.75]` with yaw `1.5708 rad`, aligned with the table centerline and facing the table.
- Table placement in the unified baseline: centered at `[0.80, 0.0, 0.0]`, putting the robot-side tabletop edge at `y=-0.30 m` and leaving about `0.45 m` of base-to-table clearance.
- Task workspace: peg, hole fixture, and target plate are on the robot-side region of the tabletop.
- Peg radius: `0.0125 m`
- Peg length: `0.11 m`
- Hole radius: `0.0135 m`
- Radial clearance: `0.001 m`
- Target plate footprint: `0.18 m x 0.18 m`
- Target plate thickness: `0.02 m`
- Table surface height: `0.75 m`
- Nominal target frame: `target_hole_frame` at `[0.52, -0.20, 0.81]` in `world`
- Insertion axis: negative target-frame Z, `[0.0, 0.0, -1.0]`
- Nominal insertion depth: `0.08 m`

The SDF target plate uses primitive collision bars around the aperture and a visual hole marker. This keeps the scene lightweight while preserving the research-critical radial clearance values in `config/task_geometry.yaml`.

## Contents

- `models/work_table`: static laboratory work table.
- `models/robot_pedestal`: static 0.75 m robot stand for the KUKA research baseline.
- `models/cylindrical_peg`: dynamic cylindrical peg model.
- `models/hole_fixture`: static fixture block under the target plate.
- `models/target_plate`: target plate with the nominal hole opening.
- `worlds/peg_in_hole_world.sdf`: Gazebo world containing the table, fixture, target plate, peg, ground plane, and camera-ready lighting.
- `urdf/peg_in_hole_task.urdf.xacro`: reusable task-frame placeholders for future TF integration.
- `urdf/research_parallel_gripper.xacro`: simplified fixed two-finger research gripper macro with primitive visual/collision geometry and a `gripper_tcp` frame.
- `urdf/lbr_iisy3_r760_research_gripper.urdf.xacro`: project-owned KUKA wrapper that includes the upstream KUKA iisy description and attaches the passive gripper at `flange`.
- `config/task_geometry.yaml`: canonical task geometry parameters.

## Launch

Build and source the workspace, then launch the standalone scene:

```bash
ros2 launch peg_in_hole_description peg_in_hole_scene.launch.py
```

For headless server-only startup:

```bash
ros2 launch peg_in_hole_description peg_in_hole_scene.launch.py use_gui:=false
```

The standalone launch is useful for reviewing task geometry without the robot. For the unified research environment, use the `thesis_bringup` baseline instead:

```bash
ros2 launch thesis_bringup research_baseline.launch.py
```

That launch starts this package's `worlds/peg_in_hole_world.sdf` and then spawns the KUKA robot, passive research gripper, and controller stack in the same Gazebo simulation. The pedestal-mounted baseline uses `z=0.75` because the table surface is `0.75 m`; the older floor-mounted `z=0.0` placement made the arm appear under the table.

## Research Gripper

The Phase 2 gripper is intentionally simplified: one palm block, two fixed fingers, primitive box visuals/collisions, lightweight inertials, and a fixed `gripper_tcp` frame centered between the fingertips. It is passive in this first version; there are no finger joints, transmissions, mimic joints, or gripper controllers yet.

The gripper is implemented in this package instead of modifying `external/kuka_robot_descriptions`. The project-owned wrapper `lbr_iisy3_r760_research_gripper.urdf.xacro` reuses the upstream KUKA `lbr_iisy3_r760` macro and attaches the gripper to the upstream `flange` link. This keeps the research tool model versioned with the peg-in-hole workcell while preserving the vendor robot description.

## Integration Path

This package is the geometric source of truth for the assembly task. Later phases should connect it as follows:

- KUKA control reads the target and insertion geometry to generate approach, search, contact, and insertion trajectories.
- The safety layer uses peg radius, clearance, insertion axis, and nominal depth to constrain force, velocity, and workspace behavior.
- Perception uses the table, fixture, plate, lighting, and frame placeholders for camera calibration and target localization.
- Experiment management varies fixture pose, clearance assumptions, insertion depth, and disturbance conditions while preserving a versioned baseline.
