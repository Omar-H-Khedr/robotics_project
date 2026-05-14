# peg_in_hole_description

`peg_in_hole_description` owns the simulated assembly scene for the safe adaptive peg-in-hole research framework. It provides the canonical Gazebo workcell, task geometry, fixture models, and frame placeholders used by later control, perception, safety, and experiment packages.

## Research Purpose

The scene is intentionally simple and reviewable: primitive SDF geometry, explicit dimensions, stable frame names, and conservative contact settings. This makes it suitable for experiment design where clearance, insertion depth, fixture pose, contact behavior, sensing, and controller assumptions must be traceable.

## Geometry Assumptions

- KUKA model: real-scale `lbr_iisy3_r760`; the robot is not scaled in the task scene.
- Work table: `0.80 m x 0.60 m` top, `0.05 m` top thickness, `0.75 m` surface height.
- Table placement in the unified baseline: centered at `[0.50, 0.0, 0.0]`, putting the near table edge at `x=0.10 m` from the default robot base at world origin.
- Peg radius: `0.0125 m`
- Peg length: `0.11 m`
- Hole radius: `0.0135 m`
- Radial clearance: `0.001 m`
- Target plate footprint: `0.18 m x 0.18 m`
- Target plate thickness: `0.02 m`
- Table surface height: `0.75 m`
- Nominal target frame: `target_hole_frame` at `[0.48, 0.0, 0.81]` in `world`
- Insertion axis: negative target-frame Z, `[0.0, 0.0, -1.0]`
- Nominal insertion depth: `0.08 m`

The SDF target plate uses primitive collision bars around the aperture and a visual hole marker. This keeps the scene lightweight while preserving the research-critical radial clearance values in `config/task_geometry.yaml`.

## Contents

- `models/work_table`: static laboratory work table.
- `models/cylindrical_peg`: dynamic cylindrical peg model.
- `models/hole_fixture`: static fixture block under the target plate.
- `models/target_plate`: target plate with the nominal hole opening.
- `worlds/peg_in_hole_world.sdf`: Gazebo world containing the table, fixture, target plate, peg, ground plane, and camera-ready lighting.
- `urdf/peg_in_hole_task.urdf.xacro`: reusable task-frame placeholders for future TF integration.
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

That launch starts this package's `worlds/peg_in_hole_world.sdf` and then spawns the KUKA robot and controller stack through `kuka_gazebo` in the same Gazebo simulation.

## Integration Path

This package is the geometric source of truth for the assembly task. Later phases should connect it as follows:

- KUKA control reads the target and insertion geometry to generate approach, search, contact, and insertion trajectories.
- The safety layer uses peg radius, clearance, insertion axis, and nominal depth to constrain force, velocity, and workspace behavior.
- Perception uses the table, fixture, plate, lighting, and frame placeholders for camera calibration and target localization.
- Experiment management varies fixture pose, clearance assumptions, insertion depth, and disturbance conditions while preserving a versioned baseline.
