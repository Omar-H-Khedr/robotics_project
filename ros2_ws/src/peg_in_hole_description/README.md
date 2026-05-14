# peg_in_hole_description

`peg_in_hole_description` owns the simulated assembly scene: peg geometry, hole geometry, fixtures, workcell frames, Gazebo worlds, and task-specific description files.

## Research Responsibilities

- Provide the canonical peg-in-hole assembly scene used in experiments.
- Keep geometry, frames, materials, contact settings, and world files versioned and reviewable.
- Support systematic variation of clearance, misalignment, insertion depth, fixture pose, and contact properties.
- Provide description resources that can be reused by control, safety, perception, and learning experiments.

## Expected Contents

- `urdf/` or Xacro task objects and workcell frames.
- `worlds/` Gazebo worlds for baseline and experiment scenes.
- `meshes/` visual and collision geometry.
- `config/` task geometry and scene parameters.
