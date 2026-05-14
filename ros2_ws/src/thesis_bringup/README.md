# thesis_bringup

`thesis_bringup` is the orchestration package for the safe adaptive KUKA peg-in-hole research framework.

Its role is to provide reproducible launch entry points for the complete simulation stack: Gazebo, KUKA robot model, controllers, task scene, safety layer, experiment manager, and later perception or learning modules. Launch files in this package should be treated as experiment protocols rather than quick demos.

## Research Responsibilities

- Define canonical launch configurations for baseline, safety-filtered, perception-enabled, and learning-enabled experiments.
- Centralize high-level parameters that select robot model, Gazebo world, controller configuration, logging mode, and trial metadata.
- Keep experiment launch behavior reproducible across machines and publications.
- Avoid embedding controller logic or task logic directly in launch files; delegate those responsibilities to focused packages.

## Notes

Existing demo packages remain in the workspace for reference, but this package should become the main entry point for thesis experiments.
