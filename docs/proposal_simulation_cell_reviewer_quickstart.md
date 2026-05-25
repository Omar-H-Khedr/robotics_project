# Proposal Simulation Cell Reviewer Quickstart

This quickstart points reviewers to the proposal simulation cell documentation and diagnostics without requiring scenario execution or robot motion.

## Primary Review Files

- Release index: `docs/proposal_simulation_cell_release_index.md`
- Sprint traceability: `docs/proposal_simulation_cell_sprint_traceability.md`
- No-false-claims statement: `docs/proposal_simulation_cell_no_false_claims_statement.md`
- Evidence package: `ros2_ws/diagnostics/proposal_simulation_cell_v1_15/`
- Reproducibility checklist: `ros2_ws/diagnostics/proposal_simulation_cell_v1_16/`

## Documentation Verification

The v1.17 launch verifies that the release documentation files, README files, v1.15 evidence package, and v1.16 reproducibility checklist are present.

```bash
cd /home/omar/code/robotics_project/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch thesis_bringup proposal_simulation_cell_v1_17_release_documentation_index.launch.py
```

The launch is diagnostic-only. It publishes release-index status and reviewer-quickstart status, writes diagnostics to `ros2_ws/diagnostics/proposal_simulation_cell_v1_17/`, and does not execute scenarios or motion.

## Key Historical Launches

```bash
ros2 launch thesis_bringup proposal_simulation_cell_v1_2_rgbd_image_bridge_fix.launch.py
ros2 launch thesis_bringup proposal_simulation_cell_v1_3_contact_physics_validation.launch.py
ros2 launch thesis_bringup proposal_simulation_cell_v1_5_safety_virtual_force_interface.launch.py
ros2 launch thesis_bringup proposal_simulation_cell_v1_6_safety_gate_readiness.launch.py
ros2 launch thesis_bringup proposal_simulation_cell_v1_9_no_motion_control_law_dry_run.launch.py
ros2 launch thesis_bringup proposal_simulation_cell_v1_10_experiment_configuration_matrix.launch.py
ros2 launch thesis_bringup proposal_simulation_cell_v1_15_evidence_package_generator.launch.py
ros2 launch thesis_bringup proposal_simulation_cell_v1_16_reproducibility_checklist.launch.py
```

## Review Boundaries

Sprint `v1.4` is absent/not implemented. The release does not claim real robot execution, controller execution, MoveIt use, `/compute_ik` calls, `FollowJointTrajectory` use, scenario execution, fake datasets, fake plots, or experimental results.
