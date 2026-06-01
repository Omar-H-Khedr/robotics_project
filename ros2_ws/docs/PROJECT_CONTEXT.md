# Project Context

Last reviewed: 2026-06-01

This workspace is the active ROS 2 Jazzy / Gazebo implementation for the PhD topic:

**Visuomotor Context-Based Meta-Reinforcement Learning with Virtual-Force Safety for Adaptable Peg-in-Hole Assembly in Smart Manufacturing.**

## Target Platform

- Robot: KUKA LBR iisy 6 R1300
- Axes: 6
- Rated payload: 6 kg
- Maximum payload: 6.9 kg
- Reach: 1300 mm
- Repeatability: +/-0.05 mm
- Footprint: 275 mm x 275 mm
- Approximate mass: 46.3 kg
- Controller family: KR C5 micro / KR C5 micro-2
- Current simulation stack: ROS 2 Jazzy, Gazebo Sim, `gz_ros2_control`

## Current Implementation Status

The current workspace contains a Gazebo workcell with:

- project-local KUKA LBR iisy 6 R1300 model adaptation;
- pedestal-mounted robot spawn at SAFE_HOME;
- fixed gripper and fixed grasped cylindrical peg;
- fixed work table, target plate, and hole fixture;
- `joint_state_broadcaster` and `joint_trajectory_controller`;
- FT sensor injection and ROS bridge to `/ft_sensor_wrench`;
- RGB-D camera model in the world;
- task-level admittance insertion node with phase logging;
- dry-run experiment/context scaffolds from earlier proposal milestones.

The strongest single-run insertion evidence so far is one simulated insertion-depth event:

- insertion depth: about 0.011 m;
- sustained contact: about 142.9 N;
- final phase sequence reached RETREAT/DONE in that run.

Repeated validation on 2026-06-01 produced 0/3 physical successes:

- one DEGRADED INSERT with only 0.0037 m depth and a 1237.45 N peak raw Fz spike;
- one ABORTED INSERT with a 3716.2 N peak raw Fz spike;
- one SEARCH timeout/no-outcome before final insertion evaluation.

This is not robust autonomous peg-in-hole success. The honest claim remains:

**first simulated insertion event with measured insertion depth.**

## Known Open Risks

- Repeat validation is complete for the current controller revision and failed: 0/3 physical successes.
- MOVING_TO_START is non-deterministic, with roughly 1-in-3 failures reported.
- MOVING_TO_START and APPROACH can have large Cartesian tracking errors.
- Peak raw Fz spikes around 1237 N and 3716 N have now been observed in repeated validation.
- Multi-point INSERT trajectory behavior is broken; current INSERT uses a single-point trajectory.
- Contact/gravity estimation depends on median Fz baseline validity and needs more validation.
- Gazebo contact physics are adequate for early simulation evidence but not final safety fidelity.
- Some older docs still describe stale iisy3 state and must not be used as current truth.

## Current Success Criteria

A state machine reaching DONE is insufficient. A physical success trial requires:

- final trial outcome `SUCCESS`;
- measured insertion depth at least 0.010 m;
- contact force above the configured insertion/contact threshold;
- no safety abort;
- no unresolved timeout that invalidates task execution;
- recorded peak raw Fz and Cartesian-error metrics.

Robust success requires repeated validation with a documented success rate and failure modes.

## Next Technical Milestone

`research_baseline_no_contact_alignment_before_descent`

Reason: force-safe insert stabilization prevented some unsafe INSERT attempts, corrected the insertion-depth metric, and added a hard force abort. Validation still failed because unsafe raw force spikes now appear during SEARCH/approach correction before INSERT. The next milestone must perform lateral alignment above the workpiece, prove no-contact XY convergence, and only then descend toward the hole.
