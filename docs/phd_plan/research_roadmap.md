# Research Roadmap

## Research Objective

Develop a ROS 2 and Gazebo-based KUKA robot simulation framework for studying
contact-rich manipulation and adaptive control. The project will begin with
repeatable simulated baselines, then add contact tasks, structured data
collection, and AI-based adaptation methods that can improve behavior under
uncertainty, contact variation, and changing task conditions.

## Simulation Pipeline

The simulation pipeline should provide a reproducible path from robot model to
experiment result:

- Use ROS 2 as the integration layer for launch files, controllers, nodes, and
  experiment orchestration.
- Use Gazebo for KUKA robot simulation, environment models, contact dynamics,
  and repeatable task scenarios.
- Keep robot descriptions, controller configuration, task definitions, and data
  logging configuration separate enough to support controlled experiments.
- Track each experiment with explicit parameters, launched components, recorded
  topics, and task outcome metadata.

## Baseline Control Stage

The first research stage should establish reliable non-learning baselines:

- Bring up a KUKA robot model in Gazebo through ROS 2 launch files.
- Validate joint state publishing, command interfaces, transforms, and basic
  controller behavior.
- Implement simple position and trajectory control experiments.
- Add baseline task metrics such as tracking error, completion time, command
  smoothness, and failure cases.
- Keep this stage deterministic enough to compare against later adaptation
  methods.

## Contact-Rich Task Stage

After baseline motion is stable, the project should introduce tasks where contact
is central to success:

- Define controlled contact scenarios such as surface following, pushing,
  constrained sliding, insertion-like tasks, and compliant interaction.
- Model task objects and contact surfaces in Gazebo with documented physical
  assumptions.
- Record contact state, robot state, commanded motion, and task outcome signals.
- Evaluate how baseline controllers behave when contact forces, friction,
  geometry, and initial conditions vary.

## Data Collection Stage

Data collection should be designed before learning methods are added:

- Define a consistent experiment run format with configuration, logs, metadata,
  and results.
- Record ROS 2 topics for joint states, commands, transforms, contact signals,
  simulation clock, and task-specific success or failure labels.
- Store enough metadata to reproduce each run, including robot model, world,
  controller configuration, task parameters, and random seeds when used.
- Separate raw logs from processed datasets so later analysis remains traceable.
- Build small tools in `scripts/kuka_tools/` for launching, checking, indexing,
  and summarizing runs.

## Learning and Adaptation Stage

The learning stage should build on the baseline and data pipeline rather than
replace them:

- Use collected simulation data to identify failure modes and adaptation targets.
- Explore AI-based adaptation for controller parameters, contact strategy,
  trajectory correction, or residual policies.
- Compare learned or adaptive behavior against the deterministic baseline using
  the same tasks and metrics.
- Keep adaptation methods modular so they can be tested offline first, then
  integrated into ROS 2 control loops when appropriate.
- Prioritize methods that remain interpretable, measurable, and compatible with
  future transfer from simulation toward physical KUKA hardware.
