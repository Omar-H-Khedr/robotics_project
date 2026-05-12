# Experiment Plan

## Purpose

The experiment plan defines staged demonstrations for a PhD proposal. Each stage should produce evidence that the project is technically grounded, measurable, and feasible with one simulated KUKA LBR iisy robot.

## Milestone 01: Simulation Baseline

Folder: `experiments/milestone_01_sim_baseline`

Question:

Can the project run a reproducible ROS 2 simulation of the KUKA LBR iisy robot and collect baseline motion data?

Initial tasks:

- Launch the robot in Gazebo.
- Execute a simple free-space joint or Cartesian motion.
- Record joint state, command, and task status.
- Verify limits and basic controller behavior.

Success criteria:

- The simulation launches from a documented command.
- The robot completes a simple motion without limit violations.
- Logs are saved in a repeatable structure.
- The run can be used as a baseline for later payload/contact comparisons.

## Milestone 02: Contact Adaptation

Folder: `experiments/milestone_02_contact_adaptation`

Question:

Can context-based adaptation reduce failures caused by changing payload or contact conditions compared with a fixed controller?

Initial tasks:

- Define payload variants such as nominal, light, and heavy.
- Define contact variants such as no contact, soft surface, stiff surface, and friction variation.
- Run a fixed-controller baseline across variants.
- Add context estimation or simulator-provided context labels for early tests.
- Adapt controller parameters or references based on context.

Metrics:

- Task success rate.
- Tracking error.
- Contact force peak and duration.
- Oscillation or instability indicators.
- Number and type of safety-layer interventions.

Success criteria:

- The fixed controller shows measurable degradation under at least one changed condition.
- The context-adapted controller improves at least one metric without violating safety constraints.
- The experiment is simple enough to explain in a proposal presentation.

## Milestone 03: Meta-RL

Folder: `experiments/milestone_03_meta_rl`

Question:

Can a learning-based policy adapt faster or more robustly across payload/contact variations when given context information and constrained by a safety layer?

Initial tasks:

- Define a small family of simulation tasks.
- Expose observations, actions, rewards, reset conditions, and termination conditions through `kuka_rl_env`.
- Train or evaluate policies across randomized payload/contact parameters.
- Compare against fixed and context-adapted non-RL baselines.

Candidate observations:

- Joint position, velocity, and effort.
- End-effector pose and velocity.
- Task error.
- Contact estimate or force signal.
- Context vector from `kuka_context_adapter`.

Candidate actions:

- Bounded joint velocity targets.
- Cartesian target offsets.
- Impedance or admittance parameter adjustments.
- Residual commands added to a stable baseline controller.

Safety requirements:

- All learned actions must pass through `kuka_safety_layer`.
- Unsafe actions should be clipped, rejected, or replaced with a safe fallback.
- Safety interventions must be logged and included in evaluation.

## Logging and Reproducibility

Each experiment folder should eventually include:

- A README with the purpose and run command.
- Configuration files for scenario parameters.
- A place for scripts or launch files that reproduce the run.
- A clear log naming convention.
- A short result summary after each completed experiment.

Raw large logs should not be committed unless they are intentionally small example artifacts.

