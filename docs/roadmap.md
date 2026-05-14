# Roadmap

## Phase 1: KUKA Gazebo Baseline and Trajectory Control

- Confirm KUKA Gazebo simulation startup.
- Confirm `joint_trajectory_controller` accepts commands for `joint_1` through `joint_6`.
- Define a minimal repeatable baseline trajectory.
- Record baseline controller feedback and execution timing.

## Phase 2: Peg-in-Hole Scene and Task Definition

- Create peg, hole, fixture, and workcell descriptions.
- Define task frames, insertion axis, nominal clearance, and initial offsets.
- Add Gazebo world files for baseline and variant scenes.
- Validate that task geometry and collision properties are versioned and reproducible.

## Phase 3: Safety Layer and Filtered Command Interface

- Define the proposed-command and filtered-command topics.
- Add joint, velocity, workspace, and task constraints.
- Add collision/contact and force placeholders for later instrumentation.
- Log safety decisions and violation events for every trial.

## Phase 4: Experiment Manager and Reproducible Trials

- Define trial manifests, parameter sweeps, seeds, and metadata.
- Add trial start, stop, reset, timeout, and result labeling.
- Record rosbag data and structured summaries.
- Produce repeatable baseline experiments over N trials.

## Phase 5: RGB-D/Perception Pipeline in Gazebo

- Add simulated RGB-D camera configuration.
- Define camera frames, topics, and calibration assumptions.
- Publish task-state estimates for peg and hole pose.
- Compare perception estimates against Gazebo ground truth.

## Phase 6: Learning/RL Interface

- Define observations, actions, rewards, resets, and done conditions.
- Route learning-generated actions through `safety_layer`.
- Support offline dataset generation from baseline and safety-filtered trials.
- Evaluate fixed policies under the same experiment manager.

## Phase 7: Evaluation Metrics and Publication Experiments

- Finalize metrics for success, insertion, collision, contact, safety, timing, and repeatability.
- Run ablation studies for baseline control, safety-filtered control, perception-enabled control, and learning-enabled control.
- Generate publication-ready logs, plots, tables, and experiment manifests.
- Document simulator assumptions and limitations.
