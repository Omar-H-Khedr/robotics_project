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
- Launch the KUKA robot and peg-in-hole task scene in one Gazebo simulation through `thesis_bringup`.
- Validate that task geometry and collision properties are versioned and reproducible.

Phase 2B status: `thesis_bringup/launch/research_baseline.launch.py` now resolves `peg_in_hole_description/worlds/peg_in_hole_world.sdf`, exports the task model path for Gazebo, and passes the world into `kuka_gazebo/gazebo_startup.launch.py` so the existing robot spawn, bridge, and controller spawners are reused.

## Phase 3: Safety Layer and Filtered Command Interface

- Define the proposed-command and filtered-command topics.
- Add joint, velocity, workspace, and task constraints.
- Add collision/contact and force placeholders for later instrumentation.
- Log safety decisions and violation events for every trial.

Baseline v0.1 status: monitor-only safety is implemented through `safety_monitor`. It checks joint soft limits, NaN/Inf values, missing `/joint_states`, and phase-duration timeout placeholders. It does not yet stop motion or perform force control.

## Phase 4: Experiment Manager and Reproducible Trials

- Define trial manifests, parameter sweeps, seeds, and metadata.
- Add trial start, stop, reset, timeout, and result labeling.
- Record rosbag data and structured summaries.
- Produce repeatable baseline experiments over N trials.

Baseline v0.1 status: `baseline_trial_manager` records metadata, `/joint_states`, `/task_phase`, `/safety_status`, and a summary JSON under `results/baseline_trials/`. Contact metrics and success labeling remain explicit placeholders.

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
