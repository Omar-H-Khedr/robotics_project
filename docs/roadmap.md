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

Baseline v0.3 status: contact sensors and contact metric logging are added as instrumentation. The real bridged contact topics are `/gazebo/contacts/peg`, `/gazebo/contacts/hole`, and `/gazebo/contacts/target`. They do not change safety behavior or robot control.

Baseline v0.5 status: validated contact force extraction is implemented for bridged `ros_gz_interfaces/msg/Contacts` messages. `max_contact_force` is computed from `Contacts.wrenches` force-vector magnitudes and was validated first in the minimal passive contact world, where a 0.1 kg probe produces approximately 0.981 N under gravity. This remains metrics instrumentation only and does not modify KUKA task control.

## Phase 4: Experiment Manager and Reproducible Trials

- Define trial manifests, parameter sweeps, seeds, and metadata.
- Add trial start, stop, reset, timeout, and result labeling.
- Record rosbag data and structured summaries.
- Produce repeatable baseline experiments over N trials.

Baseline v0.1 status: `baseline_trial_manager` records metadata, `/joint_states`, `/task_phase`, `/safety_status`, and a summary JSON under `results/baseline_trials/`. Contact metrics and success labeling remain explicit placeholders.

Baseline v0.3 status: `baseline_trial_manager` also subscribes to `/contact_event` and `/insertion_metrics`, writes `contact_events.csv`, and includes contact metrics in `trial_summary.json`. `contact_metrics_available` means at least one real Gazebo contact topic message was observed; `contact_events_count` can remain zero when those messages report no physical contacts. `safe_success` remains `task_completed == true AND safety_violations_count == 0`.

Baseline v0.5 status: `contact_events.csv` records event time, phase, source, contact count, extracted maximum force, and message text. `trial_summary.json` includes `max_contact_force`, `force_extraction_available`, `force_extraction_method`, `contact_events_count`, `contact_messages_observed`, `physical_contact_observed`, `contact_topics_connected`, and `contact_topics_seen`. Minimal contact validation trials remain valid even when no task sequence is running.

Trial workflow status: `thesis_bringup/launch/run_full_research_trial.launch.py`
is now the recommended single-command entry point for full baseline trials. It
includes the existing `run_research_trial.launch.py` baseline, waits for
`/joint_trajectory_controller/follow_joint_trajectory` through
`controller_readiness_gate`, then starts `task_trajectory_executor`
automatically. The old two-terminal workflow remains available for debugging.

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

Near-term follow-up after v0.5: define a defensible insertion-success rule from peg/hole pose, insertion depth, contact state, or a documented combination of those signals. Until then, `task_completed`, `insertion_hold_reached`, heuristic `insertion_success_estimate`, and true `insertion_success` remain separate metrics.
