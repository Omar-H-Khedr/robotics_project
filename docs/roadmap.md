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

Baseline v2.0 status: peg/hole-specific insertion validation instrumentation is added in a separate world and launch path. It preserves the v1.8 low-force segmented robot contact validation flow while adding peg and hole contact topics, insertion metrics, a conservative validation sequence, and trial-summary fields. `insertion_success` remains `null` until a validated depth/alignment rule exists.

Baseline v2.4 status: coordinate-based insertion diagnostics now publish explicit object frames before planning. Cartesian target definitions live in `kuka_task_control/config/peg_hole_cartesian_targets.yaml`; `peg_hole_frame_publisher` publishes named `world` TF frames for the hole, pre-insertion, touch, hold, final insertion, and insertion-axis marker targets; and `cartesian_insertion_diagnostics` resolves targets from TF first with YAML fallback. No trajectories are sent. IK and MoveIt-based motion generation remain future work after frame validation.

Baseline v2.5 status: IK feasibility diagnostics are added as a diagnostic-only layer before motion. `ik_feasibility_diagnostics` reads the v2.4 TF target frames, `/joint_states`, current `tool0`, and `base_link`, then reports conservative radial workspace feasibility on `/ik_feasibility_diagnostics`. It detects visible MoveIt/IK services but does not call an IK solver, send trajectory goals, or execute robot motion.

Baseline v2.5c status: execution gates are unified in
`execution_gate_monitor`, which publishes `/execution_gate_status` from
Cartesian geometry, IK diagnostics, safety status, and optional insertion
metrics. `tool_axis_audit` compares the six local `tool0` axes against the world
insertion axis and reports the best candidate, but never auto-validates it.
Controller execution remains blocked until Cartesian geometry, IK availability,
real IK solutions, manual tool-axis validation, safety guard, and force/contact
guard all pass.

Baseline v2.5d status: `cartesian_orientation_target_calculator` computes
diagnostic desired world-frame orientation quaternions for the insertion-aligned
Cartesian targets. The selected candidate is `tool0_+Z`, aligned to the world
insertion axis `[0.0, 0.0, -1.0]`, with current tool yaw used as the reference
when resolvable. The node publishes `/cartesian_orientation_targets` and keeps
`orientation_validated=false` and `motion_execution_allowed=false`; IK and a
dry-run joint plan are still required before any controller execution can be
considered.

Baseline v2.5f status: the full-pose waypoint policy now covers every planned
Cartesian waypoint, including `staging_pose`. `staging_pose` uses the same
`align_tool_axis_to_insertion_axis` policy as the insertion and retreat
waypoints so the tool is oriented before lateral alignment near the hole.
This remains diagnostic-only: no controller motion is allowed without a real IK
solver, real IK solutions for every waypoint, explicit orientation validation,
and active safety and force/contact gates.

Baseline v2.6 status: `cartesian_insertion_dry_run_planner` assembles the full
Cartesian insertion waypoint sequence from the current tool pose through
staging, axis alignment, touch, hold, final insertion, and retreat. The plan is
published on `/cartesian_insertion_dry_run_plan` as diagnostics only with
`motion_execution_enabled=false`, `trajectory_execution_requested=false`, and
`controller_execution_allowed=false`. It remains non-executable until real IK
solutions exist for all planned waypoints and the execution gates are available.
No controller command is sent.

Baseline v2.7 status: `ik_backend_audit` publishes `/ik_backend_audit` as a
diagnostic decision report for available IK infrastructure. It checks visible
`compute_ik` and MoveIt-style services, package availability through
`ament_index_python`, robot model resources, joint names, joint-limits file
readability, KUKA LBR iisy URDF/xacro discovery, and observed project readiness
from the v2.6 dry-run plan and execution gates. It does not solve IK, call
motion execution, send trajectory goals, install packages, or run Gazebo.
Controller execution remains blocked while the report decides between using a
MoveIt `/compute_ik` backend, configuring MoveIt, or adding a custom IK service.

Baseline v2.8 status: `moveit_config_audit` publishes `/moveit_config_audit`
as a diagnostic-only readiness report for KUKA LBR iisy MoveIt configuration.
It searches installed and source package shares for likely MoveIt config
packages, SRDF files, `kinematics.yaml`, joint-limits, OMPL planning config,
and move-group launch resources. MoveIt packages may be present while
`/compute_ik` is still absent, so the report keeps
`moveit_ready_for_compute_ik=false` until the config is confirmed and the
service is actually visible. The optional
`run_moveit_ik_diagnostic.launch.py` launch starts only the audit nodes and does
not launch `move_group`, `task_trajectory_executor`, Gazebo, or any controller
client.

Baseline v2.9 status: `moveit_launch_readiness_audit` publishes
`/moveit_launch_readiness_audit` as a diagnostic-only gate before any future
MoveIt/move_group IK launch. It requires an exact `lbr_iisy6_r1300` semantic
model before `moveit_launch_ready` can become true. If only the currently known
SRDF variants are present, it recommends
`create_or_select_matching_srdf_for_lbr_iisy6_r1300` and keeps `selected_srdf`
null. The v2.9 `run_move_group_ik_diagnostic.launch.py` starts only
`moveit_launch_readiness_audit`, `moveit_config_audit`, and `ik_backend_audit`;
it does not launch `move_group`, `task_trajectory_executor`, Gazebo, or any
controller client.

Baseline v2.10 status: a project-local semantic candidate for
`lbr_iisy6_r1300` exists under
`kuka_task_control/config/moveit_lbr_iisy6_r1300`. It is derived from the
same-family `lbr_iisy11_r1300_arm.srdf.xacro` template, selected by the MoveIt
audits as `project_local_lbr_iisy6_r1300_overlay`, and marked
`semantic_model_validation_status="candidate_requires_validation"`. The new
`semantic_model_validator` publishes `/semantic_model_validation` and always
keeps `approved_for_motion=false`, `controller_motion_allowed=false`, and
`trajectory_execution_allowed=false`.

Baseline v2.11 status: `robot_description_semantic_diagnostics` publishes
`/robot_description_semantic_diagnostics` from the project-local or installed
`lbr_iisy6_r1300.srdf` candidate. `moveit_launch_readiness_audit` now reports
the semantic candidate source, candidate availability, semantic diagnostics
availability, and semantic diagnostics status. A structurally valid SRDF still
keeps `moveit_launch_ready=false` while tool-link validation is required, with
`recommended_next_step="validate_tool_link_and_prepare_move_group_diagnostic_launch"`.
No `move_group` launch, `/compute_ik` call, trajectory goal, or controller
motion is enabled.

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

Near-term follow-up after v2.0: validate a real insertion-depth signal from geometry, TF, or Gazebo state before promoting `insertion_success` from `null` to a binary outcome.

Near-term follow-up after v2.11: validate the project-local LBR iisy 6 R1300
semantic candidate against the exact URDF, including tool link and collision
matrix assumptions, before preparing an explicitly guarded diagnostic
`move_group` launch.
Keep trajectory execution disabled and continue to block controller motion
until `/compute_ik` is available and only no-motion IK service tests have
passed.

Near-term follow-up after v2.7: act on the `/ik_backend_audit` decision report.
If a real `/compute_ik` service is present, add diagnostic IK requests for the
full-pose waypoints. If MoveIt resources exist but no service is running,
configure and launch MoveIt. If no backend exists, add MoveIt integration or a
custom IK service. Do not enable controller execution until real joint
solutions exist for every waypoint and all gates pass.
