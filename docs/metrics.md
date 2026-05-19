# Initial Research Metrics

This file defines the initial metrics for KUKA peg-in-hole assembly experiments. Metric definitions should become stricter as the simulator, contact instrumentation, safety layer, and experiment manager mature.

## Task Success

Binary trial-level outcome indicating whether the full assembly task completed within the trial constraints.

Initial definition: the robot completes the planned insertion procedure without timeout, fatal controller error, or unrecoverable safety stop.

## Insertion Success

Binary trial-level outcome indicating whether the peg reached the target insertion depth and final alignment tolerance.

Initial definition: the peg tip reaches the configured insertion depth along the task insertion axis while remaining within the configured lateral and angular tolerance.

Research Baseline v0.3 status: `insertion_success` remains `null`. Contact observation and task phase progress are not sufficient proof of insertion depth or alignment. `insertion_success_estimate` is a heuristic that can be true only when `insertion_hold_reached` is true, `trial_status` is `completed`, and no explicit failure status was observed.

Research Baseline v2.0-v2.2 status: `insertion_success` remains `null` for peg/hole insertion validation until a validated depth and alignment rule is implemented. v2.0 validated the instrumentation path. v2.1 corrects contact semantics so `insertion_success_estimate` may become true only when the insertion hold phase is reached, an actual peg-hole collision pair is observed, no force threshold violation is present, and the final trial status is `completed` or the accepted guarded-contact stop status. v2.2 cleans the initial world state so the peg/hole validation scene should start without uninitialized physical contact. v2.2b uses a suspended/static peg to avoid peg-table contact during instrumentation validation; this is not final grasped peg insertion. Later versions will attach the peg to, or otherwise represent it at, the robot/tool side.

Research Baseline v2.4 status: `insertion_success` remains `null`. v2.4 adds explicit object-frame TF publication for coordinate-based diagnostics: Cartesian target poses, insertion axis marker, current tool pose if available from TF, and distances from `tool0` to `hole_center` and the configured staging/pre-insertion waypoint. These diagnostics are planning prerequisites, not proof of insertion and not robot motion.

Research Baseline v2.5f status: `staging_pose` is now a full-pose diagnostic
waypoint with an orientation target, but `insertion_success` remains `null`.
Full-pose target availability is a planning prerequisite only; it does not prove
insertion depth, contact state, or successful assembly.

Research Baseline v2.6 status: the Cartesian dry-run plan is assembled and
reported, but `insertion_success` remains `null`. A non-executable dry-run plan
is still a planning diagnostic, not evidence of insertion depth, contact state,
or assembly success.

Research Baseline v2.7 status: the IK backend audit reports available IK
infrastructure and a recommended next step, but `insertion_success` remains
`null`. Backend availability or configuration readiness is not an IK solution,
not a trajectory, and not evidence of insertion depth, contact state, or
assembly success.

## Collision Events

Count of collision or contact events that are outside the expected peg-hole interaction.

Initial definition: number of unexpected contacts reported during a trial. The exact Gazebo contact topic and filtering rule will be finalized when contact instrumentation is added.

Research Baseline v0.3 status: Gazebo contact sensors are configured for `/gazebo/contacts/peg`, `/gazebo/contacts/hole`, and `/gazebo/contacts/target`. The metrics node publishes `/contact_event` JSON and the experiment manager records `contact_events.csv`. `contact_metrics_available` means contact instrumentation is connected: at least one configured ROS contact topic has a visible ROS publisher. `contact_messages_observed` means at least one ROS `Contacts` message was received. `physical_contact_observed` means at least one received message contained `contact_count > 0`. `contact_events_count` is the number of positive physical contact events. It can remain zero if the scripted trajectory completes without physically touching the peg, hole, or target contact sensors.

Research Baseline v0.5 status: `contact_events_count` is counted only for positive physical contact events where `contact_count > 0`. Continuous contact is rate controlled with start and update events so `/contact_event` does not flood while still recording meaningful positive contact observations.

Zero contact events are expected for the current scripted joint-space sequence unless that sequence physically touches instrumented objects.

## Maximum Contact Force Placeholder

Maximum measured or estimated contact force during a trial.

Initial status: placeholder metric. The value should remain explicitly marked as unavailable until contact wrench estimation or Gazebo contact-force extraction is implemented and validated.

Research Baseline v0.3 status: `max_contact_force` remains `null` by default because force extraction is disabled/unvalidated. The metrics node does not fake force values.

Research Baseline v0.5 status: `max_contact_force` is extracted from `ros_gz_interfaces/msg/Contacts.wrenches` force vectors. For each available `body_1_wrench.force` and `body_2_wrench.force`, the node computes `sqrt(x^2 + y^2 + z^2)` and tracks the maximum across all contacts and wrenches. If no force vector exists in a message, the value remains `null`; no force values are faked. `force_extraction_available` becomes true after a valid force vector is parsed, and `force_extraction_method` is `ros_gz_interfaces Contacts.wrenches force magnitude`.

## Trajectory Execution Time

Elapsed time from accepted trajectory start to trajectory completion, abort, timeout, or safety stop.

Initial definition: wall-clock ROS time between trial command start and terminal trial state.

## Safety Violations

Count and type of safety constraints violated during a trial.

Initial definition: number of safety events reported by `safety_layer`, grouped by constraint category such as joint limit, velocity limit, workspace limit, contact limit, or command validity.

## Repeatability Over N Trials

Consistency of outcomes and trajectories over a fixed number of repeated trials under the same configuration.

Initial definition: report success statistics, execution-time statistics, final pose error statistics, and safety-violation statistics over N repeated trials.

## Safe Success Rate

Fraction of trials that succeed without safety violations.

Initial definition:

```text
safe_success_rate = successful_trials_without_safety_violations / total_trials
```

This metric should be reported with the trial count, scene configuration, controller configuration, and safety-layer configuration.

## Baseline v0.1 Logged Fields

Research Baseline v0.1 writes the following trial summary fields:

- `task_success`: placeholder, not inferred yet.
- `insertion_success`: placeholder, not inferred yet.
- `collision_events`: placeholder until Gazebo contact filtering is added.
- `max_contact_force`: placeholder until contact-force extraction is validated.
- `safety_violations`: counted from `/safety_status` messages whose level is `VIOLATION`.
- `execution_time_sec`: elapsed logger runtime for the trial process.
- `safe_success`: placeholder until task success and safety violation semantics are combined.

## Baseline v0.3 Contact Fields

Research Baseline v0.3 preserves the v0.2 summary fields and adds:

- `contact_events_count`: number of non-empty contact observations reported through `/contact_event`.
- `max_contact_force`: maximum validated contact force, or `null` when unavailable.
- `insertion_attempted`: true once an insertion phase is observed.
- `insertion_hold_reached`: true once `insertion_hold` is observed.
- `insertion_success`: true/false only after a validated rule exists; currently `null`.
- `insertion_success_estimate`: heuristic based on insertion hold, completed trial status, and absence of explicit failure.
- `contact_topics_configured`: configured contact source names and ROS topic names.
- `contact_topics_connected`: configured contact sources whose ROS topics currently have publishers.
- `contact_messages_observed`: true once at least one ROS `Contacts` message callback is received.
- `physical_contact_observed`: true once at least one received contact message has `contact_count > 0`.
- `contact_metrics_available`: true when at least one configured ROS contact topic has a visible publisher.
- `contact_topics_seen`: contact sources that have produced at least one message.
- `notes`: explanation of unavailable contact force or success semantics.

`task_completed`, `insertion_hold_reached`, `insertion_success_estimate`, and true `insertion_success` are intentionally separate. A trial can complete the scripted trajectory and reach insertion hold without proving the peg reached a validated insertion depth or alignment tolerance.

## Baseline v0.5 Contact Force Fields

Research Baseline v0.5 preserves the v0.3 contact fields and adds validated force extraction:

- `max_contact_force`: maximum extracted contact force magnitude in newtons, or `null` until a force vector is observed.
- `force_extraction_available`: true once a valid force vector has been extracted.
- `force_extraction_method`: extraction method string for reproducibility.
- `contact_events.csv`: includes `ros_time_sec`, `phase`, `source`, `contact_count`, `max_contact_force`, and `message` for each logged contact event.

The minimal contact validation world is the validation source for v0.5. A 0.1 kg passive probe should report approximately `0.1 * 9.81 = 0.981 N`, matching the observed Gazebo contact wrench before applying the same extraction path to KUKA insertion experiments.

## Baseline v2.0-v2.2 Peg/Hole Insertion Fields

Research Baseline v2.0 preserves existing contact and force fields and adds
peg/hole instrumentation. Research Baseline v2.1 tightens these fields so broad
peg contact is not treated as insertion contact. Research Baseline v2.2 adds
explicit initial-contact accounting. v2.2b cleans the validation world with a
suspended/static peg so instrumentation validation starts from zero physical
contact:

- `peg_contact_observed`: true once any collision pair includes the peg.
- `hole_contact_observed`: true once any collision pair includes the hole or hole block.
- `peg_table_contact_observed`: true when a peg collision pair includes the work table or table collision.
- `peg_table_contact_count`: count of observed peg-table collision-pair classifications.
- `peg_hole_contact_observed`: true only when the same collision pair contains one peg collision and one hole or hole-block collision.
- `peg_hole_contact_count`: count of observed peg-hole collision-pair classifications.
- `first_peg_hole_contact_phase`: first task phase where a peg-hole collision pair was observed, or `null`.
- `first_peg_table_contact_phase`: first task phase where a peg-table collision pair was observed, or `null`.
- `peg_hole_collision_pairs`: unique collision-pair diagnostics classified as insertion contact.
- `non_insertion_contact_pairs`: unique collision-pair diagnostics that were not peg-hole insertion contact, including peg-table contact.
- `initial_contact_detected`: true when physical contact is observed while the task phase is still `uninitialized`.
- `initial_contact_pairs`: unique collision-pair diagnostics observed during the `uninitialized` phase.
- `uninitialized_contact_count`: count of collision-pair classifications observed before task execution.
- `clean_initial_state`: true when no contact was observed during the `uninitialized` phase.
- `max_peg_contact_force`: maximum extracted force on peg contact topics, or `null`.
- `max_hole_contact_force`: maximum extracted force on hole contact topics, or `null`.
- `insertion_depth_available`: false until a validated geometry or TF depth source exists.
- `insertion_depth_estimate`: null unless `insertion_depth_available=true`.
- `peg_hole_instrumentation_success`: true for peg/hole insertion validation when contact topics are connected, `/insertion_metrics` is received, the trial summary is generated, and no safety violations are recorded.
- `clean_scene_success`: true for peg/hole insertion validation only when `clean_initial_state=true`, no safety violations are recorded, contact topics are connected, and `/insertion_metrics` is received.

These fields validate instrumentation and logging first. They do not prove final insertion success. Peg-table contact is a non-insertion contact: it can set `peg_contact_observed=true` and `peg_table_contact_observed=true`, but it must leave `peg_hole_contact_observed=false` and `insertion_success_estimate=false`. Contact during `uninitialized` is not insertion contact; a valid insertion-contact trial should not start with `initial_contact_detected=true`.

## Baseline v2.4 Cartesian Diagnostic Fields

The `cartesian_insertion_diagnostics` node publishes JSON on
`/cartesian_insertion_diagnostics` with:

- `status`: always `diagnostic_only_no_motion`.
- `current_tool_pose_world`: current `tool0` pose in `world` when TF is available.
- `current_tool_pose_base`: current `tool0` pose in `base_link` when TF is available.
- `available_object_frames_world`: resolved object target TF frames.
- `frame_source`: `tf` or `yaml_fallback` for each object pose target.
- `hole_center_world`: hole-center Cartesian target from TF or YAML fallback.
- `pre_insertion_pose_world`: pre-insertion Cartesian target from TF or YAML fallback.
- `insertion_touch_pose_world`: touch pose target from TF or YAML fallback.
- `insertion_hold_pose_world`: hold pose target from TF or YAML fallback.
- `final_insertion_pose_world`: final insertion target from TF or YAML fallback.
- `insertion_axis_world`: configured insertion-axis direction.
- `distance_tool_to_hole`: Euclidean distance from current tool pose to hole center when available.
- `distance_tool_to_pre_insertion`: Euclidean distance from current tool pose to pre-insertion pose when available.

The `peg_hole_frame_publisher` node publishes JSON on `/peg_hole_frame_status`
with `status=object_frames_published`, `world_frame`, `published_frames`, and
`target_count`.

These fields are frame-validation diagnostics only. They do not command motion,
infer insertion depth, or mark task success.

## Baseline v2.5 IK Feasibility Diagnostic Fields

The `ik_feasibility_diagnostics` node publishes JSON on
`/ik_feasibility_diagnostics` with:

- `status`: always `ik_feasibility_diagnostic_only_no_motion`.
- `current_joint_names` and `current_joint_positions`: latest `/joint_states`
  values observed by the diagnostic node.
- `current_tool_pose_world`: current `tool0` pose in `world` when TF is
  available.
- `current_tool_pose_base`: current `tool0` pose in `base_link` when TF is
  available.
- `object_frames_used`: target frames evaluated by the diagnostic layer.
- `targets`: per-target diagnostics for `hole_center`, `pre_insertion_pose`,
  `insertion_touch_pose`, `insertion_hold_pose`, and `final_insertion_pose`.
- `target_pose_world` and `target_pose_base`: resolved TF pose for each target.
- `translational_distance_from_current_tool`: Euclidean distance from current
  tool pose to each target.
- `z_offset_from_hole_center`: target z offset relative to `hole_center`.
- `approximate_workspace_feasible`: conservative radial workspace-envelope
  result, not a solved IK result.
- `requires_ik_solver`: always true for Cartesian target execution.
- `ik_solver_available`: true only when a visible `compute_ik`/MoveIt-style
  service is detected.
- `ik_solution_available`: `null` in v2.5 because no IK solver is called.
- `feasibility_status`: diagnostic status string distinguishing geometric
  infeasibility, geometric feasibility without a called IK solver, and future
  IK-solver outcomes.
- `all_targets_geometrically_feasible`: true only when all evaluated targets are
  inside the configured approximate workspace envelope.
- `motion_execution_enabled`: always false.

These fields are planning diagnostics only. They do not prove insertion success,
do not execute trajectories, and must not be used as contact validation.

## Baseline v2.5c Execution Gate and Tool-Axis Fields

The `execution_gate_monitor` node publishes JSON on `/execution_gate_status`
with:

- `status`: always `execution_gates_diagnostic_only_no_motion`.
- `motion_execution_enabled`: always false in v2.5c.
- `trajectory_execution_requested`: always false in v2.5c.
- `geometry_valid`: copied from
  `/cartesian_insertion_diagnostics.cartesian_geometry_valid`.
- `geometry_source`: the source used for the geometry gate.
- `ik_available`: copied from IK solver detection.
- `ik_solution_available`: true only after real IK solutions exist for all
  targets.
- `all_targets_geometrically_feasible`: approximate workspace diagnostic from
  IK feasibility.
- `tool_axis_orientation_validated`: false until manually validated.
- `safety_guard_active`: true only after an observed OK `/safety_status`.
- `force_guard_active`: false unless explicitly reported active.
- `contact_metrics_available`: copied from `/insertion_metrics` when available.
- `controller_execution_allowed`: true only when all required gates pass.
- `block_reasons` and `primary_block_reason`: explicit reasons motion is
  blocked.

The `tool_axis_audit` node publishes JSON on `/tool_axis_audit` with the world
directions of `tool0` `+X`, `-X`, `+Y`, `-Y`, `+Z`, and `-Z`, alignment scores
against `[0.0, 0.0, -1.0]`, and the best candidate tool axis. It always reports
`orientation_validated=false` until a human explicitly validates the insertion
axis.

## Baseline v2.5d Orientation Target Fields

The `cartesian_orientation_target_calculator` node publishes JSON on
`/cartesian_orientation_targets` with:

- `status`: always `orientation_targets_diagnostic_only_no_motion`.
- `selected_tool_axis_candidate`: configured candidate axis, currently
  `tool0_+Z`.
- `insertion_axis_world`: configured world insertion axis, currently
  `[0.0, 0.0, -1.0]`.
- `current_tool_orientation_world`: current `tool0` quaternion from TF when
  available.
- `current_tool_axes_world`: current world directions of the six local `tool0`
  axes.
- `desired_orientations_world`: desired quaternions for `staging_pose`,
  `axis_align_pose`, `insertion_touch_pose`,
  `insertion_hold_pose`, `final_insertion_pose`, and `retreat_pose`.
- `expected_alignment_after_orientation`: predicted selected-axis alignment
  after applying the computed orientation.
- `yaw_reference_mode`: configured yaw policy.
- `yaw_reference_unresolved`: true when yaw about the insertion axis cannot use
  the current tool orientation as a reference.
- `orientation_targets_available`: true when target orientations were computed.
- `orientation_validated`: always false in v2.5d.
- `motion_execution_allowed`: always false in v2.5d.
- `validation_reason`: explanation that the orientation target is computed but
  not validated by IK or motion.

The `execution_gate_monitor` includes orientation target availability and the
selected tool-axis candidate in `/execution_gate_status`, but
`controller_execution_allowed` remains false until explicit IK and dry-run plan
validation exists.

## Baseline v2.5f Full-Pose Waypoint Fields

In v2.5f, the planned full-pose waypoint set is `staging_pose`,
`axis_align_pose`, `insertion_touch_pose`, `insertion_hold_pose`,
`final_insertion_pose`, and `retreat_pose`.

For each planned waypoint, `/cartesian_orientation_targets` reports
`orientation_target_available=true` and
`orientation_source="cartesian_orientation_targets"` when the target
orientation calculation succeeds. `ik_feasibility_diagnostics` then reports
`full_pose_feasibility_status="full_pose_ready_but_no_ik_solver"` for
full-pose waypoints when no IK solver is available.

`execution_gate_monitor.full_pose_targets_available` is true only when all
planned waypoints have both position and orientation targets. It still keeps
`controller_execution_allowed=false` without a real IK solver, real IK
solutions, explicit orientation validation, and an active force/contact guard.

## Baseline v2.6 Cartesian Dry-Run Plan Fields

The `cartesian_insertion_dry_run_planner` node publishes JSON on
`/cartesian_insertion_dry_run_plan` with:

- `status`: always `cartesian_dry_run_no_motion`.
- `motion_execution_enabled`: always false.
- `trajectory_execution_requested`: always false.
- `controller_execution_allowed`: always false.
- `waypoint_order`: `current_tool_pose`, `staging_pose`, `axis_align_pose`,
  `insertion_touch_pose`, `insertion_hold_pose`, `final_insertion_pose`, and
  `retreat_pose`.
- `waypoints`: per-waypoint pose, source, distance, approximate workspace,
  orientation target, IK availability, IK solution, joint solution, and
  executability diagnostics.
- `joint_solution`: null unless a real IK result is present in the IK
  diagnostics.
- `all_waypoints_have_full_pose`: true only when every waypoint has position and
  orientation data.
- `all_waypoints_geometrically_feasible`: true only when Cartesian geometry and
  approximate workspace diagnostics pass.
- `all_waypoints_have_ik_solution`: true only when real IK solutions exist for
  all planned Cartesian waypoints.
- `plan_executable`: true only when full poses, geometry, and real IK solutions
  are all available.
- `block_reasons` and `primary_block_reason`: explicit no-motion block state.

The execution gate monitor now also reports `dry_run_plan_available`,
`dry_run_plan_executable`, and `dry_run_primary_block_reason`. Controller
execution remains disabled in the diagnostic launch, and no controller command
is sent.

## Baseline v2.7 IK Backend Audit Fields

The `ik_backend_audit` node publishes JSON on `/ik_backend_audit` with:

- `status`: always `ik_backend_audit_diagnostic_only_no_motion`.
- `motion_execution_enabled`: always false.
- `trajectory_execution_requested`: always false.
- `controller_motion_allowed`: always false.
- `services`: visible `/compute_ik`, `compute_ik`-like, and MoveIt planning
  service diagnostics.
- `packages`: availability of `moveit_ros_move_group`, `moveit_msgs`,
  `moveit_kinematics`, `trac_ik_kinematics_plugin`, `kdl_parser_py`, and
  `urdf_parser_py` through `ament_index_python`.
- `robot_model_resources`: `robot_description` visibility, `/joint_states`
  joint names, joint-limits file availability/readability, and KUKA LBR iisy
  URDF/xacro discovery from package share folders.
- `existing_project_ik_readiness`: observed v2.6 dry-run plan, orientation
  target, and execution-gate status.
- `ik_backend_available`: true only when a callable compute-IK path is visible
  with the required message support; otherwise false.
- `recommended_backend`: one of `moveit_compute_ik`, `configure_moveit`, or
  `add_moveit_or_custom_ik_service`.
- `recommended_next_step` and `decision_reason`: diagnostic guidance for the
  next implementation step.

These fields are infrastructure diagnostics only. They do not solve IK, do not
produce joint targets, do not send trajectory goals, and do not unblock
controller execution.

## Baseline v2.8 MoveIt Config Audit Fields

The `moveit_config_audit` node publishes JSON on `/moveit_config_audit` with:

- `status`: always `moveit_config_audit_diagnostic_only_no_motion`.
- `controller_motion_allowed`: always false.
- `trajectory_execution_allowed`: always false.
- `packages`: availability of `moveit_ros_move_group`, `moveit_msgs`, and
  `moveit_kinematics`.
- `moveit_config_package_found`: true when a likely installed or source MoveIt
  config package with relevant config resources is found.
- `srdf_found`, `kinematics_yaml_found`, `joint_limits_yaml_found`,
  `ompl_planning_yaml_found`, and `move_group_launch_found`: file-level config
  readiness checks.
- `robot_description_available`: observed `robot_description` parameter
  availability from `robot_state_publisher` when visible.
- `joint_states_available` and `joint_names_observed`: current robot state
  visibility.
- `compute_ik_service_available`: true only when `/compute_ik` is visible.
- `moveit_ready_for_compute_ik`: true only when the MoveIt config resources,
  move-group launch readiness, and `/compute_ik` visibility are all confirmed.
- `recommended_next_step`: one of `create_moveit_config_package`,
  `launch_move_group_diagnostic_only`, or `test_compute_ik_service_no_motion`.

The audit is preparation only. It does not launch `move_group`, call IK, invent
joint targets, send trajectory goals, or enable controller execution.

## Baseline v2.9 MoveIt Launch Readiness Audit Fields

The `moveit_launch_readiness_audit` node publishes JSON on
`/moveit_launch_readiness_audit` with:

- `moveit_launch_ready`: true only when an exact semantic model, IK config,
  OMPL config, and a move-group launch path are present.
- `compute_ik_expected_after_launch`: true when the readiness inputs indicate a
  diagnostic move-group launch should provide `/compute_ik`, or when
  `/compute_ik` is already visible.
- `exact_robot_semantic_match`: true only for the exact `lbr_iisy6_r1300`
  semantic model.
- `selected_srdf`: the exact matching SRDF path, or null when no exact match
  exists.
- `available_srdf_variants`: discovered SRDF and SRDF xacro resources.
- `kinematics_yaml_found`, `kinematics_yaml_file`,
  `ompl_planning_yaml_found`, `ompl_planning_yaml_file`,
  `joint_limits_yaml_found`, and `joint_limits_yaml_file`: launch input
  resource checks.
- `robot_description_available` and `robot_description_semantic_available`:
  observed description parameters when visible.
- `move_group_launch_found` and `move_group_launch_files`: launch files that
  appear to start `moveit_ros_move_group`/`move_group`.
- `controller_motion_allowed` and `trajectory_execution_allowed`: always false.
- `recommended_next_step`: one of
  `create_or_select_matching_srdf_for_lbr_iisy6_r1300`,
  `create_move_group_diagnostic_launch`, `launch_move_group_diagnostic_only`,
  or `test_compute_ik_service_no_motion`.

v2.9 is launch preparation only. It does not launch `move_group`, call IK,
fake IK solutions, send `FollowJointTrajectory` goals, start Gazebo, or unblock
controller execution.

## Baseline v2.10 Semantic Model Validation Fields

The `semantic_model_validator` node publishes JSON on
`/semantic_model_validation` with:

- `target_robot_model`: `lbr_iisy6_r1300`.
- `selected_moveit_config_package`: `project_local_lbr_iisy6_r1300_overlay`.
- `selected_srdf`: the project-local `lbr_iisy6_r1300.srdf` path.
- `srdf_exists`, `srdf_contains_group_arm`, and
  `srdf_references_required_joints`: static SRDF checks for the `arm` group and
  `joint_1` through `joint_6`.
- `joint_states_available`, `joint_state_names`, and
  `joint_states_contain_required_joints`: live `/joint_states` compatibility
  checks when robot state is present.
- `tool_link_requires_validation` and `tool_link_validation_status`: explicit
  marker that end-effector/tool semantics are not yet validated.
- `semantic_model_exact_candidate`: true only when the candidate SRDF names the
  target robot, defines group `arm`, and references all required joints.
- `semantic_model_validation_status`: `candidate_requires_validation`.
- `approved_for_motion`, `controller_motion_allowed`, and
  `trajectory_execution_allowed`: always false.

v2.10 prepares a semantic model candidate only. It does not launch
`move_group`, call IK, fake IK solutions, send `FollowJointTrajectory` goals,
start Gazebo, or unblock controller execution.

## Baseline v2.11 robot_description_semantic Diagnostic Fields

The `robot_description_semantic_diagnostics` node publishes JSON on
`/robot_description_semantic_diagnostics` with:

- `srdf_file_path`, `srdf_file_exists`, and `srdf_parse_success`: the resolved
  project-local or installed SRDF candidate and parse state.
- `robot_description_semantic_available` and
  `robot_description_semantic_length`: whether file-backed semantic XML is
  available for a future parameter path, and its character length.
- `arm_group_found`, `arm_group_joints`, and `required_joints_present`: static
  `arm` group checks for `joint_1` through `joint_6`.
- `semantic_model_validation_status`: diagnostic candidate status only.
- `approved_for_motion`, `controller_motion_allowed`, and
  `trajectory_execution_allowed`: always false.

`moveit_launch_readiness_audit` now also reports
`robot_description_semantic_candidate_available`,
`robot_description_semantic_source`, `semantic_diagnostics_available`, and
`semantic_diagnostics_status`. If the SRDF candidate is structurally valid but
tool-link validation is still required, `moveit_launch_ready` remains false and
the recommended next step is
`validate_tool_link_and_prepare_move_group_diagnostic_launch`.

v2.11 prepares semantic diagnostics only. It does not launch `move_group`, call
`/compute_ik`, fake IK solutions, send `FollowJointTrajectory` goals, start
Gazebo, or unblock controller execution.

## Baseline v2.12 Tool-Link Validation Fields

The `tool_link_validator` node publishes JSON on `/tool_link_validation` with:

- `tool_link_candidate`: currently `tool0`.
- `robot_description_available`, `urdf_parse_success`, and
  `tool_link_exists_in_urdf`: whether `robot_description` was readable and
  contains the candidate link.
- `tf_world_to_tool_available`, `tf_base_to_tool_available`, and
  `tf_world_to_base_available`: required TF checks for the diagnostic candidate.
- `current_tool_pose_world` and `current_tool_pose_base`: current diagnostic TF
  poses when available.
- `arm_group_found`, `arm_group_joints`, and `required_joints_present`: SRDF
  consistency checks for the project-local `arm` group.
- `selected_tool_axis_candidate`, `expected_aligned_insertion_axis`,
  `tool_axis_candidate_available`, and `orientation_targets_available`: optional
  tool-axis/orientation observations for `tool0_+Z` and `[0, 0, -1]`.
- `tool_link_validation_status`: either
  `tool_link_candidate_valid_but_not_motion_approved` or
  `tool_link_candidate_incomplete`.
- `approved_for_motion`, `controller_motion_allowed`, and
  `trajectory_execution_allowed`: always false.

`semantic_model_validator` also reports
`tool_link_candidate_validated_for_diagnostics` when the validation topic is
observed with a valid diagnostic status. `moveit_launch_readiness_audit`
reports `tool_link_validation_available`, `tool_link_candidate`,
`tool_link_exists_in_urdf`, `tf_base_to_tool_available`, and
`tool_link_validation_status`. A valid diagnostic tool link changes the
recommended next step to `prepare_move_group_diagnostic_launch_inputs`; it does
not make `moveit_launch_ready` true and does not enable `/compute_ik` or motion.
