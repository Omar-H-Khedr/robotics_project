# Simulation Requirements

> Translates the doctoral proposal (`proposal_full.md`) into concrete ROS/Gazebo simulation requirements. Source context at `proposal_context.md`. Current workspace state at `project_map.md`.

---

## 1. Required Simulation Objective

Build a calibrated physics-based simulation of a KUKA LBR iisy collaborative robot performing peg-in-hole insertion, capable of:

- Generating repeatable contact interactions under controlled geometric and tolerance variability.
- Logging multi-modal observations (RGB-D, force/torque, proprioception) for downstream RL training.
- Evaluating insertion success, cycle time, peak contact wrench, and safety violations.
- Serving as the training environment for context-conditioned SAC policies (deferred to proposal months 10–22).
- Hosting transfer-gate evaluation: ≥90% Safe Success, zero hard constraint violations, bounded contact wrench.

**Proposal engine:** NVIDIA Isaac Sim. **Workspace engine:** Gazebo (GazeboSim + `gz_ros2_control`). This mismatch must be resolved before the architecture is finalised.

---

## 2. Minimum Demonstration That Must Work Now

Derived from proposal Months 1–6 (Foundations). Current workspace is already ahead of this baseline (v2.0–v2.16 validated), but it defines the floor:

| # | Capability | Current Status | Requirement |
|---|-----------|----------------|-------------|
| 2.1 | Full cell spawns in simulation | v2.0–v2.16 validated | Robot + gripper + table + pedestal + peg + hole fixture + camera all present at correct poses |
| 2.2 | Robot moves under position control | v2.0–v2.4 validated | `joint_trajectory_controller` executes multi-point joint trajectories |
| 2.3 | Contact sensors publish force data | v2.6–v2.10 validated | `/gazebo/contacts/peg`, `/gazebo/contacts/hole`, `/gazebo/contacts/target` publish at simulation rate |
| 2.4 | Guarded approach-to-contact sequence | v2.5–v2.10 validated | Phase-based execution: home → staging → guarded approach → contact detection → hold → retreat |
| 2.5 | Metric extraction publishes | v2.11–v2.16 validated | `/insertion_metrics` topic with force vectors, contact state |
| 2.6 | Safety monitor runs | exists | `/safety_status` with joint-limit and phase-timeout checks |
| 2.7 | RGB-D camera publishes | P2 applied | `/camera/color/image_raw`, `/camera/depth/image_raw`, `/camera/color/camera_info` bridged from Gazebo |
| 2.8 | Reproducible single-command launch | exists | `research_baseline.launch.py` with robot_model, use_gui, world_file args |

**Gap:** Multi-modal synchronous logging (RGB-D + F/T + joint states at 50 Hz time-aligned) is not yet validated end-to-end. Perception pipeline is a stub.

---

## 3. Robot Platform Requirements

| Requirement | Proposal Spec | Workspace Current | Status |
|-------------|--------------|-------------------|--------|
| Robot model | KUKA LBR iisy 6 R1300 (1300mm reach, 7-DOF) | `lbr_iisy3_r760` (760mm reach, 6-DOF) | **Mismatch — needs confirmation** |
| Joint-torque sensing | Integrated — used for wrench feedback and admittance | Not exposed in Gazebo simulation (position-only control) | **Needs implementation** |
| Controller interface | Position commands + admittance mediation | Position-only via `joint_trajectory_controller` | **Minimum met for foundations; admittance deferred** |
| Base frame | World-fixed on pedestal | `[0.80, -0.75, 0.735]` with yaw 1.5708 rad | Verified correct |
| Update rate | 50 Hz for control loop | Controller `state_publish_rate: 50.0`, `update_rate: 50` | Compliant |

**Missing / needs confirmation:**
- Exact joint limits of iisy6 R1300 (if different from iisy3 R760).
- Reachable workspace bounds for safety filter.
- Whether the real robot has 6 or 7 DOF (proposal says 7-DOF; workspace has 6 joints).
- Whether the existing MoveIt config overlay (`moveit_lbr_iisy6_r1300`) can be used or must be regenerated.

---

## 4. Workcell Requirements

| Element | Requirement | Current | Status |
|---------|-------------|---------|--------|
| Table | Fixed surface at known height | 0.80×0.60m, surface at z=0.75 | Compliant |
| Robot pedestal | Positions robot base at correct height | Cylinder 0.36m radius × 0.72m + 0.03m top plate | Compliant |
| Hole fixture | Fixed socket at known pose | 0.18×0.18×0.04m nest, 27mm opening at `[0.52, -0.20, 0.75]` | Compliant |
| Target plate | Thin plate with hole opening on top of fixture | 0.18×0.18×0.02m, 27mm opening at `[0.52, -0.20, 0.79]` | Compliant |
| World peg (unused) | Separate floating cylinder | 25mm dia × 110mm at `[0.72, -0.05, 0.75]` | Legacy — may remove later |
| Robot-table clearance | Sufficient for approach sequences | 0.45m gap between robot y=-0.75 and table edge y=-0.30 | Compliant |

**Coordinate frame convention:**
- World +X: table depth, +Y: robot-to-table direction, +Z: up
- All poses in world frame unless noted

**Missing / needs confirmation:**
- Whether additional workcell variants are needed for domain randomization (e.g., alternative hole positions, table heights).
- Pedestal/table material properties for contact physics calibration.

---

## 5. Peg-in-Hole Task Requirements

| Parameter | Value (workspace) | Proposal Spec | Status |
|-----------|-------------------|---------------|--------|
| Peg shape | Cylindrical, 25mm dia × 110mm | Not specified | **Workspace provides a baseline; proposal implies multiple variant geometries** |
| Hole shape | Circular, 27mm dia (1mm radial clearance) | Not specified | Workspace provides a baseline |
| Peg attachment | Grasped in parallel gripper, extends 0.11m below TCP | Not detailed | P3 applied — peg present on robot gripper |
| Hole frame | `target_hole_frame` at `[0.52, -0.20, 0.81]` (world) | Not specified | Compliant |
| Nominal approach | IK-planned trajectory to staging pose above hole | Classical planning provides nominal approach | Deferred — requires Cartesian motion validation |

**Task phases (from baseline_task_sequence.yaml):**
1. `safe_home` — arm raised clear of workspace
2. `observe_scene` — camera view pose
3. `pre_grasp` / `grasp_approach` — move above hole (peg is already fixed, no actual grasp)
4. `lift_clearance` — raise to clearance height
5. `pre_insert` — align above hole
6. `insertion_approach` — descend toward hole
7. `insertion_hold` — seated position
8. `retreat` — pull out
9. `return_home`

**Missing / needs confirmation:**
- Number and nature of "multiple part variants" (geometry changes? tolerance changes? both?).
- Specific clearance values for stress-test scenarios.
- Whether non-cylindrical pegs (square, chamfered) are required.
- Hole depth and bottom condition (through-hole? blind?).
- Peg chamfer geometry (if any) — critical for insertion success.
- Lubrication or friction assumptions.

---

## 6. End-Effector / Tool Requirements

| Requirement | Current | Status |
|-------------|---------|--------|
| Gripper present on robot flange | Parallel gripper (palm 80×60×30mm + two fingers 18×12×90mm) | Compliant |
| TCP defined | 125mm from palm | Compliant |
| Peg attached to gripper | Cylindrical peg 25mm dia × 110mm at TCP, extends 0.11m below | P3 applied — compliant |
| Peg_tip frame exists | `tool0_to_peg_tip` at z-offset 0.11m from tool0 | Compliant |
| Gripper fingers physically grasp | Fingers are fixed (no prismatic joint) | **Cannot close — peg is fused to gripper** |
| Physical collision model for peg | Collision + inertial properties added | P3 applied |

**Missing / needs confirmation:**
- Whether the gripper must eventually actuate (servo close around peg before insertion).
- If the gripper will grasp different peg variants, finger geometry may need to accommodate different diameters.
- Fingertip material and friction for grasp stability.
- Whether the peg is ever released after insertion.

---

## 7. Camera / Perception Requirements

| Requirement | Current | Status |
|-------------|---------|--------|
| RGB-D camera present in simulation | D405 as static world model with color + depth sensors | P2 applied |
| Camera TF frames published | `d405_camera_link`, `d405_camera_optical_frame` in URDF | P2 applied |
| Camera pose | `[0.42, -0.65, 1.18]` world, rpy `[0.95, 0, 0.35]` (~54° downward tilt) | From old prototype — **not validated against physical setup** |
| Camera bridge topics | `/camera/color/image_raw`, `/camera/depth/image_raw`, `/camera/color/camera_info` | P2 applied |
| Image resolution | 848×480 at 30 Hz (as configured in world SDF) | P2 applied |
| Render engine | ogre2 (as configured) | P2 applied |
| Perception pipeline | `perception_pipeline` stub subscribes to camera topics | **Stub only — no processing** |

**Missing / needs confirmation:**
- Camera mounting pose on the physical robot (not specified in proposal — workspace uses a guess from old prototype).
- Camera intrinsics (fx, fy, cx, cy) — not specified in proposal, not derivable from datasheet without explicit values.
- Whether depth registration to color is required.
- Whether the camera is mounted on the robot (eye-in-hand) or external (eye-to-hand). Current config is eye-to-hand (world-fixed).
- Frame rate requirement for RL observations (proposal says 50 Hz control; camera is 30 Hz — possible mismatch).
- Sensor noise model for domain randomization (not specified).

---

## 8. Controller-Driven Motion Requirements

| Requirement | Current | Status |
|-------------|---------|--------|
| Joint trajectory execution | `joint_trajectory_controller` with `FollowJointTrajectory` action | Validated |
| Command interface | Position | **Only position — no velocity, effort, stiffness, or damping** |
| State interface | Position | **Only position — no velocity or effort feedback** |
| Control rate | 50 Hz state publish, 20 Hz action monitor | Compliant |
| MoveIt integration | IK diagnostics + plan-only (v2.2–v2.4) | Validated for planning |
| MoveIt → Gazebo execution | Joint-space trajectory execution (v2.4) | Validated |
| Cartesian motion | `peg_hole_cartesian_targets.yaml` exists but `motion_execution_allowed: false` | **Not ready — "tool insertion axis not validated"** |
| Position accuracy | Not measured | **Needs characterisation** |

**Missing / needs confirmation:**
- Is velocity or effort control available in Gazebo for this hardware plugin?
- How will admittance control be implemented if only position commands are available? (External admittance: twist → position via integration?)
- Is stiffness/damping control required, or can an outer-loop admittance wrapper work with the position interface?
- Position tracking error under load (contact) — does Gazebo's position controller handle contact forces adequately?

---

## 9. Safety-Layer Requirements

| Requirement | Current | Status |
|-------------|---------|--------|
| Joint-limit checking | `safety_monitor` node checks soft limits | Exists |
| Phase timeout | `safety_monitor` enforces phase duration limits | Exists |
| `/safety_status` topic | Published with validity flags | Exists |
| Pre-contact constraint enforcement | Not implemented — current is monitoring only | **Deferred to proposal months 18–33** |
| Runtime safety filter (velocity, workspace, approach constraints) | Not implemented | **Deferred** |
| Admittance/virtual-force mediation during contact | Not implemented | **Deferred** |
| Safety limits configured | `safety_limits.yaml` | Exists — verify completeness |

**Safety constraints needed (from proposal):**
- Pre-contact: velocity limits, workspace boundaries, approach-region constraints
- Contact: force bounds via admittance mediation (ISO/TS 15066 alignment)
- Transfer gate: zero hard pre-contact constraint violations before real-robot deployment

**Missing / needs confirmation:**
- What constitutes a "hard constraint violation" — specific thresholds?
- ISO/TS 15066 force/pressure limits for KUKA LBR iisy — not specified.
- Workspace boundary geometry for safety filter — not specified.
- Whether the safety filter runs in simulation (to measure interventions) or only on real hardware.

---

## 10. What Should Be Mocked Now

The following components should exist as stubs or simplified versions to enable infrastructure validation before the learning pipeline is built:

| Component | Mock Strategy | Rationale |
|-----------|---------------|-----------|
| **Learning policy** | `learning_interface` stub — accept dummy action commands, publish dummy state | Full SAC training deferred to months 10–22 |
| **Context encoder** | v2.12–v2.13 stubs exist — fixed-context or no-context baseline | Training deferred |
| **Admittance layer** | Bypass — policy output → direct position command (no force mediation) | Deferred to months 18–33 |
| **Runtime safety filter** | Bypass — `safety_monitor` stays as observer only | Deferred to months 18–33 |
| **Domain randomization** | Manual config overrides (e.g., edit YAML, re-launch) instead of automated DR engine | Automated DR deferred but planning acceptable |
| **Multi-variant pegs/holes** | Single cylindrical peg + single hole (current setup) | Variants deferred — must validate baseline insertion first |
| **Perception encoding** | `perception_pipeline` subscribes but passes raw images through (no feature extraction) | Visual encoder training deferred |

**Temporary measures for these mocks must be clearly documented so they are not mistaken for final implementations.**

---

## 11. What Should NOT Be Implemented Yet

Derived from proposal timeline — these belong to later phases:

| Component | Proposal Phase | Earliest Month | Current Status |
|-----------|---------------|----------------|----------------|
| SAC policy training | Core learning | 10 | Stub only |
| Context encoder (GRU/Transformer) training | Core learning | 10 | Stub only |
| Context-based meta-RL inner/outer loop | Core learning | 10–22 | Not started |
| Domain randomization engine (automated) | Simulation + representation | 4–12 | Not started |
| Sim-to-real calibration pipeline | Transfer + safety | 18–33 | Not started |
| Runtime safety filter (enforcement) | Transfer + safety | 18–33 | Not started |
| Admittance/virtual-force controller | Transfer + safety | 18–33 | Not started |
| Real-robot deployment | Transfer + safety | 18–33 | Not started |
| Diffusion Policy baseline | Adjacent work (not in proposal scope) | N/A | Not started |
| Multi-variant peg/hole geometry generation | Core learning | 10–22 | Not started |
| Online adaptation evaluation | Core learning | 10–22 | Not started |

**Exception:** If the workspace is intentionally ahead of the proposal timeline and targets Isaac Sim, preliminary investigation of the simulation engine migration may begin. However, no production learning code should be merged until the simulation cell is stable and validated.

---

## 12. Acceptance Criteria for the Current Sprint

The simulation cell must pass these checks to be considered ready for the next phase (baseline insertion demonstration):

### Cell Integrity
- [ ] 12.1 Robot spawns at correct pose (`[0.80, -0.75, 0.735]`, yaw 1.5708). Verified via `tf2_echo world base_link`.
- [ ] 12.2 Gripper + peg present on robot flange. Verified via `tf2_echo tool0 peg_tip`.
- [ ] 12.3 Hole fixture + target plate at correct pose. Verified via `tf2_echo world target_hole_frame`.
- [ ] 12.4 D405 camera publishes RGB and depth streams. Verified via `ros2 topic hz /camera/color/image_raw`.
- [ ] 12.5 No TF tree warnings or missing frames.

### Motion
- [ ] 12.6 `joint_trajectory_controller` accepts and executes `FollowJointTrajectory` goals.
- [ ] 12.7 Robot can move from home to staging pose above hole without collisions.
- [ ] 12.8 Robot returns to home reliably.

### Contact
- [ ] 12.9 Contact topics publish on peg–hole or peg–target interaction. Verified via `ros2 topic echo /gazebo/contacts/peg`.
- [ ] 12.10 `contact_metrics_node` parses contact data and publishes `/insertion_metrics`.

### Safety
- [ ] 12.11 `safety_monitor` publishes `/safety_status` with `joints_valid: true` during normal motion.
- [ ] 12.12 Phase timeout triggers safety fault (e.g., if a phase exceeds its configured duration).

### Repeatability
- [ ] 12.13 Full sequence (home → approach → contact → retreat → home) runs 5 times without failure.
- [ ] 12.14 Trial manager logs each trial to `results/baseline_trials/` with timestamps.

### Launch
- [ ] 12.15 Single command launches full cell headless and produces expected topics:
  ```bash
  ros2 launch thesis_bringup research_baseline.launch.py use_gui:=false
  ```
- [ ] 12.16 Single command launches full cell with GUI for visual inspection:
  ```bash
  ros2 launch thesis_bringup research_baseline.launch.py use_gui:=true
  ```

### Evidence
- [ ] 12.17 All acceptance checks above are documented with terminal output or log excerpts in the diagnostic evidence directory (`ros2_ws/diagnostics/`).

---

## 13. Evidence Required for README

For each requirement claimed as "validated," the following evidence must exist in `ros2_ws/diagnostics/`:

| Evidence | Format | Example |
|----------|--------|---------|
| TF tree diagram | PDF/PNG | `tf_tree.pdf` showing all frames and transforms |
| Robot spawn pose | Text log + screenshot | Terminal output of `gz model --list` and Gazebo GUI showing robot on pedestal |
| Joint motion validation | CSV log + text | Position trajectory waypoints vs. actual joint states recorded during motion |
| Contact force reading | CSV log | Force vector data from `/gazebo/contacts/*` during a contact event |
| Camera image sample | PNG | Side-by-side: RGB image + depth image from bridged topics |
| Safety status log | Text log | `/safety_status` messages during normal and fault conditions |
| Metric extraction sample | JSON/CSV | Recorded `/insertion_metrics` message showing computed forces |
| Launch command output | Text log | Terminal output showing successful launch, controller activation, and clean shutdown |
| Trial log sample | CSV/JSON | One complete trial record from `results/baseline_trials/` |
| Diagnostic summary | Markdown | Per-sprint report summarising what was tested, what passed/failed, and what changed |

**Naming convention:** Evidence files should follow the sprint naming pattern, e.g. `v2_16_guarded_peg_in_hole_objective_validation/` with a README inside documenting the validated claims.

---

*Generated from `proposal_full.md` via `proposal_context.md`. No ROS source, launch, URDF, or Xacro files were modified in creating this document.*
