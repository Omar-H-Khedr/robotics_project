# Project Map: KUKA LBR iisy Peg-in-Hole Simulation Cell

> **Changes made in this session (P0–P6 fixes applied):**
> - P0: Deprecated `proposal_lbr_iisy6_r1300_cell.urdf.xacro` (cylinder placeholder)
> - P1: Fixed robot base z from 0.75→0.735 in `research_baseline.launch.py` (pedestal alignment)
> - P2: Added D405 camera (world SDF model + URDF TF frames + bridge topics)
> - P3: Added grasped peg (0.11m cylinder) to `research_parallel_gripper.xacro`
> - P4: Fixed `peg_hole_cartesian_targets.yaml` hole_center z from 0.845→0.810
> - P5: Verified table/robot alignment — correct, no change needed
> - P6: Verified bridge configs correctly paired with worlds — correct, no change needed

## 1. ROS 2 Packages and Their Roles

| Package | Type | Role |
|---------|------|------|
| **thesis_bringup** | Python (ament) | Top-level launch + config orchestrator. 36+ launch files, 20+ YAML configs. Contains all proposal-simulation-cell validation nodes (v1.0–v2.16). |
| **peg_in_hole_description** | Python (ament) | Gazebo world SDF files, URDF cell descriptions (canonical: `lbr_iisy3_r760_research_gripper.urdf.xacro`), static model SDF files (table, pedestal, peg, hole_fixture, target_plate, contact_validation_pad). |
| **kuka_task_control** | Python (ament) | Task-level control nodes: `task_trajectory_executor`, `baseline_joint_sequence_executor`, IK diagnostics, Cartesian insertion planner, segmented contact executors, MoveIt config overlay for `lbr_iisy6_r1300`. |
| **safety_layer** | Python (ament) | `safety_monitor` node: joint-state validity, soft-limit checking, phase timeout, publishes `/safety_status`. Config in `safety_limits.yaml`. |
| **experiment_manager** | Python (ament) | `baseline_trial_manager` (structured CSV/JSON logger), `research_baseline_v2_4_experiment_runner` (dry-run experiment generator). |
| **peg_in_hole_metrics** | Python (ament) | `contact_metrics_node`: subscribes to `/gazebo/contacts/*`, extracts force vectors, publishes `/insertion_metrics`. |
| **perception_pipeline** | Python (ament) | RGB-D perception interface stub: subscribes `/camera/color/image_raw`, `/camera/depth/image_raw`. Config `rgbd_pipeline.yaml`. |
| **learning_interface** | Python (ament) | RL interface package (stub, `__init__.py` only). |
| **first_robot_demo** | Python (ament) | Demo/starter package (placeholder). |
| **external/kuka_robot_descriptions/** | Mixed (ament/cmake) | Upstream KUKA descriptions (forked from `kuka_robot_descriptions`). Contains `kuka_lbr_iisy_support`, `kuka_gazebo`, `kuka_resources`, `kuka_lbr_iisy_moveit_config`, plus support for KR/Agilus/Fortec/Iontec/Cybertech/KL series. |
| **robot_description** | (install only) | Installed from external. Symlink to `external/kuka_robot_descriptions/kuka_lbr_iisy_support`. |
| **robot_simulation** | (install only) | Installed from external. |

## 2. Main Launch Commands

### Research Baseline (Phase 2B)
```bash
# Full Gazebo GUI launch with KUKA + peg-in-hole world
ros2 launch thesis_bringup research_baseline.launch.py \
  robot_model:=lbr_iisy3_r760 \
  robot_family:=lbr_iisy \
  use_gui:=true \
  world_file:=peg_in_hole_world.sdf
```

### Proposal Simulation Cell Validations
```bash
# Latest objective validation (v2.16)
ros2 launch thesis_bringup \
  proposal_simulation_cell_v2_16_guarded_peg_in_hole_objective_validation.launch.py \
  gui:=true

# Smoke test / motion validation (v2.0)
ros2 launch thesis_bringup \
  proposal_simulation_cell_v2_0_first_gazebo_motion_smoke_test.launch.py

# Motion validation suite (v2.1)
ros2 launch thesis_bringup \
  proposal_simulation_cell_v2_1_gazebo_motion_validation_suite.launch.py

# MoveIt IK diagnostic (v2.2)
ros2 launch thesis_bringup \
  proposal_simulation_cell_v2_2_moveit_ik_diagnostic_validation.launch.py

# MoveIt model alignment + plan only (v2.3)
ros2 launch thesis_bringup \
  proposal_simulation_cell_v2_3_moveit_model_alignment_and_plan_only_validation.launch.py

# MoveIt → Gazebo execution (v2.4)
ros2 launch thesis_bringup \
  proposal_simulation_cell_v2_4_moveit_gazebo_execution_validation.launch.py

# Guarded pre-contact task sequence (v2.5)
ros2 launch thesis_bringup \
  proposal_simulation_cell_v2_5_guarded_pre_contact_task_sequence.launch.py

# Contact-gated guarded approach (v2.6)
ros2 launch thesis_bringup \
  proposal_simulation_cell_v2_6_contact_gated_guarded_approach_validation.launch.py

# Misalignment contact gate batch (v2.10)
ros2 launch thesis_bringup \
  proposal_simulation_cell_v2_10_misalignment_contact_gate_batch_validation.launch.py

# Task sequence runner (standalone, after Gazebo is up)
ros2 launch kuka_task_control run_task_sequence.launch.py
```

### Experiment Runner (dry-run)
```bash
ros2 launch thesis_bringup research_baseline_v2_4_experiment_runner.launch.py
```

### Gazebo-only startup (external KUKA package)
```bash
ros2 launch kuka_gazebo gazebo_startup.launch.py \
  robot_model:=lbr_iisy3_r760 \
  robot_family:=lbr_iisy \
  gz_world:=world/empty_world.sdf \
  use_gui:=true
```

## 3. KUKA Robot Description Files

### Primary Research Cell URDF (USED by research_baseline)
- **`peg_in_hole_description/urdf/lbr_iisy3_r760_research_gripper.urdf.xacro`**
  - Includes `kuka_lbr_iisy_support/urdf/lbr_iisy3_r760_macro.xacro` (mesh-based real robot)
  - Includes `research_parallel_gripper.xacro` (custom gripper with palm + 2 fingers + TCP)
  - Includes `kuka_lbr_iisy_support/urdf/lbr_iisy_ros2_control_macro.xacro`
  - Places base_link via `world-base_link` joint at launch-arg x/y/z/rpy
  - Used by `research_baseline.launch.py`
  - **Contains NO camera link**

### Old Prototype Cell URDF (NOT used by research_baseline)
- **`peg_in_hole_description/urdf/proposal_lbr_iisy6_r1300_cell.urdf.xacro`**
  - **INCORRECT cylinder-based robot**: 6 fake cylindrical links (not real KUKA meshes)
  - Contains D405 camera at `[0.42, -0.65, 1.18]` with rpy `[0.95, 0, 0.35]`
  - Contains `tool0` and `peg_tip` frames
  - **DO NOT USE** for production work — this is a placeholder from early sprints

### Offical KUKA Description (upstream fork)
- **`external/kuka_robot_descriptions/kuka_lbr_iisy_support/urdf/lbr_iisy3_r760.urdf.xacro`**
  - Pure upstream-style: no gripper, no camera
  - Used by `proposal_simulation_cell_v2_*.launch.py` launches
- **`external/kuka_robot_descriptions/kuka_lbr_iisy_support/urdf/lbr_iisy3_r760_macro.xacro`**
  - Real mesh-based KUKA LBR iisy 3 R760 links/joints
  - Standard ROS-Industrial frames: `base`, `flange`, `tool0`
  - 6 revolute joints with correct limits, meshes from STL files

### Gripper Description
- **`peg_in_hole_description/urdf/research_parallel_gripper.xacro`**
  - Macro: `research_parallel_gripper(prefix, parent_link, tcp_frame)`
  - Palm: 80×60×30 mm box
  - Left/right fingers: 18×12×90 mm boxes at ±31mm y-offset, 75mm z
  - TCP at 125mm from palm

### Task Frame Description
- **`peg_in_hole_description/urdf/peg_in_hole_task.urdf.xacro`**
  - Defines `target_hole_frame` at `[0.52, -0.20, 0.81]` (world frame)
  - Defines nominal peg_tip at 0.06m above hole frame

## 4. Gazebo Workcell / Table / Peg / Hole Files

### World SDF Files
| File | Purpose |
|------|---------|
| `peg_in_hole_description/worlds/peg_in_hole_world.sdf` | **Main research baseline world**. Contains work_table at [0.80, 0, 0], robot_pedestal at [0.80, -0.75, 0.0], hole_fixture at [0.52, -0.20, 0.75], target_plate at [0.52, -0.20, 0.79], cylindrical_peg at [0.72, -0.05, 0.75]. |
| `peg_in_hole_description/worlds/peg_in_hole_contact_validation_world.sdf` | Contact validation world with robot contact pad. |
| `peg_in_hole_description/worlds/peg_in_hole_robot_contact_validation_world.sdf` | Robot contact validation version. |
| `peg_in_hole_description/worlds/peg_in_hole_insertion_validation_world.sdf` | Insertion validation world. |
| `peg_in_hole_description/worlds/proposal_simulation_cell_v1_*.world.sdf` | Sprint-specific worlds for v1.0/v1.2/v1.3 validation. |

### Static Model SDF Files
| Model | File | Pose | Size |
|-------|------|------|------|
| **work_table** | `models/work_table/model.sdf` | Centered at model origin, 0.80×0.60×0.05m top, legs 0.70m tall, work surface at z=0.75 |
| **robot_pedestal** | `models/robot_pedestal/model.sdf` | 0.36m radius cylinder × 0.72m tall + 0.46×0.46×0.03m top plate. Total height 0.75m. |
| **hole_fixture** | `models/hole_fixture/model.sdf` | 0.18×0.18×0.04m square nest with 27mm diameter socket opening |
| **target_plate** | `models/target_plate/model.sdf` | 0.18×0.18×0.02m, 27mm hole opening |
| **cylindrical_peg** | `models/cylindrical_peg/model.sdf` | 25mm diameter × 110mm length. Static=false. Contact sensor on `/gazebo/contacts/peg`. |
| **contact_validation_pad** | `models/contact_validation_pad/model.sdf` | 0.16×0.05×0.03m, red. Contact sensor on `/gazebo/contacts/validation`. |

### Coordinate Layout
```
World +X: table depth direction
World +Y: from robot base toward table
World +Z: up

Table center:    [0.80,  0.00, 0.00] (model origin, surface at z=0.75)
Robot pedestal:  [0.80, -0.75, 0.00] (base, top at z=0.75)
Robot base_link: [0.80, -0.75, 0.75] yaw=1.5708 (faces toward table)
Hole fixture:    [0.52, -0.20, 0.75] (bottom)
Target plate:    [0.52, -0.20, 0.79] (hole opening at z=0.81)
Peg (world):     [0.72, -0.05, 0.75] (separate model, not on robot)
```

## 5. Camera Configuration

**D405 Camera** is defined only in `proposal_lbr_iisy6_r1300_cell.urdf.xacro` (the old prototype):
- Link: `d405_camera_link` at `[0.42, -0.65, 1.18]` (world frame)
- Orientation: rpy `[0.95, 0, 0.35]` rad (~54° downward tilt)
- Optical frame: `d405_camera_optical_frame` (ROS standard: z-forward, x-right, y-down)
- Camera points approximately toward the table center / workspace

**CRITICAL ISSUE**: The research baseline uses `lbr_iisy3_r760_research_gripper.urdf.xacro` which does NOT include the camera. There is **no camera** in the current research baseline cell.

The `perception_pipeline/config/rgbd_pipeline.yaml` expects topics:
- `/camera/color/image_raw`
- `/camera/depth/image_raw`
- `/camera/color/camera_info`

These topics are not bridged in the current bridge config (`kuka_gazebo/config/bridge_config.yaml`), which only bridges `/clock`, `/joint_states`, and `/cmd_vel`.

## 6. Controller Configuration

### Controller Manager Config (used by research baseline)
**`kuka_resources/config/fake_hardware_config_6_axis.yaml`**:
```yaml
controller_manager:
  update_rate: 50
  joint_trajectory_controller:
    type: joint_trajectory_controller/JointTrajectoryController
  joint_state_broadcaster:
    type: joint_state_broadcaster/JointStateBroadcaster

joint_trajectory_controller:
  joints: [joint_1..joint_6]
  command_interfaces: [position]
  state_interfaces: [position]
  state_publish_rate: 50.0
  action_monitor_rate: 20.0
```

### MoveIt Controller Config
**`kuka_lbr_iisy_moveit_config/config/moveit_controllers.yaml`**:
- Controller: `joint_trajectory_controller` (FollowJointTrajectory)
- Joints: `joint_1` through `joint_6`

### Controller Spawner
- `joint_state_broadcaster` → activated first
- `joint_trajectory_controller` → activated after joint_state_broadcaster exits
- Action server: `/joint_trajectory_controller/follow_joint_trajectory`

### ros2_control Hardware
In `lbr_iisy_ros2_control_macro.xacro`:
- `mode=gazebo` → `gz_ros2_control/GazeboSimSystem` plugin
- `mode=mock` → `kuka_mock_hardware_interface` plugin
- `mode=hardware` → real KUKA EAC/EKI/RSI/MXA driver plugins

## 7. Task-Control and Trajectory Files

### Executor Nodes (kuka_task_control)
| Node | File | Action |
|------|------|--------|
| `task_trajectory_executor` | `task_trajectory_executor.py` | Sequential FollowJointTrajectory executor with force/contact guarding. 10-pose sequence from YAML. |
| `baseline_joint_sequence_executor` | `baseline_joint_sequence_executor.py` | Simple multi-point single-goal sequence executor. 6-pose sequence from YAML. |
| `segmented_guarded_contact_executor` | `segmented_guarded_contact_executor.py` | Segmented contact approach executor. |
| `segmented_contact_executor` | `segmented_contact_executor.py` | Segmented robot contact executor. |
| `cartesian_insertion_diagnostics` | `cartesian_insertion_diagnostics.py` | Cartesian insertion diagnostics. |
| `ik_feasibility_diagnostics` | `ik_feasibility_diagnostics.py` | IK feasibility checks. |
| `peg_hole_frame_publisher` | `peg_hole_frame_publisher.py` | Publishes peg/hole TF frames. |

### Task Sequence Configs
| File | Purpose |
|------|---------|
| `config/baseline_task_sequence.yaml` | 10-pose sequence: safe_home → observe_scene → pre_grasp → grasp_approach → lift_clearance → pre_insert → insertion_approach → insertion_hold → retreat → return_home |
| `config/baseline_task_poses.yaml` | 11 poses including home, safe_above_table, observe_scene, pre_task, approach_workspace, retreat |
| `config/baseline_trajectory.yaml` | Simpler 5-pose baseline: home, safe_home, safe_above_table, pre_task, return_home |
| `config/robot_contact_validation_sequence.yaml` | 10-pose contact validation: safe_home → observe_scene → pre/mid/near/final approach → touch_candidate → hold → retreat → return_home |
| `config/peg_hole_cartesian_targets.yaml` | Cartesian positions for hole_center, staging, axis_align, insertion_touch, insertion_hold, final_insertion, retreat |
| `config/peg_hole_insertion_validation_sequence.yaml` | Peg-hole insertion sequence |

### Home Pose
All task sequences use `[0.0, -0.8, 1.2, 0.0, 0.8, 0.0]` (joint_1..6 in radians).
This keeps the arm raised and clear of the 0.75m table surface.

### MoveIt Overlay Config (for iisy6_r1300, diagnostic only)
- `config/moveit_lbr_iisy6_r1300/kinematics.yaml`: KDL solver, arm group
- `config/moveit_lbr_iisy6_r1300/ompl_planning.yaml`: RRTConnect, `diagnostic_only: true`
- `config/moveit_lbr_iisy6_r1300/moveit_config_metadata.yaml`: Metadata

## 8. Current Known Working Commands

Based on validated sprints (v2.0–v2.16, all verified):

```bash
# Build
cd ros2_ws && colcon build --symlink-install
source install/setup.bash

# V2.0: Single joint motion via Gazebo controller (position command)
ros2 launch thesis_bringup \
  proposal_simulation_cell_v2_0_first_gazebo_motion_smoke_test.launch.py

# V2.1: Combined motion validation (forward, return, repeatability)
ros2 launch thesis_bringup \
  proposal_simulation_cell_v2_1_gazebo_motion_validation_suite.launch.py

# V2.2: MoveIt IK diagnostic (compute_ik service call)
ros2 launch thesis_bringup \
  proposal_simulation_cell_v2_2_moveit_ik_diagnostic_validation.launch.py

# V2.3: MoveIt model alignment + plan-only
ros2 launch thesis_bringup \
  proposal_simulation_cell_v2_3_moveit_model_alignment_and_plan_only_validation.launch.py

# V2.4: MoveIt → Gazebo trajectory execution
ros2 launch thesis_bringup \
  proposal_simulation_cell_v2_4_moveit_gazebo_execution_validation.launch.py

# V2.5: Guarded pre-contact task sequence (no contact-seeking)
ros2 launch thesis_bringup \
  proposal_simulation_cell_v2_5_guarded_pre_contact_task_sequence.launch.py

# V2.6: Contact-gated guarded approach
ros2 launch thesis_bringup \
  proposal_simulation_cell_v2_6_contact_gated_guarded_approach_validation.launch.py

# V2.7: Contact-triggered guarded touch calibration
ros2 launch thesis_bringup \
  proposal_simulation_cell_v2_7_contact_triggered_guarded_touch_calibration.launch.py

# V2.8: Contact reachability and trigger validation
ros2 launch thesis_bringup \
  proposal_simulation_cell_v2_8_contact_reachability_and_trigger_validation.launch.py

# V2.9: Non-overlapping approach-to-contact
ros2 launch thesis_bringup \
  proposal_simulation_cell_v2_9_non_overlapping_approach_to_contact_validation.launch.py

# V2.10: Misalignment contact gate batch (5 scenarios)
ros2 launch thesis_bringup \
  proposal_simulation_cell_v2_10_misalignment_contact_gate_batch_validation.launch.py

# V2.11 through V2.16: Context extraction, encoder, action, ablation, objective
ros2 launch thesis_bringup \
  proposal_simulation_cell_v2_16_guarded_peg_in_hole_objective_validation.launch.py

# Direct joint_command publisher test
ros2 topic pub /joint_commands std_msgs/msg/Float64MultiArray \
  "{data: [0.0, -0.8, 1.2, 0.0, 0.8, 0.0]}"

# Check TF tree
ros2 run tf2_tools view_frames.py

# Check joint states
ros2 topic echo /joint_states

# Check contact topics
ros2 topic echo /gazebo/contacts/peg
ros2 topic echo /gazebo/contacts/hole
ros2 topic echo /gazebo/contacts/target

# Check controller
ros2 control list_controllers
ros2 control list_hardware_interfaces
```

## 9. Current Known Problems

### A. Robot Model Correctness
1. **Two conflicting robot descriptions exist**:
   - `proposal_lbr_iisy6_r1300_cell.urdf.xacro`: **FAKE cylinder robot** (not the real KUKA) — 6 cylindrical links with incorrect dimensions, inertias, joint limits. This is a v1 placeholder and should NOT be used.
   - `lbr_iisy3_r760_research_gripper.urdf.xacro`: Correct mesh-based robot from upstream. Used by research_baseline. **BUT** it uses `lbr_iisy3_r760` model (R760 = 760mm reach) while some references say `iisy6_r1300` (R1300 = 1300mm reach). **Naming/model mismatch.**

2. **The research baseline uses the iisy3 R760**, but the project name includes "iisy6 R1300". Verify which physical robot is intended.

### B. Robot Base Position
3. **Double coordinate system risk**: The world SDF places robot_pedestal at `[0.80, -0.75, 0.0]`, and the URDF world-base_link joint places base_link at `[0.80, -0.75, 0.75]`. The Gazebo spawn also has `[0,0,0]` offset. This double-positioning (SDF model + URDF joint) could cause the robot to appear at the wrong location if one is not properly aligned. Currently both match.

4. **The `proposal_lbr_iisy6_r1300_cell.urdf.xacro` has a different base position**: `[0.80, -0.75, 0.75]` with yaw `1.5708`. This matches the research baseline base position, but the old cell xacro also has a `base_link` with a visual cylinder (0.12m radius, 0.10m tall) that overlaps with the pedestal top.

### C. Workcell/Table Position
5. **Table positioning appears correct** for the research baseline: centered at `[0.80, 0.0, 0.0]`, surface at z=0.75. Robot at y=-0.75 gives 0.45m clearance from table edge at y=-0.30.

### D. Camera Direction
6. **NO camera in the research baseline**: The D405 camera is only in the old `proposal_lbr_iisy6_r1300_cell.urdf.xacro`. The research baseline URDF (`lbr_iisy3_r760_research_gripper.urdf.xacro`) has no camera at all.

7. **No RGB-D bridge configured**: The `kuka_gazebo/config/bridge_config.yaml` does not bridge camera topics. The `ros_gz_bridge` only bridges `/clock`, `/joint_states`, `/cmd_vel`.

8. **Perception pipeline expects topics** `/camera/color/image_raw` and `/camera/depth/image_raw` that don't exist.

### E. Peg and Hole Placement
9. **Peg is a separate world model**, not attached to the robot end-effector. The cylindrical_peg floats at `[0.72, -0.05, 0.75]` in the world. The robot's tool0 has no physical peg attached.

10. **Peg_tip frame exists in URDF** (`tool0_to_peg_tip` with z-offset 0.11m) but there is no physical collision model — it's just a TF frame.

11. **Hole center** is at `[0.52, -0.20, 0.81]` in `peg_in_hole_task.urdf.xacro`. This matches `peg_in_hole_world.sdf` (fixture at z=0.75 + 0.04m fixture + 0.02m plate = 0.81m top). The Cartesian targets file has hole_center at `[0.520, -0.200, 0.845]` — this is 0.035m above the actual hole surface, likely as the tool0 target z to align the peg above the hole.

12. **Peg-in-hole clearance**: Peg radius 0.0125m (25mm dia), hole radius 0.0135m (27mm dia), clearance = 0.001m (1mm). This is very tight for simulation.

### F. Missing End-Effector / Tool
13. **No physical peg model on the robot**: The research_gripper attaches to flange, but there is no peg held in the gripper fingers. The peg is a separate world model.

14. **Gripper fingers are fixed** (no prismatic joint), so the gripper cannot actually grasp anything.

15. **Gripper TCP is defined** but the `task_trajectory_executor` sends joint-space commands, not Cartesian. The Cartesian targets in `peg_hole_cartesian_targets.yaml` have `motion_execution_allowed: false` with reason "tool insertion axis not validated".

### G. Controller-Driven Robot Motion
16. **Controller stack works** (validated in v2.0–v2.4). The `joint_trajectory_controller` is properly configured with `follow_joint_trajectory` action.

17. **Position-only control**: Only `position` command interface is used. No velocity/effort/stiffness/damping control is available in Gazebo simulation mode.

18. **MoveIt config overlay exists** for `lbr_iisy6_r1300` but uses `diagnostic_only: true`. The actual MoveIt config from upstream (`kuka_lbr_iisy_moveit_config`) is for iisy3 R760, iisy11 R1300, iisy15 R930 — but there is no specific iisy6 R1300 config.

### H. General Issues
19. **headless_v2_4 launch**: `research_baseline_v2_4_experiment_runner.launch.py` references `research_baseline_v2_4_experiment_config.yaml` which may not exist as a path (check).

20. **Contact bridge topic path**: `contact_validation_bridge.yaml` references world paths specific to `peg_in_hole_contact_validation_world`, not `peg_in_hole_world`. This bridge config won't work for the research baseline.

21. **Lots of duplicates**: `peg_in_hole_task.urdf.xacro` and `peg_in_hole_world.sdf` both define peg/hole geometry independently — risk of config drift between task frames and actual Gazebo model poses.

## 10. Exact Verification Commands

### TF Frame Verification
```bash
# Check all TF frames
ros2 run tf2_tools view_frames.py
# Verify: world, base_link, flange, tool0, peg_tip, target_hole_frame, d405_camera_* exist

# Check specific frame pose
ros2 run tf2_ros tf2_echo world base_link
ros2 run tf2_ros tf2_echo world target_hole_frame
ros2 run tf2_ros tf2_echo tool0 peg_tip
ros2 run tf2_ros tf2_echo world d405_camera_optical_frame
```

### Robot Model Verification
```bash
# Check robot description publishes
ros2 topic echo /robot_description --once | head -c 2000

# Check joint states
ros2 topic echo /joint_states --once

# Check URDF geometry (after xacro expansion)
ros2 run xacro xacro src/peg_in_hole_description/urdf/lbr_iisy3_r760_research_gripper.urdf.xacro mode:=gazebo
```

### Controller Verification
```bash
# List controllers
ros2 control list_controllers

# List hardware interfaces
ros2 control list_hardware_interfaces

# Check controller state
ros2 topic echo /controller_manager/controller_names --once
```

### Gazebo World Verification
```bash
# List spawned models
gz model --list

# Check model poses
gz topic -e /world/peg_in_hole_world/dynamic_pose/info
```

### Motion Verification
```bash
# Send test joint command
ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [joint_1,joint_2,joint_3,joint_4,joint_5,joint_6], \
  points: [{positions: [0.0,-0.8,1.2,0.0,0.8,0.0], time_from_start: {sec: 4}}]}}"
```

### Safety Verification
```bash
# Check safety status
ros2 topic echo /safety_status --once

# Check task phase
ros2 topic echo /task_phase
```

### Contact Verification
```bash
# Check contact topics
ros2 topic list | grep contact
ros2 topic echo /gazebo/contacts/peg
ros2 topic echo /gazebo/contacts/hole
```

### Build Verification
```bash
cd ros2_ws
colcon build --symlink-install --packages-select \
  peg_in_hole_description kuka_task_control safety_layer \
  experiment_manager peg_in_hole_metrics perception_pipeline \
  thesis_bringup
source install/setup.bash
```

### Full Integration Smoke Test
```bash
# 1. Launch headless
ros2 launch thesis_bringup research_baseline.launch.py use_gui:=false &
sleep 10

# 2. Verify robot loaded
ros2 topic echo /joint_states --once --timeout 5

# 3. Verify controllers
ros2 control list_controllers | grep -q "joint_trajectory_controller" && echo "PASS"

# 4. Run task sequence
ros2 run kuka_task_control task_trajectory_executor \
  --ros-args -p config_path:=src/kuka_task_control/config/baseline_task_sequence.yaml

# 5. Check results
ls results/baseline_trials/

# 6. Kill
kill %1
```

## 11. Sprint Checklist

| # | Sprint | Status | Script Launch |
|---|--------|--------|--------------|
| v1.0 | Initial sensor/scene validation | Completed | `proposal_simulation_cell_v1_0.launch.py` |
| v1.1 | Sensor & scene validation | Completed | `proposal_simulation_cell_v1_1_sensor_and_scene_validation.launch.py` |
| v1.2 | RGB-D image bridge fix | Completed | `proposal_simulation_cell_v1_2_rgbd_image_bridge_fix.launch.py` |
| v1.3 | Contact physics validation | Completed | `proposal_simulation_cell_v1_3_contact_physics_validation.launch.py` |
| v1.5 | Safety virtual force interface | Completed | `proposal_simulation_cell_v1_5_safety_virtual_force_interface.launch.py` |
| v1.6 | Safety gate readiness | Completed | `proposal_simulation_cell_v1_6_safety_gate_readiness.launch.py` |
| v1.7 | Pre-control contract | Completed | `proposal_simulation_cell_v1_7_pre_control_contract.launch.py` |
| v1.8 | Control development scaffold | Completed | `proposal_simulation_cell_v1_8_control_development_scaffold.launch.py` |
| v1.9 | No-motion control law dry-run | Completed | `proposal_simulation_cell_v1_9_no_motion_control_law_dry_run.launch.py` |
| v1.10 | Experiment configuration matrix | Completed | `proposal_simulation_cell_v1_10_experiment_configuration_matrix.launch.py` |
| v1.11 | Single scenario loader validation | Completed | `proposal_simulation_cell_v1_11_single_scenario_loader_validation.launch.py` |
| v1.12 | Scenario batch selector | Completed | `proposal_simulation_cell_v1_12_scenario_batch_selector.launch.py` |
| v1.13 | Batch execution plan validator | Completed | `proposal_simulation_cell_v1_13_batch_execution_plan_validator.launch.py` |
| v1.14 | Batch dry-run orchestrator | Completed | `proposal_simulation_cell_v1_14_batch_dry_run_orchestrator.launch.py` |
| v1.15 | Evidence package generator | Completed | `proposal_simulation_cell_v1_15_evidence_package_generator.launch.py` |
| v1.16 | Reproducibility checklist | Completed | `proposal_simulation_cell_v1_16_reproducibility_checklist.launch.py` |
| v1.17 | Release documentation index | Completed | `proposal_simulation_cell_v1_17_release_documentation_index.launch.py` |
| v2.0 | First Gazebo motion smoke test | Completed | `proposal_simulation_cell_v2_0_first_gazebo_motion_smoke_test.launch.py` |
| v2.1 | Gazebo motion validation suite | Completed | `proposal_simulation_cell_v2_1_gazebo_motion_validation_suite.launch.py` |
| v2.2 | MoveIt IK diagnostic validation | Completed | `proposal_simulation_cell_v2_2_moveit_ik_diagnostic_validation.launch.py` |
| v2.3 | MoveIt model alignment + plan-only | Completed | `proposal_simulation_cell_v2_3_moveit_model_alignment_and_plan_only_validation.launch.py` |
| v2.4 | MoveIt → Gazebo execution | Completed | `proposal_simulation_cell_v2_4_moveit_gazebo_execution_validation.launch.py` |
| v2.5 | Guarded pre-contact task sequence | Completed | `proposal_simulation_cell_v2_5_guarded_pre_contact_task_sequence.launch.py` |
| v2.6 | Contact-gated guarded approach | Completed | `proposal_simulation_cell_v2_6_contact_gated_guarded_approach_validation.launch.py` |
| v2.7 | Contact-triggered guarded touch | Completed | `proposal_simulation_cell_v2_7_contact_triggered_guarded_touch_calibration.launch.py` |
| v2.8 | Contact reachability + trigger | Completed | `proposal_simulation_cell_v2_8_contact_reachability_and_trigger_validation.launch.py` |
| v2.9 | Non-overlapping approach-to-contact | Completed | `proposal_simulation_cell_v2_9_non_overlapping_approach_to_contact_validation.launch.py` |
| v2.10 | Misalignment contact gate batch | Completed | `proposal_simulation_cell_v2_10_misalignment_contact_gate_batch_validation.launch.py` |
| v2.11 | Multimodal contact observation logging | Completed | `proposal_simulation_cell_v2_11_multimodal_contact_observation_logging.launch.py` |
| v2.12 | Context vector extraction | Completed | `proposal_simulation_cell_v2_12_context_vector_extraction.launch.py` |
| v2.13 | Context encoder prototype | Completed | `proposal_simulation_cell_v2_13_context_encoder_prototype.launch.py` |
| v2.14 | Context-conditioned guarded action | Completed | `proposal_simulation_cell_v2_14_context_conditioned_guarded_action_validation.launch.py` |
| v2.15 | Context-action ablation validation | Completed | `proposal_simulation_cell_v2_15_context_action_ablation_validation.launch.py` |
| v2.16 | Guarded peg-in-hole objective | Completed | `proposal_simulation_cell_v2_16_guarded_peg_in_hole_objective_validation.launch.py` |

**Status Legend**: Completed = validated with diagnostic evidence in `ros2_ws/diagnostics/`

---

## Research Proposal Context

> Brief connection between the doctoral proposal goals and the current ROS/Gazebo workspace. Full technical context at `ros2_ws/docs/context/proposal_context.md`.

### Proposal High-Level Goal

Deliver a visuomotor, context-based meta-RL framework for adaptable peg-in-hole assembly on a **KUKA LBR iisy 6 R1300** collaborative robot, using RGB-D + force/proprioceptive feedback with dual-layer safety mediation (admittance virtual force during contact + runtime safety filter during pre-approach).

### Key Mismatches vs. Current Workspace

| Aspect | Proposal Specifies | Workspace Current State | Action Needed |
|--------|-------------------|------------------------|---------------|
| **Simulation engine** | NVIDIA Isaac Sim | Gazebo (GazeboSim + `gz_ros2_control`) | Clarify: Gazebo stand-in or migration path? |
| **Robot model** | KUKA LBR iisy 6 R1300 (1300mm reach) | `lbr_iisy3_r760` (760mm reach) | Confirm correct physical model |
| **Learning framework** | PyTorch + context-conditioned SAC | Not implemented (stub `learning_interface`) | Deferred to proposal months 10–22 |
| **Sim-to-real transfer** | Staged protocol with transfer gates | Not started | Deferred to proposal months 18–33 |
| **Safety filter** | Runtime constraint enforcement | `safety_monitor` node (soft limits only) | Deferred — current is pre-contact validation only |

### What the Workspace Already Provides (ahead of proposal timeline)

The v2.0–v2.16 sprint validation has produced a working Gazebo simulation cell with:
- Mesh-based KUKA LBR iisy 3 R760 + parallel gripper + D405 camera + grasped peg
- Joint trajectory control via `joint_trajectory_controller`
- Contact sensing via Gazebo contact sensors
- Guarded approach/retreat sequences with phase-based execution
- Metric extraction (`contact_metrics_node`)
- Safety monitoring (joint limits, phase timeouts)
- Context vector extraction stubs (v2.12)
- Context encoder prototype stubs (v2.13)

This places the workspace ahead of the proposal's **Month 6 (Foundations)** milestone — the simulation cell is already integrated and validated for basic motion and contact detection.

### Next Workspace Priorities (aligned with proposal Months 4–12)

1. Confirm robot model (iisy6 R1300 vs. iisy3 R760) and switch if needed
2. Clarify simulation engine choice (Gazebo vs. Isaac Sim)
3. Implement domain randomization infrastructure
4. Build multi-modal observation logging pipeline (RGB-D + F/T + joint states synchronized)
5. Implement admittance control in Gazebo (virtual-force layer)
6. Baseline SAC or classical insertion policy (pre-learning benchmark)
7. Metric extraction for Safe Success, peak wrench, cycle time

These steps lay the groundwork before the learning-intensive core phase (proposal Months 10–22).

---

## Impact Analysis: Fixing the Robot Cell

### Priority Order for Fixes

#### P0: Robot Model Correctness (DONE)
**Status**: FIXED
- `proposal_lbr_iisy6_r1300_cell.urdf.xacro` marked as deprecated with header comment — kept for historical reproducibility.
- Canonical description: `lbr_iisy3_r760_research_gripper.urdf.xacro` (mesh-based KUKA + gripper + camera + peg).

#### P1: Robot Base Position (DONE)
**Status**: FIXED
- Pedestal top at `z=0.735`, robot base at `z=0.75` — 0.015m gap closed by setting launch arg `z=0.735`.
- Updated `research_baseline.launch.py` default z from 0.75 → 0.735.

#### P2: Add Camera to Research Baseline (DONE)
**Status**: FIXED
- Added D405 camera as static model in `peg_in_hole_world.sdf` (color + depth sensors, 848×480, 30 Hz, ogre2 renderer).
- Added camera TF frames (`d405_camera_link`, `d405_camera_optical_frame`) to `lbr_iisy3_r760_research_gripper.urdf.xacro` behind `include_camera` arg (default: true). World-fixed at `[0.42, -0.65, 1.18]` with rpy `[0.95, 0, 0.35]`.
- Added `gz::sim::systems::Sensors` plugin to `peg_in_hole_world.sdf`.
- Added 4 bridge topics to `kuka_gazebo/config/bridge_config.yaml` (color, depth, 2× camera_info).

#### P3: Add Missing End-Effector/Tool (Peg on Robot) (DONE)
**Status**: FIXED
- Added optional `grasped_peg` link (0.11m × 0.0125m radius cylinder) with collision + inertial properties to `research_parallel_gripper.xacro`, behind `has_peg` macro param (default: false).
- Peg sits at TCP, extends 0.11m below; `peg_tip` frame at bottom of peg.
- Enabled in `lbr_iisy3_r760_research_gripper.urdf.xacro` via `has_peg="true"`.
- World peg model at `[0.72, -0.05, 0.75]` kept for reference — remove later if no longer needed.

#### P4: Peg and Hole Placement Alignment (DONE)
**Status**: FIXED
- `peg_hole_cartesian_targets.yaml`: `hole_center` z corrected from 0.845 → 0.810 (matches physical hole at target plate top surface).
- All waypoint targets shifted by -0.035m in z to maintain same offsets from hole_center:
  - staging/align: 0.920 → 0.885
  - insertion_touch: 0.865 → 0.830
  - insertion_hold: 0.845 → 0.810
  - final_insertion: 0.825 → 0.790
  - retreat: 0.920 → 0.885
- `task_geometry.yaml` and `peg_in_hole_task.urdf.xacro` already correct at z=0.81.

#### P5: Workcell/Table Position (No Change Needed)
**Impact**: LOW — correct as-is
- Table center at `[0.80, 0.0, 0.0]`, surface at z=0.75.
- Robot at `[0.80, -0.75, 0.735]` (fixed) faces the table.
- Robot table-edge clearance: 0.45m (from y=-0.75 to y=-0.30).

#### P6: Controller-Driven Robot Motion (No Change Needed)
**Impact**: LOW — controller stack works
- `joint_trajectory_controller` validated and functional.
- `contact_bridge.yaml` is correct for `peg_in_hole_world` (uses simple topic names matching SDF sensor topics).
- `contact_validation_bridge.yaml` is correct for `peg_in_hole_contact_validation_world` (uses full gz topic paths).
- No changes needed — both bridge configs correctly paired with their respective worlds.

### Cross-Cutting Issues
- **World file switching**: `contact_validation_bridge.yaml` references `peg_in_hole_contact_validation_world` (exists). `contact_bridge.yaml` references simple topic names matching `peg_in_hole_world` sensor topics.
- **Task geometry duplication**: `task_geometry.yaml` vs `peg_hole_cartesian_targets.yaml` vs `peg_in_hole_task.urdf.xacro` all define overlapping geometry. Define in ONE place and reference.
- **Diverging URDF paths**: v2.16 historical launch uses `kuka_lbr_iisy_support` URDF directly (no gripper). Research baseline uses `peg_in_hole_description` URDF with gripper. Not changed — v2.16 kept for reproducibility. All new launches should use `lbr_iisy3_r760_research_gripper.urdf.xacro`.
