# Robot Cell Audit

> Workspace inspection audit. No source files modified. Based on direct file reads from `ros2_ws/` at commit time.

---

## 1. KUKA LBR iisy Model

### Package / Source

| Item | Path | Description |
|------|------|-------------|
| Upstream KUKA descriptions | `external/kuka_robot_descriptions/` | Forked from `kuka_robot_descriptions`; contains `kuka_lbr_iisy_support`, `kuka_gazebo`, `kuka_resources`, `kuka_lbr_iisy_moveit_config` |
| Primary research URDF | `src/peg_in_hole_description/urdf/lbr_iisy3_r760_research_gripper.urdf.xacro` | Includes upstream macro + custom gripper + camera TF frames |
| Upstream macro | `external/kuka_robot_descriptions/kuka_lbr_iisy_support/urdf/lbr_iisy3_r760_macro.xacro` | Mesh-based (STL) real KUKA LBR iisy 3 R760 |
| Deprecated cylinder placeholder | `src/peg_in_hole_description/urdf/proposal_lbr_iisy6_r1300_cell.urdf.xacro` | FAKE: cylindrical links, wrong kinematics. Header says DEPRECATED. |
| MoveIt overlay SRDF | `src/kuka_task_control/config/moveit_lbr_iisy6_r1300/lbr_iisy6_r1300.srdf` | **Candidate only** — copied from iisy11 R1300 template; not validated |

### URDF / Xacro Files

| File | Contents |
|------|----------|
| `lbr_iisy3_r760_macro.xacro` | 6 links (`base_link`, `link_1`…`link_6`) + 6 revolute joints + `base`, `flange`, `tool0` ROS-Industrial frames. Mesh-based collision/visual via STL files. |
| `lbr_iisy3_r760_research_gripper.urdf.xacro` | Includes macro, ros2_control, parallel gripper, camera TF, `world-base_link` joint. Sets `has_peg="true"`, `include_camera="true"`. |
| `research_parallel_gripper.xacro` | Macro: `research_parallel_gripper(prefix, parent_link, tcp_frame, has_peg)`. Fixed parallel-jaw gripper (no actuation). Optional grasped peg. |
| `peg_in_hole_task.urdf.xacro` | Defines `target_hole_frame` and nominal `peg_tip` frame. Separate from robot URDF. |
| `proposal_lbr_iisy6_r1300_cell.urdf.xacro` | DEPRECATED. 6 cylindrical links, wrong inertias/limits, no ros2_control. |

### Joint Structure (upstream iisy3 R760)

| Joint | Type | Parent → Child | Axis |
|-------|------|----------------|------|
| `joint_1` | revolute | `base_link` → `link_1` | (0,0,-1) |
| `joint_2` | revolute | `link_1` → `link_2` | (0,0,1) |
| `joint_3` | revolute | `link_2` → `link_3` | (0,0,1) |
| `joint_4` | revolute | `link_3` → `link_4` | (0,0,1) |
| `joint_5` | revolute | `link_4` → `link_5` | (0,0,1) |
| `joint_6` | revolute | `link_5` → `link_6` | (0,0,1) |

Fixed frames: `base_link-base`, `link6-flange` (flange), `link6-tool0` (tool0).

### Scale Assumptions

- Upstream macro uses 1:1 scale STL meshes from KUKA — **correct** for iisy3 R760.
- The research URDF adds a gripper with `has_peg="true"` — peg is 25mm dia × 110mm cylinder at TCP.
- **The macro has 6 DOF.** The proposal specifies "KUKA LBR iisy 6 R1300" which is also 6 DOF (the "6" in "iisy6" refers to the payload class). The `iisy6_r1300` and `iisy3_r760` differ in reach (1300mm vs 760mm) and possibly payload, but both are 6-axis arms.

### Base Link and Joint Structure — Consistency Assessment

The upstream macro is a **consistent, real mesh-based KUKA model** from the official `kuka_robot_descriptions` fork. No suspicious parameters found. All inertias, collision meshes, and joint limits are physically plausible.

**Suspicious items:**
1. The SRDF for iisy6 R1300 (`lbr_iisy6_r1300.srdf`) is explicitly marked as "derived from same-family iisy11 R1300 semantic model" — it is a **candidate that requires validation**. It has `diagnostic_only: true`.
2. The `peg_in_hole_task.urdf.xacro` defines a peg radius of 0.015m (15mm) and peg length 0.08m (80mm) in its properties — **these differ from the actual grasped peg** which is 0.0125m radius × 0.11m. The task URDF peg dimensions do not match the physical peg model. Potential config drift.
3. The `peg_in_hole_task.urdf.xacro` defines `hole_radius` as 0.016m (16mm) while the actual hole in the SDF models is 0.0135m radius (27mm dia). **Mismatch between task URDF geometry and Gazebo model geometry.**

---

## 2. Robot Base

### Spawn Position

| Parameter | Value | Source |
|-----------|-------|--------|
| Launch arg `x` | `0.80` | `research_baseline.launch.py` line 343 |
| Launch arg `y` | `-0.75` | line 344 |
| Launch arg `z` | `0.735` | line 345 (pedestal top plate surface) |
| Launch arg `roll` | `0` | line 346 |
| Launch arg `pitch` | `0` | line 347 |
| Launch arg `yaw` | `1.5708` | line 348 (90° rotation to face table) |

These values are passed as `xacro args` to the URDF's `world-base_link` joint (line 79–83 of the URDF):
```xml
<joint name="world-base_link" type="fixed">
  <parent link="world"/>
  <child link="base_link"/>
  <origin xyz="$(arg x) $(arg y) $(arg z)" rpy="$(arg roll) $(arg pitch) $(arg yaw)"/>
</joint>
```

The Gazebo spawn (`ros_gz_sim create`) additionally uses `-x 0.0 -y 0.0 -z 0.0 -R 0.0 -P 0.0 -Y 0.0` (lines 196–213 of the launch file). This means the spawn offset in Gazebo is zero — the robot is positioned entirely by the URDF's `world-base_link` joint. **This is correct.**

### Height Relative to Table / Floor

| Element | Z value | Calculation |
|---------|---------|-------------|
| Floor (ground plane) | z=0.0 | SDF ground plane |
| Pedestal model | Origin at world pose `[0.80, -0.75, 0.0]` | Bottom at z=0.0 |
| Pedestal column | 0.72m tall, center at z=0.36 → top at z=0.72 | |
| Pedestal top plate | 0.03m thick, center at z=0.735 → top at z=0.75 | |
| Robot base_link | Spawned at z=0.735 (pedestal top plate surface) | `world-base_link` joint |
| Table surface | z=0.75 | Table model, collision at z=0.725, thickness 0.05 → top at z=0.75 |

**The base_link is at z=0.735, which is 0.015m below the table surface (z=0.75).** This is intentional — the robot base sits on the pedestal top plate, and its own base geometry extends upward. The first joint (`joint_1`) origin is at z=0.1264 within the base_link frame, so the actual arm starts at z=0.735+0.1264=0.8614. The arm is well above the table.

**Verdict:** Robot base position is correct. No intersection with the table. The `base_link` is below the table surface, but this is expected — the base is a physical structure that sits lower than the tabletop.

---

## 3. Workcell / Table

### Table Dimensions

| Property | Value | Source |
|----------|-------|--------|
| Top size | 0.80m × 0.60m × 0.05m | `work_table/model.sdf` lines 8–9 |
| Legs | 4 legs, each 0.06×0.06×0.70m at (±0.34, ±0.24) | lines 24–62 |
| Total height | 0.75m (0.70m legs + 0.05m top) | Collision at z=0.725, top at z=0.75 |

### Table Pose

| Axis | Value | Source |
|------|-------|--------|
| World pose | `[0.80, 0.0, 0.0]` | `peg_in_hole_world.sdf` line 71 |
| Orientation | `[0, 0, 0]` | same |

Centered at (0.80, 0.0). The table spans x=[0.40, 1.20], y=[-0.30, 0.30].

### Collision Geometry

Table top collision is a box 0.80×0.60×0.05m at z=0.725 (center). Four leg collisions at the corners. Friction: μ=0.8. Surface material: not explicitly modelled.

### Visual Geometry

Same as collision — box + four leg boxes. Ambient/diffuse/specular colours set.

### Workspace Reachability

- Robot base at (0.80, -0.75, 0.735), facing +Y (yaw=1.5708).
- Table edge at y=-0.30 → clearance = 0.45m.
- Hole fixture at (0.52, -0.20, 0.75) — this is on the table surface (z=0.75) near the robot-side edge.
- iisy3 R760 has 760mm horizontal reach. The hole center is at x=0.52, y=-0.20 relative to world. In robot base frame (0.80,-0.75), the hole is at Δx=-0.28, Δy=+0.55 — about 0.62m from base. **Within reach of a 760mm arm.**

However: the yaw=1.5708 means the robot's "forward" (+Y in world) points toward the table. Joint-space poses in `baseline_task_sequence.yaml` reach joint_2 values as low as -1.40 rad — this puts the arm in a downward-and-forward posture that should reach the hole vicinity.

**Verdict:** The table is correctly placed, the hole is reachable with the iisy3 R760 arm.

---

## 4. Camera

### Camera Model / Source

| Aspect | Value |
|--------|-------|
| Model | Intel RealSense D405 |
| SDF model | `peg_in_hole_world.sdf` lines 117–163 — static model `d405_rgbd_camera` |
| URDF TF frames | `lbr_iisy3_r760_research_gripper.urdf.xacro` lines 59–77 (behind `include_camera` arg) |
| Sensor type | 2 sensors: `camera` (RGB, topic `/d405/color/image_raw`) + `depth_camera` (topic `/d405/depth/image_rect_raw`) |
| Resolution | 848×480 |
| Frame rate | 30 Hz |
| FOV | horizontal_fov=1.22 rad (~70°) |
| Clip | near=0.07m, far=2.0m |
| Render engine | ogre2 |

### Pose

| Parameter | SDF (line 119) | URDF TF (line 69) |
|-----------|----------------|-------------------|
| Pose | `0.42 -0.65 1.18 0.95 0 0.35` | Same (in URDF TF joint) |
| Description | ~54° downward tilt about X, ~20° yaw | Same |

The camera is at (0.42, -0.65, 1.18) in world frame — above and to the robot's left side, looking down and toward the table.

### Direction

Optical frame convention: ROS standard (z-forward, x-right, y-down). The optical frame is created with an offset `rpy="-1.5708 0 -1.5708"` relative to the camera link (URDF line 75).

**The camera pose is copied from the old prototype (`proposal_lbr_iisy6_r1300_cell.urdf.xacro` lines 129–133). It was never validated against the physical setup.** The approximately 54° downward tilt with the 0.35 rad yaw should point toward the hole area.

### Camera View of Peg / Hole Workspace

- Camera at (0.42, -0.65, 1.18).
- Hole center at (0.52, -0.20, 0.81).
- Vector from camera to hole: Δx=0.10, Δy=0.45, Δz=-0.37.
- Distance: ~0.60m — within the 2.0m far clip and above the 0.07m near clip.
- The camera should be able to see the peg and hole area.

### Bridge Topic Mismatch

**Critical issue:** The SDF camera publishes to Gazebo topics `/d405/color/image_raw` and `/d405/depth/image_rect_raw`. The bridge config (`kuka_gazebo/config/bridge_config.yaml` lines 17–32) bridges `/d405/color/image_raw` → ROS `/d405/color/image_raw` and `/d405/depth/image_rect_raw` → ROS `/d405/depth/image_rect_raw`. **However, the `perception_pipeline/config/rgbd_pipeline.yaml` expects topics `/camera/color/image_raw` and `/camera/depth/image_raw`.** There is a topic name mismatch. The perception pipeline is subscribed to topics that do not exist.

**Verdict:** Camera is present, positioned reasonably, and publishes data. But the perception pipeline expects different topic names than what is bridged. This is a configuration issue, not a hardware or modeling issue.

---

## 5. Peg and Hole

### Peg Model

| Aspect | Value | Source |
|--------|-------|--------|
| **World peg** | 25mm dia × 110mm cylinder | `cylindrical_peg/model.sdf` |
| **Grasped peg** | 25mm dia × 110mm cylinder | `research_parallel_gripper.xacro` lines 110–111 |
| World peg pose | `[0.72, -0.05, 0.75]` in world | `peg_in_hole_world.sdf` line 99 |
| World peg static | `static=false` — can fall | model.sdf line 4 |
| Grasped peg fixed | Fixed to gripper palm at origin | xacro joint `gripper_palm_to_grasped_peg` at (0,0,0) |
| Grasped peg material | 0.15kg, friction μ=0.35, contact stiffness kp=1e6, kd=10 | model.sdf lines 9–20; xacro lines 118–121 |
| Contact sensor | `/gazebo/contacts/peg` on world peg | model.sdf lines 22–28 |

### Hole Model

| Aspect | Value |
|--------|-------|
| **Hole fixture** | 0.18×0.18×0.04m square nest at `[0.52, -0.20, 0.75]` (world) |
| **Target plate** | 0.18×0.18×0.02m plate at `[0.52, -0.20, 0.79]` (world) |
| Hole opening | 27mm diameter (radius 0.0135m) on both fixture and plate |
| Hole top surface | z=0.81m (fixture bottom at 0.75 + 0.04m fixture + 0.02m plate) |
| Contact sensors | `/gazebo/contacts/hole` (fixture), `/gazebo/contacts/target` (plate) |

### Pose

- Hole frame (`target_hole_frame` in `peg_in_hole_task.urdf.xacro`): `[0.52, -0.20, 0.81]` — matches the physical hole at the top of the target plate.
- Nominal peg_tip in task URDF: 0.06m above hole frame.
- Peg_hole_cartesian_targets.yaml: `hole_center` at `[0.520, -0.200, 0.810]` — matches.
- Clearance: peg radius 0.0125m, hole radius 0.0135m → **1mm radial clearance**.

### Peg Starting Position — Realistic?

**The world peg floats at [0.72, -0.05, 0.75]** — this is on the table surface near the robot base. It is a separate dynamic model, not attached to the robot. **This is the peg that the robot would need to pick up** — but the gripper has fixed fingers that cannot close. There is no grasp sequence.

**The grasped peg on the robot** is fixed to the gripper palm at the TCP. It is always present. This means:
- The robot always has a peg attached.
- The world peg is a separate entity that is never used in the current task sequence.
- The task sequence (`baseline_task_sequence.yaml`) performs poses named `pre_grasp` and `grasp_approach` but these are meaningless — there is no grasping action.

**Verdict:** The peg is always attached to the robot via a fixed joint in the gripper. The separate world peg is vestigial. The current setup simulates "robot always has the peg in its gripper" — no pickup is required.

### Hole on Work Surface

Yes. The target plate is at z=0.79 to z=0.81, sitting on top of the fixture which sits on the table at z=0.75. The hole opening is at the top of the target plate at z=0.81.

---

## 6. End-Effector / Tool

### Is a Tool Attached?

**Yes.** The `research_parallel_gripper.xacro` attaches:
1. `gripper_palm` — fixed to `flange` at (0,0,0)
2. `gripper_left_finger` — fixed to palm at y=+0.031, z=0.075
3. `gripper_right_finger` — fixed to palm at y=-0.031, z=0.075
4. `gripper_tcp` (frame only) — fixed to palm at z=0.125
5. `grasped_peg` — fixed to palm at (0,0,0), extends below TCP
6. `peg_tip` (frame only) — at bottom of grasped peg

### Where Should It Attach?

The gripper attaches to `flange` (the ROS-Industrial flange frame defined in the upstream macro at line 198–203). This is correct.

The `has_peg` parameter is set to `true` in the research URDF (line 56).

### Missing Files or Missing Frames

| Item | Status |
|------|--------|
| `flange` frame | Defined in upstream macro |
| `tool0` frame | Defined in upstream macro |
| `gripper_tcp` frame | Defined in gripper xacro |
| `peg_tip` frame | Defined in gripper xacro (when `has_peg=true`) |
| `grasped_peg` collision | Defined and has collision + inertial |
| `grasped_peg` link origin | At (0,0,0) relative to palm — **peg extends 0.055m above and below palm centre** |
| `gripper_palm_to_grasped_peg` joint | At origin — **peg is centred at the palm origin, not at the TCP** |

**Frame chain:** `world → base_link → link_1→…→link_6 → flange → gripper_palm → grasped_peg → peg_tip`

**Issue with peg position:** The peg is attached at the palm origin (0,0,0), not at the TCP (z=0.125). The TCP frame `gripper_tcp` is 0.125m above the palm. The peg extends from -0.055m to +0.055m around the palm origin. This means:
- The top of the peg is 0.055m **below** the palm-top surface.
- The peg tip is at z = -0.055m relative to the palm.
- The TCP (gripper_tcp) is at z = +0.125m relative to palm.
- The tool0 frame is approximately at the flange, which is at the link_6 end via the fixed `link6-tool0` joint at `[0,0,-0.1605]` with rpy `[0, π, 0]`.

**This is physically unusual** — the peg should be below the TCP/gripper, centred along the gripper's centreline. The current attachment places the peg centre at the palm origin, meaning the peg extends both above and below the palm. In reality, the peg would be held in the fingers and protrude below.

### Minimum Viable Gripper / Tool for Current Sprint

The current setup is sufficient for the minimum demonstration:
- The robot has a grasped peg fixed to the end-effector.
- The peg has collision geometry.
- The peg_tip frame exists for Cartesian target calculation.
- No grasping actuation is needed for the baseline.

**Missing / needs confirmation:**
- Peg is not positioned at the TCP — it is centred at the palm. The `peg_tip` frame is `0.11m` below the peg centre, so peg_tip is at z=-0.055m in palm frame. The TCP is at z=+0.125m. The tip is 0.18m below the TCP. This is a large offset that must be accounted for in Cartesian planning.
- If the physical robot has a different gripper, the dimensions are pure assumptions.

---

## 7. Controllers

### Controller YAML

**File:** `external/kuka_robot_descriptions/kuka_resources/config/fake_hardware_config_6_axis.yaml`

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

**Key observations:**
- **Only position command interface.** No velocity, effort, or stiffness interfaces.
- **Only position state interface.** No velocity or effort feedback.
- 50 Hz state publish rate matches the proposal requirement.
- The `action_monitor_rate` is 20.0 Hz — fine for trajectory monitoring.

### joint_state_broadcaster

Standard `joint_state_broadcaster/JointStateBroadcaster`. Activated first in the launch sequence. Publishes `/joint_states`.

### joint_trajectory_controller

Standard `joint_trajectory_controller/JointTrajectoryController`. Activated second, after the broadcaster exits. Accepts `FollowJointTrajectory` goals.

### Action Server Name

`/joint_trajectory_controller/follow_joint_trajectory` — defined in `research_baseline.yaml` line 6 and used in `task_trajectory_executor.py` line 51.

### Is Motion Controller-Driven?

**Yes.** The launch sequence is:
1. `ros_gz_sim create` spawns the robot from `robot_description` topic.
2. After spawn completes: → `joint_state_broadcaster` spawner (activated).
3. After broadcaster exits: → `joint_trajectory_controller` spawner (activated).
4. The `task_trajectory_executor` (or `baseline_joint_sequence_executor`) sends `FollowJointTrajectory` goals to the action server.

All motion is through the ros2_control framework → GazeboSimSystem plugin. **No direct joint_command topics are used in the controlled flow.**

### ros2_control Hardware Plugin

From `lbr_iisy_ros2_control_macro.xacro`:
- `mode=gazebo` → `gz_ros2_control/GazeboSimSystem`
- `mode=mock` → `kuka_mock_hardware_interface`
- `mode=hardware` → real KUKA drivers

The research baseline uses `mode:=gazebo`.

---

## 8. Launch Flow

### Main Launch Files

| File | Role |
|------|------|
| `research_baseline.launch.py` | **Canonical launch.** 369 lines. Loads config, sets up world, spawns robot, starts bridge + controllers. |
| `proposal_simulation_cell_v2_16_guarded_peg_in_hole_objective_validation.launch.py` | Sprint v2.16 validation — uses upstream URDF directly (no gripper). Kept for reproducibility. |
| All other `proposal_simulation_cell_v2_*.launch.py` | Sprint-specific validation launches. |

### Spawn Sequence (research_baseline.launch.py)

```
1. Set GZ_SIM_RESOURCE_PATH to include peg_in_hole_description/models/
2. Launch gz_sim or gz_server (Gazebo GUI or headless) with world SDF
3. Start ros_gz_bridge (standard bridge: /clock, /joint_states, /cmd_vel, /d405/*)
4. Start contact_ros_gz_bridge (contact bridge: /gazebo/contacts/*)
5. Start robot_state_publisher (publishes /robot_description + TF)
6. ros_gz_sim create (spawn robot from /robot_description at [0,0,0] offset)
7. AFTER spawn exits: start joint_state_broadcaster spawner
8. AFTER broadcaster exits: start joint_trajectory_controller spawner
```

### Gazebo Startup

Uses `ros_gz_sim` package:
- **With GUI:** `gz_sim.launch.py` with world path + `-r -v1` arguments
- **Headless:** `gz_server.launch.py` with world SDF file + container args

### Controller Startup

Controllers are spawned by `controller_manager/spawner` executable:
- `joint_state_broadcaster` — activated immediately
- `joint_trajectory_controller` — activated immediately
- Both are daisy-chained via `OnProcessExit` event handlers

### Task-Control Launch Files

- `kuka_task_control/launch/` — contains any task executor launch files
- The executor runs as a standalone node, not as part of the Gazebo launch. It is started after the simulation is up:
  ```bash
  ros2 run kuka_task_control task_trajectory_executor
  ```

**Verdict:** Launch flow is well-structured. Sequential dependencies are properly handled via event handlers.

---

## 9. Current Problems

### P0: Robot Model Mismatch (Proposal vs Workspace)

| Observation | The proposal specifies KUKA LBR iisy 6 R1300 (1300mm reach). The workspace uses iisy3 R760 (760mm reach). The iisy3 R760 macro has 6 DOF with correct mesh-based geometry. The iisy6 R1300 has a candidate-only MoveIt SRDF derived from iisy11 R1300, never validated. |
|-------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| Likely cause | The workspace was set up before the proposal was finalised, or the iisy3 R760 is used for simulation development while the iisy6 R1300 is the planned physical robot. |
| Affected files | `peg_in_hole_description/urdf/lbr_iisy3_r760_research_gripper.urdf.xacro`, `external/kuka_robot_descriptions/kuka_lbr_iisy_support/urdf/lbr_iisy3_r760_macro.xacro`, `src/kuka_task_control/config/moveit_lbr_iisy6_r1300/*` |
| Priority | **HIGH** — Must confirm which robot is the target, as it affects URDF, kinematics, reachability, and MoveIt config. |

### P1: Grasped Peg Position Is Physically Unusual

| Observation | The grasped peg is attached at the palm origin (0,0,0), not at the TCP. The peg centre is at the palm centre, so the peg extends 0.055m above and below the palm. The TCP is 0.125m above the palm. The peg_tip is 0.18m below the TCP. |
|-------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| Likely cause | Simplified attachment for simulation convenience during early sprints. The peg was added quickly and positioned at the palm origin because that was the nearest convenient frame. |
| Affected files | `peg_in_hole_description/urdf/research_parallel_gripper.xacro` lines 133–136 |
| Priority | **MEDIUM** — Works for basic motion but will affect Cartesian peg_tip accuracy and visual consistency. |

### P2: Camera Bridge Topic Mismatch

| Observation | Camera publishes to `/d405/color/image_raw` and `/d405/depth/image_rect_raw`. Bridge relays these as `/d405/...`. But `perception_pipeline/config/rgbd_pipeline.yaml` expects `/camera/color/image_raw` and `/camera/depth/image_raw`. |
|-------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| Likely cause | The perception pipeline config was written before the D405 camera was added to the world. The topic names were assumed to match a generic camera convention (`/camera/...`), but the actual implementation uses D405-specific topic names (`/d405/...`). |
| Affected files | `perception_pipeline/config/rgbd_pipeline.yaml` lines 3–5, `peg_in_hole_world.sdf` lines 131, 148, `bridge_config.yaml` lines 17–32 |
| Priority | **HIGH** — Perception pipeline will never receive data until this is fixed. |

### P3: Task URDF Geometry Mismatch

| Observation | `peg_in_hole_task.urdf.xacro` defines `peg_radius=0.015` (15mm), `peg_length=0.08` (80mm), `hole_radius=0.016` (16mm). The actual peg model is 0.0125m radius (12.5mm) × 0.11m (110mm). The actual hole is 0.0135m radius (13.5mm). These do not match. |
|-------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| Likely cause | The task URDF was written independently from the SDF models. It was never validated against the actual Gazebo geometry. |
| Affected files | `peg_in_hole_description/urdf/peg_in_hole_task.urdf.xacro` lines 3–5 |
| Priority | **MEDIUM** — Only affects task-frame calculations if they use the URDF geometry values instead of SDF model geometry. |

### P4: Only Position Control Available

| Observation | The controller config (`fake_hardware_config_6_axis.yaml`) defines only `position` command and state interfaces. No velocity, effort, or stiffness interfaces. Admittance control (required by the proposal) cannot be implemented with position-only commands. |
|-------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| Likely cause | GazeboSimSystem supports only position interfaces for this joint type. Real KUKA hardware drivers may expose additional interfaces. |
| Affected files | `external/kuka_robot_descriptions/kuka_resources/config/fake_hardware_config_6_axis.yaml`, GazeboSimSystem plugin |
| Priority | **HIGH** for admittance control implementation — block the proposal's virtual-force layer if not resolved. **LOW** for current sprint. |

### P5: Cartesian Motion Blocked

| Observation | `peg_hole_cartesian_targets.yaml` has `motion_execution_allowed: false` with reason "tool insertion axis not validated". All waypoints are defined but cannot be executed. |
|-------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| Likely cause | The Cartesian tool axis was not yet validated when the config was last updated. The issue noted in P1 (peg_tip not at TCP) may also affect this. |
| Affected files | `kuka_task_control/config/peg_hole_cartesian_targets.yaml` lines 8–10 |
| Priority | **MEDIUM** — Blocks Cartesian insertion, but joint-space approaches can proceed. |

### P6: World Peg Is Vestigial

| Observation | A separate dynamic `cylindrical_peg` model floats at `[0.72, -0.05, 0.75]` in the world. The robot has its own peg fixed to the gripper. The world peg is never used. |
|-------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| Likely cause | The world peg was the original peg model from early validation. The grasped peg was added later (P3 fix) and made the world peg redundant. |
| Affected files | `peg_in_hole_description/worlds/peg_in_hole_world.sdf` lines 96–100, `peg_in_hole_description/models/cylindrical_peg/` |
| Priority | **LOW** — Harmless but confusing. Can be removed when the world is next updated. |

### P7: Safety Monitor Is Observer-Only

| Observation | The `safety_monitor` node publishes status but does not enforce any constraint. Phase timeout logs a warning but does not stop execution. No velocity, workspace, or force limits are enforced. This is explicitly stated in the code: "phase timeout policy is monitor-only in v0.2" (line 245). |
|-------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| Likely cause | The safety layer was implemented as a first-pass monitoring solution. Enforcement was deferred to later sprints. |
| Affected files | `safety_layer/safety_layer/safety_monitor.py` |
| Priority | **MEDIUM** — Meets the current sprint requirement (monitoring), but the proposal requires a runtime safety filter that enforces constraints. |

---

## 10. Recommended Smallest Safe Implementation Sprint

### Objective

Fix the highest-priority configuration issue (camera topic mismatch) and validate end-to-end perception pipeline connectivity, while documenting the robot model question for resolution.

### Exact Files Likely to Edit

| File | Change |
|------|--------|
| `perception_pipeline/config/rgbd_pipeline.yaml` (lines 3–5) | Change topic names from `/camera/...` to `/d405/...` to match actual bridge topics |
| Or alternatively: add topic remapping in launch config | Remap `/d405/...` → `/camera/...` via ROS 2 topic remapping |

**No URDF, no SDF, no launch logic changes.** Only the perception pipeline config topic names.

### Expected Result

The `perception_pipeline` node subscribes to the correct bridged camera topics. RGB and depth images appear in the ROS 2 topic list under the names expected by the pipeline.

### Verification Commands

```bash
# 1. Launch the baseline
ros2 launch thesis_bringup research_baseline.launch.py use_gui:=false

# 2. Check camera bridge is publishing
ros2 topic list | grep d405
# Expected: /d405/color/image_raw, /d405/depth/image_rect_raw, /d405/color/camera_info, /d405/depth/camera_info

# 3. Check perception pipeline receives data
ros2 topic hz /perception/task_state
# Expected: non-zero hz after pipeline starts

# 4. Check raw camera data
ros2 topic echo /d405/color/image_raw --once | head -c 100
# Expected: binary image data (not empty)

# 5. Run task sequence to verify camera integration
ros2 run kuka_task_control task_trajectory_executor
```

### README Evidence to Collect

| Evidence | How |
|----------|-----|
| Camera topic list | `ros2 topic list \| grep d405` terminal output |
| Image reception | Screenshot or log showing `/d405/color/image_raw` has publishers |
| Perception pipeline output | Log line showing `/perception/task_state` being published |
| Task sequence completion | Terminal output showing all 10 poses completed |
| TF tree | `ros2 run tf2_tools view_frames.py` — verify camera frames exist |

### Next Sprint After This One

Once camera topics are fixed and validated:

1. **Resolve robot model**: Confirm iisy6 R1300 vs iisy3 R760. If iisy6 R1300 is the target, acquire or generate the correct URDF/macro for iisy6 R1300.
2. **Fix peg attachment position**: Move the grasped peg so it protrudes below the TCP, not centred at the palm.
3. **Document the position-control limitation**: Verify if GazeboSimSystem can expose velocity interfaces, or plan an external admittance wrapper.

---

*Audit completed by reading all URDF, SDF, launch, config, and source files in `ros2_ws/src/peg_in_hole_description/`, `ros2_ws/src/thesis_bringup/`, `ros2_ws/src/kuka_task_control/`, `ros2_ws/src/safety_layer/`, `ros2_ws/src/perception_pipeline/`, `ros2_ws/src/peg_in_hole_metrics/`, `ros2_ws/src/external/kuka_robot_descriptions/`. No files were modified.*
