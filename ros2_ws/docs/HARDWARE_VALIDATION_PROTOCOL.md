# Hardware Validation Protocol

Date: 2026-06-12
Status: FUTURE WORK -- All current results are simulation-only.
This document defines the protocol for deploying the validated simulation
pipeline to a physical KUKA LBR iisy 6 R1300.

---

## 1. Current State (Honest Assessment)

| Aspect | Status |
|--------|--------|
| Simulation trials completed | 140 (Stage C) + 60 baseline = 200 total |
| Simulation success rate | 88.3% baseline, 90% at 1.0mm clearance |
| Hardware trials completed | 0 |
| Sim-to-real transfer attempted | No |
| Physical KUKA available | Not confirmed |
| Physical F/T sensor available | Not confirmed |
| Physical gripper + peg available | Not confirmed |
| Domain randomization implemented | No |
| Hardware launch files exist | No |

**Everything below this line is a plan for future work, not a description
of completed activities.**

---

## 2. Required Hardware Setup

### 2.1 Robot

- **Model**: KUKA LBR iisy 6 R1300 (iiwa-compatible, 6-axis)
- **Controller**: KUKA Sunrise Cabinet (or Sunrise Office)
- **Firmware**: Sunrise 1.11 or later (confirmed compatible with ros2_control)
- **Mounting**: Pedestal mount, base at z=0.735m (matches simulation spawn height)
- **Safety**: Dual-channel E-stop, safety-rated monitored stop (SS1/SS2)

### 2.2 End-Effector

- **Gripper**: Parallel-jaw gripper (Schunk or equivalent)
  - Grip force: adjustable 5-50N
  - Stroke: >= 30mm (must clear 28mm peg diameter)
  - Repeatability: <= 0.02mm
- **F/T Sensor**: ATI Mini45 or equivalent
  - Mounted between wrist flange and gripper
  - Force range: >= 200N (z-axis), Torque range: >= 10 Nm
  - Resolution: <= 0.1N force, <= 0.005 Nm torque
  - Sample rate: >= 1 kHz (bridged to ROS2 at 500 Hz minimum)

### 2.3 Peg and Hole Fixture

- **Peg**: Cylindrical, hardened steel or aluminum
  - Diameter: 25.000mm +/- 0.005mm (baseline scenario)
  - Length: 110mm
  - Surface finish: Ra <= 0.8 um
  - Mounted rigidly in gripper, concentricity <= 0.01mm
- **Hole Fixture**: Precision-bored plate
  - Diameter: 27.000mm +/- 0.005mm (baseline scenario)
  - Plate thickness: 20mm
  - Material: hardened steel or aluminum
  - Mounted rigidly to work surface, perpendicularity <= 0.01mm

### 2.4 Camera

- **Model**: Intel RealSense D405 (matches simulation camera)
- **Mounting**: Fixed to work surface, viewing peg and hole from above/side
- **Calibration**: Intrinsics and extrinsics must be calibrated before use
- **Note**: Depth camera did not produce meaningful data in Gazebo; real D405
  should provide actual depth. This is a sim-to-real difference.

### 2.5 Compute

- **ROS2 Jazzy** on Ubuntu 24.04 (matches simulation environment)
- **Controller rate**: 25 Hz (matches simulation default)
- **F/T bridge**: `ros2_control` F/T state interface or custom bridge node
- **Network**: All nodes on localhost (no distributed latency)

---

## 3. ROS2/Gazebo-to-Hardware Mapping

### 3.1 Topics

| Simulation Topic | Hardware Equivalent | Msg Type | Notes |
|------------------|---------------------|----------|-------|
| `/joint_states` (from `joint_state_broadcaster`) | `/joint_states` (from `joint_state_broadcaster` on real HW) | `sensor_msgs/JointState` | Identical interface via `ros2_control` |
| `/joint_trajectory_controller/joint_trajectory` | `/joint_trajectory_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | Same controller stack on real HW |
| `/ft_sensor_wrench` (from Gazebo bridge) | `/ft_sensor_wrench` (from F/T sensor driver) | `geometry_msgs/Wrench` | Different source, same msg type |
| `/d405/color/image_raw` | `/camera/color/image_raw` (from D405 driver) | `sensor_msgs/Image` | Topic remapping required |
| `/d405/depth/image_rect_raw` | `/camera/depth/image_rect_raw` | `sensor_msgs/Image` | Topic remapping required |
| `/d405/color/camera_info` | `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | Topic remapping required |
| `/d405/depth/camera_info` | `/camera/depth/camera_info` | `sensor_msgs/CameraInfo` | Topic remapping required |
| `/task_phase` | `/task_phase` | `std_msgs/String` | Published by admittance controller (same) |
| `/safety_status` | `/safety_status` | `std_msgs/String` | Published by safety monitor (same) |
| `/insertion_state` | `/insertion_state` | `std_msgs/String` | Published by admittance controller (same) |

### 3.2 Services

| Simulation Service | Hardware Equivalent | Notes |
|--------------------|---------------------|-------|
| `/controller_manager/list_controllers` | `/controller_manager/list_controllers` | Identical via `ros2_control` |
| `/controller_manager/switch_controller` | `/controller_manager/switch_controller` | Identical via `ros2_control` |
| `/gazebo/spawn_entity` | N/A | Not needed (robot is already present) |

### 3.3 Parameters

| Simulation Parameter | Hardware Equivalent | Value | Notes |
|----------------------|---------------------|-------|-------|
| `use_sim_time: true` | `use_sim_time: false` | -- | Critical: must change for real HW |
| `control_rate: 10.0` | `control_rate: 10.0` | 10 Hz | May increase to 25 Hz after validation |
| `contact_threshold: 5.0` | `contact_threshold: 5.0` | 5 N | Tune on real hardware |
| `safety_threshold: 350.0` | `safety_threshold: 100.0` | 100 N | Lower on real HW for safety |
| `approach_speed: 0.01` | `approach_speed: 0.005` | 5 mm/s | Slower for first HW trials |
| `position_gain: 1000.0` | Tune via `ros2_control` YAML | -- | Depends on real joint stiffness |
| `search_recenter_duration_s: 5.0` | `search_recenter_duration_s: 8.0` | 8 s | Longer for real HW settling |
| `search_settle_duration_s: 6.0` | `search_settle_duration_s: 9.0` | 9 s | Longer for real HW settling |

### 3.4 Launch Files

The current `research_baseline.launch.py` must be adapted for hardware:
- Remove Gazebo-related nodes (`gz_sim`, `gz_server`, `spawn_robot`, `ros_gz_bridge`)
- Add `ros2_control` hardware component launch
- Add D405 camera driver launch
- Add F/T sensor driver launch
- Change `use_sim_time` to `false`
- Remap camera topics to real D405 topics
- Create `research_baseline_hardware.launch.py`

---

## 4. Safety Checklist

This checklist must be completed before any physical trial. Every item is
mandatory. No exceptions.

### 4.1 Robot Safety (Items 1-7)

- [ ] 1. Dual-channel E-stop circuit tested: press E-stop, robot stops
  within 50ms, release E-stop, robot does not resume automatically
- [ ] 2. SS1/SS2 safety function verified: safety controller monitors
  joint velocities and stops robot if any joint exceeds 1.5x rated speed
- [ ] 3. Joint soft limits configured in `ros2_control` YAML and verified
  against KUKA specification sheet (joint_1: +/-170 deg, etc.)
- [ ] 4. Cartesian workspace boundary defined: tool tip must not exceed
  x=[0.3, 0.7], y=[-0.4, 0.0], z=[0.7, 1.1] (relative to robot base)
- [ ] 5. Collision detection enabled on Sunrise Cabinet: max torque
  threshold set to 80% of rated joint torque
- [ ] 6. Maximum joint velocity limited to 50% of rated speed for initial
  trials (increased only after 10 successful trials at reduced speed)
- [ ] 7. Gravity compensation verified: robot holds position with no
  external force applied, drift < 0.1mm over 60s

### 4.2 F/T Sensor Safety (Items 8-10)

- [ ] 8. F/T sensor zero offset calibrated with no load (peg not in gripper)
- [ ] 9. F/T sensor wiring secured: no strain on cable during full range
  of robot motion
- [ ] 10. F/T sensor overload protection configured: software abort at
  100N force / 10 Nm torque (hardware limits should be higher)

### 4.3 Gripper Safety (Items 11-13)

- [ ] 11. Gripper open/close tested at reduced force (10N) before full
  force operation
- [ ] 12. Peg grip verified: gripper holds peg at 30N grip force, robot
  performs 6-axis motion at 50% speed, peg does not slip
- [ ] 13. Gripper loss-of-grip detection: if gripper reports open during
  INSERT phase, immediate abort

### 4.4 Fixture Safety (Items 14-16)

- [ ] 14. Hole fixture rigidly mounted: apply 50N lateral force to fixture,
  displacement < 0.01mm
- [ ] 15. Peg concentricity verified: rotate gripper 360 deg at safe
  height, peg tip wobble < 0.02mm (measured with dial indicator)
- [ ] 16. Work surface clearance verified: robot can reach all waypoints
  (AXIS_ALIGN, TOUCH, HOLD, FINAL_INSERTION) without collision

### 4.5 Software Safety (Items 17-20)

- [ ] 17. Safety monitor node running and publishing `/safety_status`
  at >= 5 Hz
- [ ] 18. Admittance controller safety gate tested: command force >
  safety_threshold, controller aborts within 1 control cycle
- [ ] 19. Timeout logic verified: if any phase exceeds max_phase_duration,
  trial aborts and robot retreats to SAFE_HOME
- [ ] 20. Emergency stop callback registered: ROS2 lifecycle transition
  to error state triggers robot stop and retreat

### 4.6 Environment Safety (Items 21-23)

- [ ] 21. Work area clear of all non-essential objects (tools, cables,
  papers) within 1m radius of robot
- [ ] 22. Emergency stop buttons accessible from all operator positions
  (minimum 2 E-stop locations)
- [ ] 23. Operator PPE verified: safety glasses, no loose clothing or
  jewelry near robot

---

## 5. F/T Sensor Calibration Procedure

### Prerequisites
- F/T sensor mounted between wrist flange and gripper
- Gripper open (no peg held)
- Robot at SAFE_HOME position

### Steps

1. **Record zero-offset baseline**
   - Command robot to remain stationary at SAFE_HOME
   - Record 500 F/T samples (5 seconds at 100 Hz minimum)
   - Compute mean force: `F_offset = mean(Fx, Fy, Fz)` over all samples
   - Compute mean torque: `T_offset = mean(Tx, Ty, Tz)` over all samples
   - Store offset values in calibration file

2. **Verify gravity compensation**
   - Move robot slowly to 3 different orientations (SAFE_HOME, AXIS_ALIGN,
     TOUCH position)
   - At each position, record 200 F/T samples
   - The gravity vector changes with orientation; Fz should change
     proportionally to the z-component of gravity in the sensor frame
   - If gravity-induced force exceeds 2N after offset subtraction,
     gravity compensation model is incorrect -- recalibrate

3. **Verify noise floor**
   - With robot stationary, record 1000 F/T samples
   - Compute standard deviation of Fx, Fy, Fz
   - Acceptable: sigma < 0.2N for force, sigma < 0.002 Nm for torque
   - If noise exceeds limits, check cable shielding and grounding

4. **Verify contact detection**
   - With robot at TOUCH position, manually apply known force to
     gripper (push against calibrated weight or force gauge)
   - Compare measured force from F/T sensor with applied force
   - Acceptable: error < 5% for forces in 1-50N range

5. **Log calibration results**
   - Write offset values, noise statistics, and verification results
     to `hardware_calibration/ft_sensor_calibration.yaml`

---

## 6. Tool/Peg Calibration Procedure

### Prerequisites
- Peg mounted in gripper
- Dial indicator or touch probe available

### Steps

1. **Measure peg tip position relative to flange**
   - Mount dial indicator on work surface
   - Move robot so peg tip touches dial indicator at multiple orientations
   - Record joint positions at each contact point
   - Compute peg tip position in tool frame: `P_tool = [x, y, z]`
   - Store in calibration file

2. **Verify peg axis alignment**
   - Move robot in straight-line motion along Z-axis at 5mm/s
   - Record peg tip displacement with dial indicator
   - If lateral displacement > 0.02mm over 50mm travel, peg is misaligned
   - Re-grip or re-machine if necessary

3. **Measure peg length**
   - Use calipers or touch probe to measure peg length
   - Compare with simulation value (0.11m)
   - Update `peg_length_m` parameter if different

4. **Verify gripper centering**
   - Open gripper, re-grip peg, measure tip position
   - Repeat 5 times
   - Repeatability must be <= 0.05mm (standard deviation)

---

## 7. Hole/Workpiece Calibration Procedure

### Prerequisites
- Hole fixture mounted on work surface
- Touch probe or calibration pin available

### Steps

1. **Measure hole center position**
   - Use touch probe to measure hole center in X and Y
   - Record hole center position in world frame: `H_world = [x, y, z]`
   - Compare with simulation value (0.52, -0.20, 0.81)
   - Update `hole_center_x`, `hole_center_y`, `hole_top_z` parameters

2. **Measure hole diameter**
   - Use pin gauges or CMM to measure hole diameter at 3 depths
   - Record minimum and maximum diameter
   - Update `hole_radius_m` parameter if different from nominal

3. **Verify perpendicularity**
   - Insert calibration pin into hole
   - Measure pin tilt with dial indicator at 2 heights
   - If tilt > 0.5mm over 20mm, fixture is not perpendicular
   - Re-machine or shim fixture

4. **Measure hole depth**
   - Use depth gauge to measure hole depth
   - Update `nominal_insertion_depth_m` parameter

---

## 8. Emergency Stop Requirements and Testing

### 8.1 Requirements

- **Response time**: Robot must stop all joint motion within 50ms of
  E-stop signal
- **State preservation**: After E-stop, joint positions must be held
  (not released)
- **Resume policy**: Robot must NOT resume automatically after E-stop
  release. Operator must explicitly command resume.
- **ROS2 integration**: Safety monitor must detect E-stop state and
  publish `safety_status: "E_STOPPED"` on `/safety_status`
- **Logging**: All E-stop events must be logged with timestamp, robot
  state, and F/T readings at time of trigger

### 8.2 Testing Protocol

1. Place robot at AXIS_ALIGN position (above table, away from fixture)
2. Start admittance controller in MOVING_TO_START phase
3. Press E-stop during motion
4. Verify: robot stops within 50ms, no further motion
5. Verify: safety_status publishes "E_STOPPED"
6. Release E-stop
7. Verify: robot does not resume motion
8. Verify: E-stop event logged
9. Repeat from TOUCH position (closest to fixture)
10. Repeat from SEARCH position (peg at hole height)
11. Repeat from INSERT position (peg inside hole)

---

## 9. First-Contact Test Protocol

This is the first time the robot makes physical contact with the workpiece.

### Prerequisites
- All safety checklist items (Section 4) passed
- F/T sensor calibrated (Section 5)
- Peg and hole calibrated (Sections 6, 7)
- Emergency stop tested (Section 8)
- Robot at SAFE_HOME

### Procedure

1. **Dry run (no contact)**
   - Run the full task sequence at 25% speed
   - Robot moves through all waypoints without contacting the hole
   - Verify all Cartesian positions are within 1mm of expected
   - If any waypoint is > 2mm off, abort and recalibrate

2. **Slow approach**
   - Reduce approach_speed to 2 mm/s
   - Command robot to TOUCH position (5mm above hole surface)
   - Monitor F/T readings: Fz should be within noise floor (no contact)
   - If Fz > 5N at TOUCH position, the Z calibration is wrong -- abort

3. **First contact**
   - Command robot to descend 2mm below TOUCH position (slowly, 1 mm/s)
   - Monitor F/T: Fz should increase upon contact
   - If Fz > 20N, abort immediately (peg may be hitting fixture, not hole)
   - If Fz is in 5-20N range, contact is valid -- log and retreat

4. **Verify contact location**
   - Compare measured contact Z with expected hole_top_z
   - If offset > 1mm, recalibrate hole Z position
   - Log contact force profile

---

## 10. Slow Guarded Insertion Test

### Prerequisites
- First-contact test passed (Section 9)
- Contact force profile is within expected range

### Procedure

1. **Set conservative limits**
   - Safety threshold: 50N (reduced from 350N)
   - Approach speed: 1 mm/s
   - Search recenter duration: 10s (extended)
   - Search settle duration: 12s (extended)

2. **Single guarded insertion**
   - Run the full task sequence (MOVING_TO_START -> APPROACH -> SEARCH ->
     INSERT -> RETREAT -> DONE)
   - Monitor forces continuously:
     - Fz during APPROACH: should be < 5N (no contact)
     - Fz during SEARCH: may spike to 10-30N (searching in hole)
     - Fz during INSERT: should be < 50N (insertion force)
     - Lateral forces (Fx, Fy): should be < 20N during INSERT
   - If any force exceeds safety_threshold, abort and retreat

3. **Abort conditions**
   - Fz > 50N at any point
   - Fx or Fy > 30N during INSERT
   - Insertion depth < 5mm after 30s of INSERT phase
   - Any joint velocity exceeds limits
   - E-stop pressed

4. **Success criteria**
   - Peg inserted to at least 10mm depth
   - All forces within limits throughout
   - Robot retreats successfully
   - Final XY error < 1mm

5. **Log all data**
   - F/T readings at 500 Hz
   - Joint positions at 500 Hz
   - Task phase transitions with timestamps
   - Insertion depth at 1 Hz

---

## 11. Geometry/Tolerance Hardware Validation Matrix

### Scenarios from Stage C (Simulation-Validated)

Each scenario must be tested on hardware with the same 20-trial protocol
used in simulation.

| Scenario | Peg | Hole | Clearance | Offset | Sim Rate | HW Trials Required |
|----------|-----|------|-----------|--------|----------|---------------------|
| baseline_loose | 25mm | 27mm | 1.0mm | 0mm | 80% | 20 |
| clearance_medium | 25mm | 26mm | 0.5mm | 0mm | 0% | 10 (reduced) |
| clearance_tight | 25mm | 25.5mm | 0.25mm | 0mm | 0% | 5 (reduced) |
| large_peg_large_hole | 28mm | 30mm | 1.0mm | 0mm | 95% | 20 |
| small_peg_small_hole | 22mm | 24mm | 1.0mm | 0mm | 100% | 20 |
| misaligned_baseline | 25mm | 27mm | 1.0mm | 1mm | 85% | 20 |
| tight_plus_misaligned | 25mm | 25.5mm | 0.25mm | 1mm | 0% | 5 (reduced) |

**Total minimum hardware trials**: 100

### Notes on Scenario Selection

- Tight clearance scenarios (0.25mm, 0.5mm) are expected to fail on
  hardware based on simulation results (0% success). They are included
  to characterize the real-world operating envelope.
- Reduced trial counts for tight scenarios: these are exploratory, not
  primary validation targets.
- The primary hardware validation target is 1.0mm clearance scenarios.

---

## 12. Data Logging Requirements

### 12.1 Perception Data

| Data | Source | Rate | Format |
|------|--------|------|--------|
| RGB image | D405 camera | 20 Hz | PNG (compressed) |
| Depth image | D405 camera | 20 Hz | 16-bit PNG |
| Joint positions | joint_state_broadcaster | 500 Hz | CSV |
| Joint velocities | joint_state_broadcaster | 500 Hz | CSV |
| F/T wrench | F/T sensor driver | 500 Hz | CSV |
| Task phase | admittance_controller | 10 Hz | CSV |
| Safety status | safety_monitor | 10 Hz | CSV |

### 12.2 Trajectory Data

| Data | Source | Rate | Format |
|------|--------|------|--------|
| Commanded joint positions | joint_trajectory_controller | 500 Hz | CSV |
| Actual joint positions | joint_state_broadcaster | 500 Hz | CSV |
| Tracking error | trajectory_tracking_observer | 10 Hz | CSV |
| Cartesian position | admittance_controller | 25 Hz | CSV |

### 12.3 Force Data

| Data | Source | Rate | Format |
|------|--------|------|--------|
| Raw F/T wrench | F/T sensor | 500 Hz | CSV |
| Contact force estimate | admittance_controller | 25 Hz | CSV |
| Gravity baseline | admittance_controller | 25 Hz | CSV |
| Contact sensor events | contact_state_observer | 10 Hz | CSV |

### 12.4 Trial Metadata

| Data | Source | Format |
|------|--------|--------|
| Trial ID | experiment_manager | YAML |
| Scenario ID | launch arguments | YAML |
| Start/end timestamp | experiment_manager | YAML |
| Outcome (success/failure) | admittance_controller | YAML |
| Failure reason | admittance_controller | YAML |
| Duration per phase | admittance_controller | YAML |
| Peak force during insertion | wrench_state_observer | YAML |
| Final insertion depth | admittance_controller | YAML |

---

## 13. Pass/Fail Criteria Per Scenario

### 13.1 Overall Pass Criteria

A scenario PASSES if:
- Success rate >= 75% for 1.0mm clearance scenarios
- Success rate >= 50% for 0.5mm clearance scenarios
- Success rate >= 25% for 0.25mm clearance scenarios (exploratory)
- No safety threshold violations in any trial
- No hardware damage in any trial
- All failures are handled safely (no uncontrolled motion)

### 13.2 Per-Scenario Targets

| Scenario | Pass Threshold | Sim Rate | Notes |
|----------|---------------|----------|-------|
| baseline_loose | 15/20 (75%) | 80% | Primary target |
| clearance_medium | 5/10 (50%) | 0% | Expect degradation |
| clearance_tight | 1/5 (20%) | 0% | Exploratory |
| large_peg_large_hole | 15/20 (75%) | 95% | Primary target |
| small_peg_small_hole | 15/20 (75%) | 100% | Primary target |
| misaligned_baseline | 12/20 (60%) | 85% | Primary target |
| tight_plus_misaligned | 0/5 (0%) | 0% | Exploratory |

### 13.3 Per-Trial Pass Criteria

A single trial PASSES if:
- Peg inserted to at least 10mm depth
- All forces within safety limits throughout
- Robot retreats successfully to SAFE_HOME
- No timeout (trial completes within 300s)
- Final XY error < clearance radius

### 13.4 Per-Trial Failure Modes

| Failure Mode | Description | Severity |
|--------------|-------------|----------|
| force_exceeded | F/T reading exceeded safety threshold | HIGH |
| timeout | Trial exceeded 300s deadline | MEDIUM |
| sideload_abort | Lateral force exceeded limit during INSERT | HIGH |
| xy_exceeded | Pre-insertion XY error > clearance | MEDIUM |
| handoff_timeout | SEARCH-to-INSERT handoff failed | MEDIUM |
| joint_limit | Joint reached soft limit | HIGH |
| e_stop | Emergency stop triggered | HIGH |
| gripper_loss | Gripper lost grip on peg | HIGH |

---

## 14. Risks and Mitigation

| Risk | Likelihood | Impact | Mitigation |
|------|-----------|--------|------------|
| Sim-to-real gap larger than expected | HIGH | HIGH | Start with conservative limits, increase gradually. Compare sim vs real force profiles. |
| F/T sensor noise on real hardware | MEDIUM | HIGH | Low-pass filter, increase deadband, recalibrate per session. |
| Peg/hole misalignment worse than sim | MEDIUM | MEDIUM | Precise calibration (Section 6, 7), use visual servoing for initial alignment. |
| Joint backlash not modeled in sim | MEDIUM | MEDIUM | Characterize backlash offline, compensate in controller gains. |
| Camera calibration drift | LOW | MEDIUM | Recalibrate before each session, use fixed mounting. |
| Gripper grip inconsistency | LOW | HIGH | Monitor grip force, retry grip if inconsistent. |
| Robot thermal drift | LOW | LOW | Allow 30min warmup, monitor joint temperatures. |
| Cable/connector failures | LOW | HIGH | Secure all cables, test connections before each session. |
| Unexpected contact during approach | MEDIUM | HIGH | Slow approach (1-2 mm/s), continuous F/T monitoring, immediate abort. |
| Hardware fatigue from repeated trials | LOW | MEDIUM | Limit to 20 trials per session, inspect peg/hole after each session. |

---

## 15. Estimated Timeline

### Week 1: Hardware Setup and Calibration
- Days 1-2: Robot installation, controller configuration, E-stop testing
- Days 3-4: F/T sensor mounting and calibration (Section 5)
- Day 5: Peg/gripper calibration (Section 6)

### Week 2: Fixture Calibration and First Contact
- Days 1-2: Hole fixture calibration (Section 7)
- Day 3: Safety checklist completion (Section 4)
- Day 4: Dry run (no contact), slow approach tests
- Day 5: First-contact test (Section 9)

### Week 3: Baseline Validation
- Days 1-3: Slow guarded insertion tests (Section 10)
- Days 4-5: baseline_loose scenario trials (20 trials)

### Week 4: Multi-Scenario Validation
- Days 1-2: small_peg_small_hole and large_peg_large_hole scenarios
- Day 3: misaligned_baseline scenario
- Day 4: clearance_medium scenario (reduced trials)
- Day 5: Data analysis, documentation, comparison with simulation

**Total estimated duration**: 4 weeks (20 working days)

### Contingency
- Add 1-2 weeks if sim-to-real gap is larger than expected
- Add 1 week if hardware issues require repairs/replacement
- Add 1 week if F/T sensor calibration is problematic

---

## 16. Documentation Requirements

After hardware validation, produce:
- Hardware validation results table (same format as Stage C simulation results)
- Sim-to-real comparison: simulation success rate vs hardware success rate per scenario
- Force profile comparison: simulation F/T traces vs hardware F/T traces
- Updated operating envelope (if different from simulation)
- Lessons learned and recommendations for future hardware experiments
