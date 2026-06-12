# Sim-to-Real Transfer Plan

Date: 2026-06-12
Status: FUTURE WORK -- All current results are simulation-only. This
document defines the plan for transferring the validated simulation pipeline
to physical hardware.

---

## 1. Current Simulation Stack (What Exists Today)

### 1.1 Simulation Environment

| Component | Implementation | Version |
|-----------|---------------|---------|
| Physics engine | Gazebo Harmonic | Harmonic |
| Robot model | KUKA LBR iisy 6 R1300 | Custom URDF/SDF |
| Control interface | `gz_ros2_control` | 0.4+ |
| Joint controller | `joint_trajectory_controller` | ros2_control |
| F/T sensor | Simulated via Gazebo contact sensor | Bridged via `ros_gz_bridge` |
| Camera | Intel RealSense D405 model (static in world SDF) | Bridged via `ros_gz_bridge` |
| Task state machine | `admittance_insertion_node` (Python) | Custom |
| Safety layer | `safety_monitor` (Python) | Custom |
| Perception | `multimodal_observation_logger` | Custom |

### 1.2 Control Architecture

- **Controller**: Deterministic admittance controller (`admittance_insertion_node.py`)
- **State machine**: 7 states (IDLE -> MOVING_TO_START -> APPROACH -> SEARCH -> INSERT -> RETREAT -> DONE)
- **Control rate**: 10 Hz (task logic), 500 Hz (joint trajectory controller)
- **Key gains**: position_gain=1000, position_derivative_gain=0, joint_damping_scale=1.0 (canonical); position_gain=3000, derivative=10, damping=10 (tuned for geometry matrix)
- **Safety**: 350N force threshold, joint soft limits, timeout logic

### 1.3 What Is Validated in Simulation

| Aspect | Evidence | Confidence |
|--------|----------|------------|
| Task state machine correctness | 200+ trials, 7 states verified | HIGH |
| SEARCH spiral convergence | 100% convergence rate at 1.0mm clearance | HIGH |
| Safety gate behavior | 140 trials, all failures handled safely | HIGH |
| Geometry/tolerance envelope | 140 trials across 7 scenarios | HIGH |
| Admittance controller stability | No oscillations, no divergence | HIGH |
| v2_14 classifier offline accuracy | 99.98% on held-out simulation data | HIGH |

### 1.4 What Is NOT Validated in Simulation

| Aspect | Risk Level | Notes |
|--------|-----------|-------|
| Contact dynamics fidelity | HIGH | Gazebo contact model is approximate |
| F/T sensor noise | HIGH | Simulated F/T is noiseless |
| Joint friction/damping | HIGH | SDF friction model is simplified |
| Camera noise | HIGH | D405 depth data is synthetic in Gazebo |
| Sim-to-real latency | MEDIUM | Gazebo bridge adds variable latency |
| Joint backlash | MEDIUM | Not modeled in SDF |
| Gravity effects on real arm | MEDIUM | Simulation gravity model is idealized |

---

## 2. Known Sim-to-Real Gaps

### 2.1 Contact Dynamics Fidelity

**Gap**: Gazebo Harmonic uses a penalty-based contact model (ODE or DART).
The contact stiffness, damping, and friction parameters are approximate.
Real metal-on-metal contact between peg and hole involves:
- Elastic deformation (Hertzian contact)
- Plastic deformation at high forces
- Surface roughness effects (Ra 0.8 um)
- Coulomb friction with velocity-dependent coefficient

**Impact**: The simulation may underestimate insertion forces, overshoot
contact stiffness, or fail to capture stick-slip behavior during insertion.

**Measured in simulation**: Contact forces are smooth and predictable.
In reality, insertion forces will be noisier and may exhibit spikes.

### 2.2 Sensor Noise Models

**Gap**: Simulated F/T sensor provides perfect wrench data. Real F/T sensors
have:
- Bias drift (thermal, time-dependent)
- White noise (sigma ~ 0.1-0.5N depending on sensor)
- Cross-axis coupling (1-3% of reading)
- Quantization error (ADC resolution)

**Impact**: The contact detection threshold (5N) may need adjustment.
The gravity baseline computation (running median) will need tuning for
real noise characteristics.

**Current status**: The `admittance_insertion_node` uses a deadband and
median filter for contact estimation, which should be robust to moderate
noise. But the thresholds have not been validated against real sensor data.

### 2.3 Joint Friction/Damping

**Gap**: The SDF model uses simplified joint damping. Real KUKA joints have:
- Viscous friction (velocity-dependent)
- Coulomb friction (constant, opposes motion)
- Stribeck effect (friction decrease at low velocity)
- Joint-specific variation (each joint has different friction)

**Impact**: The admittance controller's position gain and derivative gain
were tuned for simulation dynamics. On real hardware, the same gains may
produce oscillations (if too aggressive) or sluggish response (if too
conservative).

**Evidence from simulation**: The controller works well at position_gain=3000,
derivative=10, damping=10. These gains may not transfer directly.

### 2.4 Camera Noise/Calibration

**Gap**: The D405 depth camera in Gazebo produces synthetic depth data.
Real D405 has:
- Depth noise: sigma ~ 1-3mm at 0.5m distance
- Minimum depth distance: ~70mm (blind zone)
- Multi-path interference on shiny surfaces
- IR pattern interference with peg/hole surfaces

**Impact**: The v2_14 classifier uses depth features (dimensions 48-53
of the 68-dim context vector). These features will be different on real
hardware. The classifier may need retraining or fine-tuning.

**Current status**: The depth camera was not active in Gazebo simulation
(depth features are synthetic). Real D405 data will be genuinely new
information for the classifier.

### 2.5 F/T Sensor Calibration

**Gap**: The simulated F/T sensor provides raw wrench without calibration
requirements. Real F/T sensors require:
- Zero-offset calibration (per session)
- Gravity compensation (per orientation)
- Thermal calibration (warm-up drift)
- Sensitivity matrix calibration (cross-axis)

**Impact**: If calibration is incorrect, the contact force estimates will
be wrong, leading to false contact detection or missed contacts.

### 2.6 Simulated vs Real Robot Dynamics

**Gap**: The KUKA LBR iisy 6 R1300 in Gazebo uses `gz_ros2_control` which
provides position-level control. Real KUKA robots use Sunrise Cabinet
controllers with:
- Internal force control mode (not used here, but affects dynamics)
- Joint-level safety monitoring (SS1/SS2)
- Communication latency (Ethernet to Sunrise Cabinet)
- Gravity compensation built into the controller

**Impact**: The robot's response to position commands may differ. The
admittance controller sends position commands; the real KUKA controller
may add its own filtering or limits.

---

## 3. Domain Randomization Plan

Domain randomization varies simulation parameters during training to improve
sim-to-real transfer. This plan defines the randomization ranges.

### 3.1 Joint Friction

| Parameter | Distribution | Range | Unit |
|-----------|-------------|-------|------|
| Joint damping | uniform(0.1, 0.5) | [0.1, 0.5] | Nm/(rad/s) |
| Joint Coulomb friction | uniform(0.1, 0.3) | [0.1, 0.3] | Nm |
| Joint viscous friction | uniform(0.01, 0.05) | [0.01, 0.05] | Nm/(rad/s) |

**Implementation**: Modify SDF joint damping and friction parameters
per trial. The admittance controller should be robust to these variations.

### 3.2 Contact Dynamics

| Parameter | Distribution | Range | Unit |
|-----------|-------------|-------|------|
| Contact stiffness | uniform(1000, 10000) | [1000, 10000] | N/m |
| Contact damping | uniform(10, 100) | [10, 100] | Ns/m |
| Contact friction coefficient | uniform(0.2, 0.6) | [0.2, 0.6] | -- |

**Implementation**: Modify Gazebo world SDF contact parameters per trial.

### 3.3 Sensor Noise

| Parameter | Distribution | Range | Unit |
|-----------|-------------|-------|------|
| F/T force noise | Gaussian(0, 0.5) | sigma=0.5 | N |
| F/T torque noise | Gaussian(0, 0.01) | sigma=0.01 | Nm |
| F/T force bias | uniform(-1, 1) | [-1, 1] | N |
| F/T torque bias | uniform(-0.02, 0.02) | [-0.02, 0.02] | Nm |
| Joint position noise | Gaussian(0, 0.0005) | sigma=0.0005 | rad |

**Implementation**: Add noise injection nodes between sensor topics and
subscribers. Apply noise after Gazebo publishes clean data.

### 3.4 Camera Noise

| Parameter | Distribution | Range | Unit |
|-----------|-------------|-------|------|
| RGB pixel noise | Gaussian(0, 0.01) | sigma=0.01 | normalized [0,1] |
| Depth noise | Gaussian(0, 0.003) | sigma=0.003 | m |
| Camera pose perturbation | uniform(-0.01, 0.01) | [-0.01, 0.01] | m (per axis) |

**Implementation**: Add image noise injection node. Perturb camera-to-world
transform per trial.

### 3.5 Peg/Hole Geometry Variation

| Parameter | Distribution | Range | Unit |
|-----------|-------------|-------|------|
| Peg diameter | uniform(-0.0001, 0.0001) | +/-0.1mm | m |
| Hole diameter | uniform(-0.0001, 0.0001) | +/-0.1mm | m |
| Peg position (XY) | uniform(-0.0005, 0.0005) | +/-0.5mm | m |
| Hole position (XY) | uniform(-0.0005, 0.0005) | +/-0.5mm | m |

**Implementation**: Modify SDF geometry and spawn position per trial.

### 3.6 Randomization Schedule

- **Phase 1 (no randomization)**: Baseline validation, 20 trials
- **Phase 2 (light randomization)**: Only sensor noise + joint friction,
  50 trials
- **Phase 3 (full randomization)**: All parameters, 100 trials
- **Phase 4 (adversarial)**: Worst-case combinations, 20 trials

---

## 4. Transfer Validation Protocol

### Step 1: Dry Run (No Contact)

**Goal**: Verify robot motion matches simulation without any contact.

**Procedure**:
1. Run the full task sequence with hole fixture removed
2. Robot moves through all waypoints: SAFE_HOME -> AXIS_ALIGN -> TOUCH ->
   HOLD -> FINAL_INSERTION -> retreat
3. Record Cartesian positions at each waypoint
4. Compare with simulation waypoint positions

**Pass criteria**:
- All Cartesian positions within 2mm of simulation values
- Joint tracking error < 1 degree at each waypoint
- No oscillations or jerky motion
- Task completes without timeout

**Duration**: 1 day (setup + 5 runs)

### Step 2: Slow Approach (Monitor Forces)

**Goal**: Verify force monitoring works with real F/T sensor.

**Procedure**:
1. Robot approaches TOUCH position at 2 mm/s
2. Monitor F/T readings throughout approach
3. Verify Fz is within noise floor (no contact)
4. Command robot to descend 1mm below TOUCH (first contact)
5. Verify Fz increases upon contact
6. Retreat immediately

**Pass criteria**:
- F/T readings are stable (no drift, no spikes > 5N during free motion)
- Contact detected within 1mm of expected Z position
- Contact force < 20N
- Robot retreats successfully

**Duration**: 1 day (setup + 10 approach/contact cycles)

### Step 3: Single Trial (Baseline Geometry)

**Goal**: Complete one full insertion with monitoring.

**Procedure**:
1. Set conservative limits: safety_threshold=50N, approach_speed=1 mm/s
2. Run the full task sequence
3. Monitor all forces and positions continuously
4. If any force exceeds 50N, abort and retreat
5. If insertion completes, log all data

**Pass criteria**:
- Trial completes (success or safe failure)
- No safety threshold violations
- No hardware damage
- All data logged successfully

**Duration**: 1 day (setup + 1-3 trials with extended monitoring)

### Step 4: 10-Trial Baseline

**Goal**: Establish hardware baseline success rate.

**Procedure**:
1. Run 10 trials with baseline geometry (25mm peg, 27mm hole, 1.0mm clearance)
2. Use same parameters as simulation baseline
3. Log all data per trial
4. Compute success rate, force profiles, timing

**Pass criteria**:
- >= 7/10 (70%) success rate
- All failures handled safely
- Force profiles are consistent across trials
- No progressive degradation (peg/hole wear)

**Duration**: 1-2 days (10 trials with setup and analysis)

### Step 5: Multi-Scenario Validation

**Goal**: Validate across geometry/tolerance scenarios.

**Procedure**:
1. Run 20 trials per primary scenario (1.0mm clearance):
   - baseline_loose
   - large_peg_large_hole
   - small_peg_small_hole
   - misaligned_baseline
2. Run 10 trials per exploratory scenario:
   - clearance_medium (0.5mm)
3. Log all data per scenario
4. Compare with simulation results

**Pass criteria**:
- 1.0mm clearance scenarios: >= 75% success rate
- 0.5mm clearance scenarios: >= 50% success rate
- Results consistent with simulation trends

**Duration**: 3-5 days (70 trials across 5 scenarios)

---

## 5. Calibration Procedures

### 5.1 Camera Intrinsics/Extrinsics

**Intrinsics**:
1. Print checkerboard pattern (9x6, 25mm squares)
2. Capture 20+ images at different angles/distances
3. Use OpenCV `calibrateCamera` to compute intrinsic matrix and distortion
4. Reprojection error must be < 0.5 pixels
5. Store calibration in `hardware_calibration/camera_intrinsics.yaml`

**Extrinsics** (camera-to-world):
1. Mount AprilTag (36h11) on work surface at known position
2. Detect tag pose from camera images
3. Compute camera-to-world transform
4. Verify: tag position from camera matches known position within 2mm
5. Store calibration in `hardware_calibration/camera_extrinsics.yaml`

### 5.2 F/T Sensor Zero Offset

See Section 5 of HARDWARE_VALIDATION_PROTOCOL.md.

**Key additional notes for sim-to-real**:
- The zero offset may drift over time (thermal effects)
- Recalibrate at the start of each session (minimum)
- If offset drifts > 1N during a session, recalibrate and restart

### 5.3 Joint Position Offsets

**Procedure**:
1. Command each joint to a known position (use KUKA teach pendant)
2. Record `joint_states` position feedback
3. Compare with commanded position
4. Compute offset: `offset = feedback - commanded`
5. If offset > 0.001 rad for any joint, apply compensation in launch config
6. Store offsets in `hardware_calibration/joint_offsets.yaml`

**Verification**:
1. Command robot to SAFE_HOME
2. Measure joint positions with external encoder or dial indicator
3. Compare with `joint_states` feedback
4. Agreement within 0.001 rad

### 5.4 Peg/Hole Position in World Frame

See Sections 6 and 7 of HARDWARE_VALIDATION_PROTOCOL.md.

**Additional sim-to-real considerations**:
- The simulation hole position is (0.52, -0.20, 0.81) in world frame
- The real hole position must be measured and may differ by several mm
- Update `hole_center_x`, `hole_center_y`, `hole_top_z` parameters
- Verify by moving robot to expected position and checking alignment

---

## 6. Data Collection for Fine-Tuning

### 6.1 Real-World Perception Logs

**Purpose**: Collect real RGB/depth/joint/FT data for classifier fine-tuning.

**Format**: Same as simulation perception logs (CSV + PNG images).

**Required per trial**:
- RGB image at 20 Hz (PNG)
- Depth image at 20 Hz (16-bit PNG)
- Joint positions at 500 Hz (CSV)
- F/T wrench at 500 Hz (CSV)
- Task phase labels at 10 Hz (CSV)

**Minimum collection**:
- 10 successful trials with baseline geometry
- 5 failed trials (if available) for failure mode coverage
- Data from multiple sessions (to capture thermal drift)

### 6.2 Real-World Trajectory Logs

**Purpose**: Compare commanded vs actual trajectories for controller tuning.

**Format**: Same as simulation trajectory logs.

**Required per trial**:
- Commanded joint positions at 500 Hz
- Actual joint positions at 500 Hz
- Tracking error at 10 Hz
- Cartesian position at 25 Hz

**Analysis**:
- Compute RMS tracking error per joint
- Identify joints with highest tracking error
- Compare with simulation tracking error
- Adjust controller gains if tracking error > 2x simulation value

### 6.3 Comparison with Simulation

**Purpose**: Quantify sim-to-real gap.

**Metrics to compare**:

| Metric | Simulation Value | Hardware Value | Gap | Acceptable? |
|--------|-----------------|----------------|-----|-------------|
| Success rate (baseline) | 80% | TBD | -- | >= 75% |
| Mean insertion force | TBD | TBD | -- | < 2x sim |
| Mean insertion time | TBD | TBD | -- | < 2x sim |
| Peak force | TBD | TBD | -- | < 150% sim |
| Tracking error (RMS) | TBD | TBD | -- | < 2x sim |
| SEARCH convergence rate | 100% | TBD | -- | >= 90% |

**If gaps exceed acceptable limits**:
1. Identify root cause (sensor noise, friction, calibration, etc.)
2. Add targeted domain randomization for that parameter
3. Re-tune controller gains
4. Re-collect hardware data
5. Re-compare

---

## 7. Risks and Mitigation

| Risk | Likelihood | Impact | Mitigation |
|------|-----------|--------|------------|
| Gazebo contact model too different from real physics | HIGH | HIGH | Start with loose clearance (1.0mm). Characterize real contact forces before attempting tight clearance. Add contact dynamics randomization. |
| F/T sensor noise causes false contact detection | HIGH | MEDIUM | Tune deadband and median filter on real data. Increase contact_threshold if needed. Log noise statistics. |
| Joint friction causes tracking degradation | HIGH | MEDIUM | Characterize real friction offline. Retune position_gain and derivative_gain. Consider impedance control. |
| Camera calibration error shifts perception features | MEDIUM | HIGH | Recalibrate intrinsics/extrinsics per session. Use fixed camera mount. Log calibration quality metrics. |
| Peg/hole position offset > 2mm | MEDIUM | MEDIUM | Precise measurement with touch probe. Update parameters. Consider visual servoing for initial alignment. |
| Robot communication latency affects control | MEDIUM | MEDIUM | Profile real latency. Increase controller rate if needed. Use `ros2_control` hardware interface for lower latency. |
| Gripper grip inconsistency | LOW | HIGH | Monitor grip force. Retry grip if inconsistent. Use parallel-jaw gripper with force control. |
| Progressive wear of peg/hole | LOW | MEDIUM | Inspect after each session. Replace if wear > 0.01mm. Use hardened materials. |
| Thermal drift during long sessions | LOW | LOW | Allow 30min warmup. Recalibrate F/T sensor periodically. Monitor joint temperatures. |
| Domain randomization ranges too narrow | MEDIUM | MEDIUM | Start with simulation, expand ranges based on real-world measurements. Use Bayesian optimization for range tuning. |

---

## 8. Estimated Timeline

### Phase 1: Preparation (Weeks 1-2)

| Task | Duration | Dependencies |
|------|----------|--------------|
| Hardware setup (robot, controller, E-stop) | 3 days | Robot availability |
| F/T sensor mounting and calibration | 2 days | F/T sensor availability |
| Camera mounting and calibration | 1 day | D405 camera |
| Peg/hole fixture installation | 1 day | Fixture machining |
| Safety checklist completion | 1 day | All hardware |
| ROS2 hardware launch file creation | 2 days | All calibration |
| Dry run testing (Step 1) | 1 day | Launch file |

### Phase 2: First Contact (Week 3)

| Task | Duration | Dependencies |
|------|----------|--------------|
| Slow approach testing (Step 2) | 1 day | Phase 1 complete |
| Single trial testing (Step 3) | 1 day | Step 2 passed |
| 10-trial baseline (Step 4) | 2 days | Step 3 passed |
| Data analysis and controller tuning | 1 day | Step 4 data |

### Phase 3: Multi-Scenario (Weeks 4-5)

| Task | Duration | Dependencies |
|------|----------|--------------|
| Baseline loose scenario (20 trials) | 2 days | Phase 2 complete |
| Large peg/large hole scenario (20 trials) | 2 days | Baseline complete |
| Small peg/small hole scenario (20 trials) | 2 days | Baseline complete |
| Misaligned baseline scenario (20 trials) | 2 days | Baseline complete |
| Medium clearance scenario (10 trials) | 1 day | All 1.0mm scenarios |
| Data analysis and sim comparison | 2 days | All scenarios |

### Phase 4: Domain Randomization and Fine-Tuning (Weeks 6-8)

| Task | Duration | Dependencies |
|------|----------|--------------|
| Implement domain randomization in Gazebo | 3 days | Phase 3 analysis |
| Train with domain randomization (GPU cluster) | 5-10 days | Randomization impl. |
| Collect additional real-world data for fine-tuning | 3 days | Phase 3 data |
| Fine-tune classifier on real data | 2 days | Real data collected |
| Validate fine-tuned classifier (shadow mode) | 2 days | Fine-tuning complete |

### Phase 5: Documentation (Week 8)

| Task | Duration | Dependencies |
|------|----------|--------------|
| Sim-to-real comparison report | 1 day | All data |
| Updated operating envelope | 1 day | All data |
| Recommendations for future work | 1 day | All analysis |

**Total estimated duration**: 8 weeks (40 working days)

### Contingency

- Add 2-4 weeks if sim-to-real gap is larger than expected
- Add 1-2 weeks if hardware issues require repairs
- Add 1-2 weeks if controller retuning requires extensive iteration
- Add 1 week if domain randomization training requires GPU cluster access

---

## 9. Success Criteria for Sim-to-Real Transfer

### 9.1 Minimum Viable Transfer

The transfer is considered **minimally successful** if:
- Robot completes at least 1 full insertion without damage
- Force profiles are qualitatively similar to simulation
- All safety mechanisms function correctly on real hardware
- Data is collected for offline analysis

### 9.2 Acceptable Transfer

The transfer is considered **acceptable** if:
- Baseline scenario success rate >= 75% (simulation: 80%)
- All 1.0mm clearance scenarios >= 60% success rate
- Force profiles within 2x simulation values
- No safety violations in any trial
- Controller gains transfer with < 50% adjustment

### 9.3 Excellent Transfer

The transfer is considered **excellent** if:
- Baseline scenario success rate >= 85% (simulation: 80%)
- All 1.0mm clearance scenarios >= 75% success rate
- Force profiles within 1.5x simulation values
- Controller gains transfer without adjustment
- v2_14 classifier achieves >= 80% agreement with simulation on real data

---

## 10. What Is NOT Yet Done (Honest Summary)

| Item | Status | What Would Be Needed |
|------|--------|---------------------|
| Physical KUKA robot | NOT AVAILABLE | Lab access, robot installation |
| Physical F/T sensor | NOT AVAILABLE | Sensor procurement, mounting |
| Physical gripper + peg | NOT AVAILABLE | Procurement, machining |
| Hole fixture | NOT AVAILABLE | Precision machining |
| Hardware ROS2 launch files | NOT CREATED | Adapt `research_baseline.launch.py` |
| Hardware calibration scripts | NOT CREATED | Write calibration utilities |
| Domain randomization in Gazebo | NOT IMPLEMENTED | SDF parameter modification per trial |
| Real-world data collection | NOT DONE | All hardware above |
| Classifier fine-tuning on real data | NOT DONE | Real data above |
| Sim-to-real gap characterization | NOT DONE | Hardware trials above |
| Controller gain transfer tuning | NOT DONE | Hardware testing |
| Contact dynamics model validation | NOT DONE | Force comparison data |

**This document is a plan, not a record of completed work.** All items
above require hardware access and future implementation.
