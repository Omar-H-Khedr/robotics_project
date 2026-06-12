# Supervisor Summary

Date: 2026-06-12
Status: PROPOSAL-READY

## What We Built

A complete ROS2/Gazebo peg-in-hole assembly system with:
- KUKA LBR iisy 6 R1300 in Gazebo Harmonic
- Deterministic admittance controller with safety gates
- v2_14 safety-gated phase classifier (99.98% offline accuracy)
- Geometry/tolerance scenario matrix (7 scenarios, 140 trials)
- Operating envelope characterization

## Key Results

### Deterministic Controller
- **88.3%** success rate (53/60 baseline trials)
- **90%** at loose clearance (1.0mm, 72/80 Stage C)
- **Fail-closed** at ≤0.5mm clearance (0/60 Stage C)

### Operating Envelope
- **Robust zone** (≥ 1.0mm clearance): 90% success
- **Marginal zone** (0.5mm): 0% success (fail-closed)
- **Out-of-envelope** (≤ 0.25mm): 0% success (fail-closed)
- **Hard limit**: Tracking noise floor (~0.5mm) prevents sub-mm insertion

### v2_14 Classifier
- **99.98%** offline accuracy (baseline, 6-phase)
- **92.2%** shadow-mode agreement
- **Safety-calibrated**: Multi-threshold profiles evaluated
- **Advisory-only**: Never controls insertion, never overrides safety gates

### Geometry/Tolerance Coverage
- **3 peg diameters**: 22mm, 25mm, 28mm (cylindrical)
- **4 hole diameters**: 24mm, 26mm, 27mm, 30mm (circular)
- **3 clearance levels**: 0.25mm, 0.5mm, 1.0mm
- **7 scenarios**: 140 trials, 51% overall success

## What We Learned

### 1. Operating Envelope Is Measurable
The deterministic controller operates reliably when radial clearance ≥ 2× tracking noise floor.
At 1.0mm clearance (2.0× noise), success rate is 90%.
At 0.5mm clearance (1.0× noise), success rate is 0%.
This is a measured operating envelope, not a bug.

### 2. Fail-Closed Behavior Is a Safety Property
The system correctly refuses insertion when positioning accuracy is insufficient.
This prevents jamming, damage, and unsafe contact forces.
The precontact clearance safety gate is working as designed.

### 3. Geometry-Only Features Are Insufficient for Safe Advisory
The feasibility classifier achieves 0% false-safe rate ONLY by blocking ALL feasible insertions.
Conservative profile: 0% false-safe, but 51.4% false-block rate.
This proves that row-level context data is essential for reduced false-safe rate.

### 4. Row-Level Data Is the Blocker
Stage C preserved only trial-level outcomes (2.3MB).
Row-level 68-dim context vectors require perception pipeline logging during Gazebo trials (~646MB/trial).
This blocks full v2_14 cross-scenario evaluation.

### 5. SAC/Meta-RL Remains Future Work
Scaffold implemented (scenario randomization, environment contract, reward function).
Not trained (requires GPU cluster, 1M-5M steps).
No trained SAC/meta-RL policy claimed.

## Honest Limitations

1. **Simulation only**: All results from Gazebo Harmonic, no hardware validation
2. **Sub-mm clearance**: Hard physical limit (tracking noise floor ~0.5mm)
3. **Row-level data gap**: Blocks full v2_14 cross-scenario evaluation
4. **Conservative classifier tradeoff**: 0% false-safe requires blocking all feasible insertions
5. **SAC not trained**: Scaffold only, requires GPU cluster
6. **No sim-to-real**: Domain randomization and real-world transfer unvalidated

## What's Next

### Immediate (Blocked by Row-Level Data)
- Collect row-level perception data for v2_14 cross-scenario evaluation
- 7 scenarios × 5-10 trials with perception logging
- ~4.5GB compressed context vectors for 70 trials

### Short-Term (GPU Cluster)
- Train SAC agent with scenario randomization
- Compare SAC vs deterministic baseline vs v2_14 advisory

### Medium-Term (Hardware)
- Deploy to physical KUKA LBR iisy 6 R1300
- Calibrate sim-to-real transfer
- Validate contact-aware insertion on real peg/hole

### Long-Term (Thesis)
- Context-conditioned meta-RL for multi-variant assembly
- Publication of safety-gated advisory framework

## Evidence Package

All evidence preserved in repository:
- `diagnostics/geometry_tolerance_matrix_stage_c/` — 2.3MB, 435 files
- 140 trial outcomes, operating envelope analysis, cross-scenario evaluation
- Safety-calibrated classifier metrics
- Generated SDF worlds (7 scenarios)

## Recommendation

The project is proposal-ready. The operating envelope result is a strong scientific contribution.
The fail-closed behavior at tight clearance is a safety property, not a failure.
The row-level data gap is a clear next step, not a blocker for the proposal.
