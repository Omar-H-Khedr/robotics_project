# Final Limitations and Next Steps

Date: 2026-06-10

## Honest Limitations

### 1. Simulation Only — No Hardware Validation

All results are from Gazebo Harmonic simulation on a KUKA LBR iisy 6 R1300. No physical hardware was used. Sim-to-real transfer is a known open challenge in robotics, especially for contact-rich manipulation tasks. The 88.3% success rate may not transfer directly to hardware.

### 2. Success Rate Is Strong But Not Perfect

53/60 (88.3%) across 60 automated trials. 7 failures occurred:
- **6 side-load aborts**: peg entered hole at slight XY offset, triggered safety gate
- **1 timeout**: trial exceeded 600s deadline without completing

All failures were handled safely by the deterministic controller. No damage, no uncontrolled motion.

### 3. DONE Precision = 3.4% (Structural Limitation)

The v2_14 classifier predicts DONE with only 3.4% precision. Root cause:
- **Context similarity**: RETREAT and DONE phases produce near-identical sensor readings (post-insertion, low forces, similar joint positions)
- **Class imbalance**: DONE has 16 training rows (0.08%) vs RETREAT with 2,107 (9.2%)
- **Impact**: The guarded advisory system blocks ALL DONE predictions from ML. DONE is only confirmed by the deterministic task state machine.

This is a structural limitation of per-tick classification without temporal context. It would require sequence models (e.g., LSTM, Transformer) or more DONE training data to resolve.

### 4. INSERT Remains Fully Deterministic

The v2_14 advisory system never controls INSERT. INSERT always defers to the deterministic admittance controller. This is by design:
- INSERT is the most safety-critical phase (contact forces, precision alignment)
- ML confidence is irrelevant when physical safety is at stake
- The deterministic controller has been validated at 87.5% success rate

### 5. SAC Training Not Completed

The SAC baseline is scaffolded but not trained locally. Reason:
- Gazebo physics simulation at 25Hz requires ~40ms per step
- 1M-5M training steps would take 11-55 hours of pure simulation
- Neural network updates require additional compute
- No GPU cluster was available locally

The SAC environment contract, reward function, and termination conditions are validated via mock rollouts. Full Gazebo training is deferred to a GPU cluster deployment.

### 6. Single Peg/Hole Variant — PARTIALLY ADDRESSED

Multiple peg/hole geometries and clearance levels are now parameterized and validated:
- 3 peg diameters: 22mm, 25mm, 28mm (cylindrical)
- 4 hole diameters: 24mm, 26mm, 27mm, 30mm (circular)
- 3 clearance levels: 0.25mm, 0.5mm, 1.0mm
- 7 scenarios, 22 trials, 91% overall success

Remaining gaps:
- Non-circular geometries (square peg/hole) not implemented
- Friction/material variation not tested
- Stage C (140 trials) needed for statistical confidence
- Cross-scenario train/test generalization not evaluated

### 7. F/T Sensor Bridge Crash

The `ft_sensor_bridge` package exits with SIGSEGV at startup in every trial. F/T features are zero in the 68-dim context vector. The admittance controller receives F/T data through `gz_ros_control` separately, so the physical task still works. But the ML classifier does not use F/T features.

### 8. Depth Camera Not Active in Simulation

The D405 depth camera does not produce meaningful depth data in Gazebo simulation. Depth features in the context vector are synthetic. This limits the classifier's ability to use visual depth for phase discrimination.

### 9. Encoder Pre-Training Is a Negative Result

The v2_13 autoencoder (68→32→68) was designed to learn a compressed representation. It achieved 92.4% accuracy but 0% SEARCH recall. The raw 68-dim context achieves 99.91% with all classes above 0.996 F1. The encoder bottleneck destroys information needed for fine-grained phase discrimination. This is a documented negative ablation.

### 10. No Automated Domain Randomization

Simulation parameters (joint friction, contact stiffness, camera noise) are fixed. No automated domain randomization was implemented. This limits the system's robustness to distribution shift.

## What Is Next

### Immediate (Geometry/Tolerance — Stage C)
- Run Stage C: 20 trials per scenario = 140 new trials
- Evaluate cross-scenario generalization (train on some, test on held-out)
- Extend to non-circular geometries if Gazebo SDF supports them

### Short-Term (GPU Cluster)
- Train SAC agent on Gazebo peg-in-hole task (1M-5M steps)
- Compare SAC vs deterministic baseline vs v2_14 advisory
- Implement domain randomization for sim-to-real preparation

### Medium-Term (Hardware)
- Deploy to physical KUKA LBR iisy 6 R1300
- Calibrate sim-to-real transfer
- Validate contact-aware insertion on real peg/hole
- Collect real-world data for classifier fine-tuning

### Long-Term (Thesis)
- Context-conditioned meta-RL for multi-variant assembly
- Multi-variant peg/hole generalization
- Publication of safety-gated advisory framework
- Publication of encoder negative ablation result
