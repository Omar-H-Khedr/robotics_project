# Proposal Implementation Mapping

Date: 2026-06-09

This document maps each proposal deliverable to its current implementation status.

## Phase 1: Foundations (Months 1-6) — COMPLETED

| Proposal Deliverable | Status | Evidence |
|---|---|---|
| Fully integrated robotic cell in simulation | DONE | KUKA LBR iisy 6 R1300 + gripper + peg + hole fixture in Gazebo |
| Baseline joint trajectory execution | DONE | joint_state_broadcaster + joint_trajectory_controller via ros2_control |
| Contact sensing pipeline | DONE | Contact topics, F/T estimation, wrench_state_samples.csv |
| Multi-modal observation logging | DONE | multimodal_observation_logger: RGB-D + joint + F/T at 20 Hz |
| Metric extraction | DONE | Outcome JSON with depth, XY error, contact force, phase durations |
| Safe task sequence execution | DONE | Full task: MOVING_TO_START → APPROACH → SEARCH → INSERT → RETREAT → DONE |
| Reproducible launch | DONE | research_baseline.launch.py with parameterized config |

## Phase 2: Simulation + Representation (Months 4-12) — IN PROGRESS

| Proposal Deliverable | Status | Evidence |
|---|---|---|
| Calibrated simulation v1 | DONE | Gazebo (not Isaac Sim — see Open Issues) |
| Domain randomization | PARTIAL | Manual config overrides; no automated engine |
| Multi-modal observation pipeline | DONE | D405 RGB-D + joint states + F/T at 20 Hz |
| Context vector representation | DONE | v2_13: 68-dim (RGB + depth + joints + phase + safety) |
| Autoencoder pre-training | DONE | v2_13: 68→32→68, test_mse=0.003 |
| Action classifier | DONE | v2_14: 98.8% test accuracy, 5-phase classification |
| Ablation studies | DONE | v2_15: 5 variants, comprehensive per-class metrics |
| Simulation benchmarks | PARTIAL | Insertion success rate (10/10), cycle time, peak force tracked |
| Multi-variant pegs/holes | NOT DONE | Single variant: 25mm peg, 27mm hole |

## Phase 3: Core Learning (Months 10-22) — NOT STARTED

| Proposal Deliverable | Status | Evidence |
|---|---|---|
| SAC baseline training | NOT DONE | — |
| Context-conditioned meta-RL | NOT DONE | — |
| A1: No-context/fixed-context ablation | DONE (offline) | v2_15 ablation: raw 68-dim is optimal, encoder hurts |
| A2: Modality dropout | NOT DONE | — |
| A3: Safety ablations (admittance, filter) | NOT DONE | Safety filter is observer-only |

## Phase 4: Transfer + Safety (Months 18-33) — NOT STARTED

| Proposal Deliverable | Status | Evidence |
|---|---|---|
| Admittance-based virtual-force layer | NOT DONE | Position control only in Gazebo |
| Runtime safety filter (enforcement) | NOT DONE | safety_monitor is observer-only |
| Sim-to-real calibration | NOT DONE | — |
| Conservative real-robot deployment | NOT DONE | — |

## Phase 5: Benchmark + Thesis (Months 24-36) — NOT STARTED

| Proposal Deliverable | Status | Evidence |
|---|---|---|
| Final benchmarks | NOT DONE | — |
| Manuscripts | NOT DONE | — |
| Thesis | IN PROGRESS | Expose template exists |

## Key Implementation Metrics

### Physical Robustness Evidence

| Run | Trials | Success Rate | SEARCH Entry | SEARCH Convergence |
|---|---|---|---|---|
| Production-safe 10-trial | 10 | 100% (10/10) | 100% | 100% |
| Post-fix 10-trial | 10 | 90% (9/10) | 100% | 100% |
| **Combined** | **20** | **95% (19/20)** | **100%** | **100%** |

### Perception Pipeline Performance

| Component | Metric | Value |
|---|---|---|
| v2_13 Autoencoder | Test MSE | 0.00315 |
| v2_14 Action Classifier | Test Accuracy | 98.8% |
| v2_15 Best Variant (raw 68-dim) | Test Accuracy | 100% |
| v2_15 Best Variant | SEARCH Recall | 100% |

### Dataset Coverage

| Dataset | Rows | Phases | Missing |
|---|---|---|---|
| Multi-trial v2 (10 trials) | 15,555 | MOVING_TO_START, APPROACH, SEARCH, INSERT, UNKNOWN | RETREAT, DONE |

## Open Issues

### Critical

1. **Simulation engine mismatch**: Proposal specifies NVIDIA Isaac Sim; implementation uses Gazebo. This affects domain randomization, contact fidelity, and sim-to-real pipeline.

2. **F/T sensor bridge crash**: ft_sensor_bridge exits with SIGSEGV at startup in every trial. F/T features are zero in all perception logs. Admittance node receives F/T through gz_ros_control separately.

3. **RETREAT/DONE missing from perception log**: Logger subscription to /task_phase does not capture RETREAT or DONE phases despite the task completing successfully.

### Medium

4. **No admittance control**: Position control only in Gazebo. Admittance control is required by the proposal but cannot be implemented without velocity/effort interfaces.

5. **No runtime safety filter enforcement**: safety_monitor is observer-only. Enforcement mode is deferred.

6. **Single peg/hole variant**: Only one geometry (25mm peg, 27mm hole). Multi-variant generation not implemented.

7. **No automated domain randomization**: Manual config overrides only.

### Low

8. **Grasped peg centered at palm origin**: Physically unusual but does not affect simulation results.

9. **Task URDF geometry mismatch**: Controller uses HOLE_TOP_Z=0.810; SDF model values differ. Controller values are authoritative.

## Decision Log

| Date | Decision | Rationale |
|---|---|---|
| 2026-06-08 | Raw 68-dim context is the validated representation | Encoder pre-training hurts SEARCH recall (0% vs 100%) |
| 2026-06-08 | search_entry_threshold_m=0.0 as production default | Always enters SEARCH, providing robustness against positioning uncertainty |
| 2026-06-08 | SEARCH_CONVERGENCE_TICKS=4 | Calibrated to 500Hz gain=3000/D=10 physical limit |
| 2026-06-08 | INSERT_SHALLOW_SIDELOAD_RECOVERY_DEPTH_M=0.010 | Closes gap where side-load at 6-10mm depth was unrecoverable |
| 2026-06-08 | INSERT_PREDEPTH_RECENTER_MAX_ATTEMPTS=3 | Gives one more recenter chance for pre-depth drift |
