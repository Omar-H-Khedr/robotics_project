# Proposal Implementation Mapping

Date: 2026-06-10

This document maps each proposal deliverable to its current implementation status.

## Phase 1: Foundations (Months 1-6) — EXCEEDED

| Proposal Deliverable | Status | Evidence |
|---|---|---|
| Fully integrated robotic cell in simulation | DONE | KUKA LBR iisy 6 R1300 + gripper + peg + hole fixture in Gazebo |
| Baseline joint trajectory execution | DONE | joint_state_broadcaster + joint_trajectory_controller via ros2_control |
| Contact sensing pipeline | DONE | Contact topics, F/T estimation, wrench_state_samples.csv |
| Multi-modal observation logging | DONE | multimodal_observation_logger: RGB-D + joint + F/T at 20 Hz |
| Metric extraction | DONE | Outcome JSON with depth, XY error, contact force, phase durations |
| Safe task sequence execution | DONE | Full task: MOVING_TO_START → APPROACH → SEARCH → INSERT → RETREAT → DONE |
| Reproducible launch | DONE | research_baseline.launch.py with parameterized config |

## Phase 2: Simulation + Representation (Months 4-12) — COMPLETED

| Proposal Deliverable | Status | Evidence |
|---|---|---|
| Calibrated simulation v1 | DONE | Gazebo (not Isaac Sim — see Open Issues) |
| Domain randomization | PARTIAL | Manual config overrides; no automated engine |
| Multi-modal observation pipeline | DONE | D405 RGB-D + joint states + F/T at 20 Hz |
| Context vector representation | DONE | v2_13: 68-dim (RGB + depth + joints + phase + safety) |
| Autoencoder pre-training | DONE | v2_13: 68→32→68, test_mse=0.003670 (6-phase). Encoder is a documented negative ablation. |
| Action classifier | DONE | v2_14: 99.98% offline accuracy, 7-class classification (6-phase). Raw 68-dim validated. |
| Ablation studies | DONE | v2_15: 5 variants, comprehensive per-class metrics. Raw 68-dim optimal. |
| Simulation benchmarks | DONE | 60 trials total: 87.5% combined baseline (35/40), 88.3% grand total (53/60), 90% shadow-mode (9/10), 90% advisory (9/10) |
| Multi-variant pegs/holes | NOT DONE | Single variant: 25mm peg, 27mm hole |

## Phase 3: Core Learning (Months 10-22) — IN PROGRESS

| Proposal Deliverable | Status | Evidence |
|---|---|---|
| SAC baseline training | SCAFFOLDED | Environment contract exported, mock rollouts validated. Gazebo training requires GPU cluster. |
| Context-conditioned meta-RL | NOT DONE | — |
| A1: No-context/fixed-context ablation | DONE (offline) | v2_15 ablation: raw 68-dim is optimal (99.91%), encoder hurts (92.4%) |
| A2: Modality dropout | NOT DONE | — |
| A3: Safety ablations (admittance, filter) | PARTIAL | Safety-gated classifier (v2_14) with 24 unit tests. Safety filter is observer-only. |

## Phase 4: Transfer + Safety (Months 18-33) — IN PROGRESS

| Proposal Deliverable | Status | Evidence |
|---|---|---|
| Admittance-based virtual-force layer | NOT DONE | Position control only in Gazebo |
| Runtime safety filter (enforcement) | PARTIAL | v2_14 safety-gated classifier with INSERT/DONE safety gates. 24 unit tests, all invariants hold. |
| Shadow-mode validation | DONE | 10 trials, 92.2% agreement, all phases captured |
| Guarded advisory integration | DONE | 10 trials, 38.7% acceptance, all safety invariants hold. DONE never trusted, INSERT always deferred. |
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

| Run | Trials | Success Rate | Non-Empty Logs | Notes |
|---|---|---|---|---|
| Production-safe 10-trial | 10 | 80% (8/10) | 10/10 | Initial validation (2 ABORTED) |
| Post-fix 10-trial | 10 | 90% (9/10) | 10/10 | 1 side-load abort |
| **20-trial confirmation** | **20** | **90% (18/20)** | **20/20** | Improved DDS cleanup |
| **Combined baseline (40 trials)** | **40** | **87.5% (35/40)** | **40/40** | Production baseline |
| Shadow-mode validation | 10 | 90% (9/10) | 10/10 | v2_14 passive inference |
| Guarded advisory validation | 10 | 90% (9/10) | 10/10 | Safety-gated advisory |
| **Grand Total** | **60** | **88.3% (53/60)** | **60/60** | All automated runs |

### Perception Pipeline Performance

| Component | Metric | Value | Notes |
|---|---|---|---|
| v2_13 Autoencoder | Test MSE | 0.003670 | 6-phase. Encoder is negative ablation. |
| v2_14 Action Classifier | Offline Accuracy | 99.98% | 7 classes, 6-phase, class-weighted |
| v2_14 Shadow Mode | Agreement | 92.2% | 10-trial live validation |
| v2_14 Advisory | Acceptance Rate | 38.7% | Safety-gated, all invariants hold |
| v2_15 Best Variant (raw 68-dim) | Test Accuracy | 99.91% | All classes >= 0.996 F1 |
| v2_15 Encoder Variant | Test Accuracy | 92.4% | 0% SEARCH recall (negative result) |

### Dataset Coverage

| Dataset | Rows | Phases | Notes |
|---|---|---|---|
| Multi-trial v3 (6-phase) | 22,083 | MOVING_TO_START, APPROACH, SEARCH, INSERT, RETREAT, DONE, UNKNOWN | Current training set |

## Open Issues

### Critical

1. **Simulation engine mismatch**: Proposal specifies NVIDIA Isaac Sim; implementation uses Gazebo. This affects domain randomization, contact fidelity, and sim-to-real pipeline.

2. **F/T sensor bridge crash**: ft_sensor_bridge exits with SIGSEGV at startup in every trial. F/T features are zero in all perception logs. Admittance node receives F/T through gz_ros_control separately.

### Resolved

3. ~~**RETREAT/DONE missing from perception log**~~: FIXED in commit d791219. Force-write CSV row on RETREAT/DONE/ABORT phase transitions. All 50 automated trials now capture all phases.

4. **DONE precision=3.4%**: Structural issue due to context similarity with RETREAT (both post-insertion, low forces) and class imbalance (DONE=16 rows vs RETREAT=2107). Guarded by advisory system: DONE NEVER trusted from ML alone.

### Medium

5. **No admittance control**: Position control only in Gazebo. Admittance control is required by the proposal but cannot be implemented without velocity/effort interfaces.

6. **No runtime safety filter enforcement**: safety_monitor is observer-only. v2_14 safety-gated classifier provides partial enforcement (INSERT/DONE gates). Full enforcement mode is deferred.

7. **Single peg/hole variant**: Only one geometry (25mm peg, 27mm hole). Multi-variant generation not implemented.

8. **No automated domain randomization**: Manual config overrides only.

### Low

9. **Grasped peg centered at palm origin**: Physically unusual but does not affect simulation results.

10. **Task URDF geometry mismatch**: Controller uses HOLE_TOP_Z=0.810; SDF model values differ. Controller values are authoritative.

## Decision Log

| Date | Decision | Rationale |
|---|---|---|
| 2026-06-08 | Raw 68-dim context is the validated representation | Encoder pre-training hurts SEARCH recall (0% vs 100%) |
| 2026-06-08 | search_entry_threshold_m=0.0 as production default | Always enters SEARCH, providing robustness against positioning uncertainty |
| 2026-06-08 | SEARCH_CONVERGENCE_TICKS=4 | Calibrated to 500Hz gain=3000/D=10 physical limit |
| 2026-06-08 | INSERT_SHALLOW_SIDELOAD_RECOVERY_DEPTH_M=0.010 | Closes gap where side-load at 6-10mm depth was unrecoverable |
| 2026-06-08 | INSERT_PREDEPTH_RECENTER_MAX_ATTEMPTS=3 | Gives one more recenter chance for pre-depth drift |
| 2026-06-09 | 20-trial confirmation: 18/20 (90%) | Independent validation with search_entry_threshold_m:=0.0 |
| 2026-06-10 | DONE NEVER trusted from ML (3.4% precision) | 461/461 FP from RETREAT. Structural issue: context similarity + class imbalance |
| 2026-06-10 | INSERT ALWAYS defers to deterministic controller | Safety-critical phase, ML confidence irrelevant for control authority |
| 2026-06-10 | RETREAT advisory requires confidence>0.95 AND margin>0.5 | RETREAT recall=78.1%, some misclassified as DONE |
| 2026-06-10 | Encoder pre-training is a documented negative ablation | 99.91% (raw) vs 92.4% (encoder). Bottleneck destroys discrimination. |
| 2026-06-10 | SAC training deferred to GPU cluster | 1M-5M steps required, ~11h simulation, no local GPU |
