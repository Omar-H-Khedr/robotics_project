# Scope Gap Audit — PhD Proposal vs Implementation

Date: 2026-06-10
Auditor: Autonomous technical lead

## Methodology

Each of the 11 proposal deliverables is audited against actual codebase evidence.
Status: **implemented** / **not implemented** / **scaffold only**

---

## 1. Different Peg Geometries

| Field | Value |
|-------|-------|
| Status | **NOT IMPLEMENTED** |
| Evidence | `peg_in_hole_description/models/cylindrical_peg/model.sdf` — only one geometry: cylindrical, 25mm diameter |
| Current | Single cylindrical peg (r=0.0125m, L=0.11m). No square, triangular, tapered, or hexagonal variants exist anywhere in codebase |
| What remains | Create 2+ additional peg SDF/URDF variants (e.g., square 25mm, tapered 22-28mm), run scenario matrix per variant |

## 2. Different Hole Geometries

| Field | Value |
|-------|-------|
| Status | **NOT IMPLEMENTED** |
| Evidence | `peg_in_hole_description/models/target_plate/model.sdf` — single circular hole (r=0.0135m, 27mm diameter) |
| Current | One circular hole only. No square, hexagonal, or slot-shaped holes defined |
| What remains | Create 2+ hole variants (e.g., square 27mm, slot 27x30mm), run scenario matrix per variant |

## 3. Different Clearance/Tolerance Levels

| Field | Value |
|-------|-------|
| Status | **NOT IMPLEMENTED** (config exists but never executed) |
| Evidence | `thesis_bringup/config/proposal_simulation_cell_v1_10.yaml:19-26` defines `clearance_mm: [0.1, 0.2, 0.5]` but all `execution_policy` flags are `false` |
| Current | Only 1mm radial clearance (25mm peg / 27mm hole) was actually tested. The v1_10 config is a dry-run-only matrix generator |
| What remains | Parameterize peg/hole SDF dimensions to sweep clearance levels. Run 10+ trials per clearance level |

## 4. Product/Tolerance Variation Experiments

| Field | Value |
|-------|-------|
| Status | **NOT IMPLEMENTED** |
| Evidence | No scripts exist that vary peg diameter, hole diameter, or tolerance across trials |
| Current | Fixed geometry for all 60 trials. No sensitivity analysis |
| What remains | Implement parameterized world generation. Sweep peg radius, hole radius, clearance. Add friction variation |

## 5. Systematic Generalization Across Geometry/Tolerance Scenarios

| Field | Value |
|-------|-------|
| Status | **NOT IMPLEMENTED** |
| Evidence | All 60 trials used identical 25mm/27mm/1mm-clearance geometry. No cross-scenario evaluation exists |
| Current | Zero generalization evidence. System validated on one geometry only |
| What remains | Execute full scenario matrix (see MILESTONE_GEOMETRY_TOLERANCE_MATRIX below), compute cross-scenario metrics |

## 6. Trained SAC Policy

| Field | Value |
|-------|-------|
| Status | **SCAFFOLD ONLY** |
| Evidence | `perception_pipeline/sac_baseline_scaffold.py` — environment contract, reward, termination validated via mock rollouts. `diagnostics/sac_environment_contract.json` |
| Current | No trained model weights (.pt/.pth) exist. No training logs. Mock rollouts: 1% random, 5% deterministic success |
| What remains | GPU cluster training (1M-5M steps), evaluation on scenario matrix, comparison to baseline |

## 7. Trained Meta-RL Policy

| Field | Value |
|-------|-------|
| Status | **NOT IMPLEMENTED** |
| Evidence | Zero code matching PEARL, MAML, RL2, or any meta-RL algorithm. `src/learning_interface/` is a placeholder (one `__init__.py`) |
| Current | No meta-RL implementation exists |
| What remains | Full implementation: meta-training loop, context encoder, adaptation mechanism, training on scenario distribution |

## 8. Context-Based Meta-RL (PEARL/MAML/RL2 or Equivalent)

| Field | Value |
|-------|-------|
| Status | **NOT IMPLEMENTED** |
| Evidence | Same as #7. No context-conditioned RL code exists |
| Current | The 68-dim context vector and phase classifier are the closest analog, but they are classification (not RL) |
| What remains | Implement context-conditioned policy that adapts behavior based on 68-dim observation |

## 9. Comparison: Deterministic Baseline vs v2_14 Advisory vs SAC vs Meta-RL

| Field | Value |
|-------|-------|
| Status | **PARTIAL** (2 of 4 compared) |
| Evidence | Deterministic baseline: 35/40 (87.5%). v2_14 advisory: 9/10 (90%). SAC: scaffold only. Meta-RL: not implemented |
| Current | Only deterministic vs advisory comparison exists. No SAC or meta-RL to compare |
| What remains | Train SAC + meta-RL, then run head-to-head comparison on same scenario matrix |

## 10. Hardware KUKA Validation

| Field | Value |
|-------|-------|
| Status | **NOT IMPLEMENTED** |
| Evidence | All 60 trials are Gazebo simulation. `proposal_simulation_cell_v2_16.yaml:98`: `real_robot_allowed: false`. No hardware config, IP addresses, or real-robot data |
| Current | Simulation-only. 88.3% success rate may not transfer to hardware |
| What remains | Deploy to physical KUKA LBR iisy 6 R1300, calibrate sim-to-real, validate with 10+ physical trials |

## 11. Sim-to-Real Transfer Evidence

| Field | Value |
|-------|-------|
| Status | **NOT IMPLEMENTED** |
| Evidence | No real-robot data, no domain adaptation code, no sim-to-real metrics |
| Current | Known open challenge (documented limitation) |
| What remains | Domain randomization, system identification, real-world fine-tuning, sim-to-real gap measurement |

---

## Summary

| # | Item | Status |
|---|------|--------|
| 1 | Different peg geometries | NOT IMPLEMENTED |
| 2 | Different hole geometries | NOT IMPLEMENTED |
| 3 | Different clearance/tolerance levels | NOT IMPLEMENTED (config only) |
| 4 | Product/tolerance variation | NOT IMPLEMENTED |
| 5 | Systematic generalization | NOT IMPLEMENTED |
| 6 | Trained SAC policy | SCAFFOLD ONLY |
| 7 | Trained meta-RL policy | NOT IMPLEMENTED |
| 8 | Context-based meta-RL | NOT IMPLEMENTED |
| 9 | Full comparison (det vs adv vs SAC vs meta-RL) | PARTIAL (2/4) |
| 10 | Hardware KUKA validation | NOT IMPLEMENTED |
| 11 | Sim-to-real transfer | NOT IMPLEMENTED |

**Honest assessment**: The project delivers a validated deterministic baseline (87.5%), a validated perception pipeline (99.98% offline), and a validated advisory integration (90%). However, 7 of 11 proposal items are not implemented or scaffold-only. The geometry/tolerance generalization gap (items 1-5) is the most critical for a doctoral thesis.
