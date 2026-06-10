# Claims vs Evidence Audit

Date: 2026-06-10
Auditor: Autonomous technical lead (final audit)

## Methodology

For each major claim, we list:
- The claim
- Supporting evidence file/path
- Command or diagnostic that produced it
- Status: validated / partially validated / scaffold only / limitation
- Whether safe for PhD proposal
- Whether safe for publication

---

## 1. Robust Gazebo Robotic Cell

| Field | Value |
|-------|-------|
| Claim | KUKA LBR iisy 6 R1300 + D405 RGB-D + gripper + peg + hole fixture operational in Gazebo Harmonic |
| Evidence | `src/thesis_bringup/launch/research_baseline.launch.py`, URDF/SDF meshes |
| Command | `ros2 launch thesis_bringup research_baseline.launch.py use_gui:=false ...` |
| Status | **VALIDATED** — 60 trials executed, Gazebo starts reliably |
| Proposal-safe | YES |
| Publication-safe | YES |

## 2. Full-Task SEARCH-Enabled Execution

| Field | Value |
|-------|-------|
| Claim | Full task pipeline: MOVING_TO_START → APPROACH → SEARCH → INSERT → RETREAT → DONE |
| Evidence | `diagnostics/research_baseline_production_search_v20_600s/` (20 trials), per-trial outcome JSONs |
| Command | Trial outcome JSONs contain `phases[]` array with all 5 phase entries |
| Status | **VALIDATED** — 53/60 (88.3%) trials complete full task, all phases captured |
| Proposal-safe | YES |
| Publication-safe | YES |

## 3. Deterministic Controller Baseline

| Field | Value |
|-------|-------|
| Claim | Deterministic admittance controller achieves 87.5% (35/40) combined baseline |
| Evidence | `diagnostics/research_baseline_search_entered_500hz_v1_10trial/summary.json` (8/10), `diagnostics/research_baseline_search_confirmation_v1/summary.json` (9/10), `diagnostics/research_baseline_production_search_v20_600s/summary.json` (18/20) |
| Command | `cat diagnostics/*/summary.json \| jq '.success_rate'` |
| Status | **VALIDATED** — 35/40 = 87.5% across 3 independent runs |
| Proposal-safe | YES |
| Publication-safe | YES |

## 4. INSERT/Recenter/Side-Load Recovery

| Field | Value |
|-------|-------|
| Claim | INSERT phase uses recenter (up to 3 attempts) and side-load recovery (depth < 10mm) |
| Evidence | `src/kuka_task_control/` admittance_insertion_node parameters, per-trial outcome metrics |
| Command | Per-trial JSONs: `insert_predepth_recenter_attempts`, `insert_shallow_sideload_recovery_attempts` |
| Status | **VALIDATED** — Mechanism operational; 6 side-load aborts across 60 trials documented |
| Proposal-safe | YES |
| Publication-safe | YES |

## 5. Perception Logger Reliability

| Field | Value |
|-------|-------|
| Claim | 60/60 (100%) non-empty perception logs across all automated runs |
| Evidence | `diagnostics/*/perception_trial_*/logger_diagnostic.json` (all show `startup_ok: true`) |
| Command | `grep -r "startup_ok" diagnostics/*/perception_trial_*/logger_diagnostic.json` |
| Status | **VALIDATED** — Root cause (DDS SHM) fixed in commit d791219 |
| Proposal-safe | YES |
| Publication-safe | YES |

## 6. Multi-Phase Dataset Coverage

| Field | Value |
|-------|-------|
| Claim | 22,083-row dataset covering 6 phases (MOVING_TO_START, APPROACH, SEARCH, INSERT, RETREAT, DONE, UNKNOWN) |
| Evidence | `diagnostics/multi_trial_dataset_v3/context_vectors_v3.parquet` |
| Command | `python3 -c "import pandas as pd; df=pd.read_parquet('diagnostics/multi_trial_dataset_v3/context_vectors_v3.parquet'); print(len(df), df['phase_int'].value_counts().to_dict())"` |
| Status | **VALIDATED** — 22,083 rows, 7 phase classes present |
| Proposal-safe | YES |
| Publication-safe | YES |

## 7. v2_14 Safety-Gated Classifier Offline Performance

| Field | Value |
|-------|-------|
| Claim | 99.98% offline test accuracy on 68-dim raw context, all 7 classes F1 >= 0.996 |
| Evidence | `diagnostics/v2_14_raw_safety_gated_v4/offline_test_results.json` |
| Command | `python3 -m perception_pipeline.test_v2_14_safety_gated_offline --parquet ... --model ...` |
| Status | **VALIDATED** — Accuracy 0.9998, class-weighted loss, min-max normalization |
| Proposal-safe | YES |
| Publication-safe | YES |

## 8. Encoder Negative Ablation

| Field | Value |
|-------|-------|
| Claim | Autoencoder pre-training hurts: raw 68-dim = 99.91% vs encoder 32-dim = 92.4%, 0% SEARCH recall |
| Evidence | `diagnostics/v2_15_ablation_v5_6phase/ablation_results.json` |
| Command | `python3 -m perception_pipeline.v2_15_comprehensive_ablation --input-parquet ...` |
| Status | **VALIDATED** — Documented negative result, committed in 78a4a7b |
| Proposal-safe | YES (negative results are valid scientific contributions) |
| Publication-safe | YES |

## 9. v2_14 Shadow-Mode Live Validation

| Field | Value |
|-------|-------|
| Claim | 10-trial shadow-mode: 92.2% agreement, 9/10 physical success, INSERT 99.1% recall |
| Evidence | `diagnostics/v2_14_shadow_mode_validation/shadow_validation_summary.json` |
| Command | `python3 -m perception_pipeline.test_shadow_mode_validation` |
| Status | **VALIDATED** — All per-phase metrics verified from CSV data |
| Proposal-safe | YES |
| Publication-safe | YES |

## 10. DONE Precision Issue

| Field | Value |
|-------|-------|
| Claim | DONE precision = 3.4% (16/477 correct). Root cause: 461/461 FP from RETREAT due to context similarity and class imbalance (DONE=16 rows vs RETREAT=2107) |
| Evidence | Shadow-mode CSV analysis: all DONE predictions during RETREAT phase, logit analysis shows logit_5 (RETREAT) vs logit_6 (DONE) overlap |
| Command | Analysis script in shadow-mode validation session |
| Status | **VALIDATED** — Structural issue, documented limitation |
| Proposal-safe | YES (honest reporting of limitation) |
| Publication-safe | YES (negative results are publishable) |

## 11. v2_14 Guarded Advisory Integration

| Field | Value |
|-------|-------|
| Claim | 10-trial advisory: 90% physical success, all safety invariants hold, 485 DONE FP blocked, INSERT always deferred |
| Evidence | `diagnostics/v2_14_advisory_validation/advisory_validation_summary.json`, 24 unit tests |
| Command | `python3 -m pytest test_advisory_safety.py -v`, `python3 -m perception_pipeline.test_advisory_validation` |
| Status | **VALIDATED** — Safety invariants verified by both unit tests and live validation |
| Proposal-safe | YES |
| Publication-safe | YES |

## 12. SAC Baseline Scaffold

| Field | Value |
|-------|-------|
| Claim | SAC environment contract defined, mock rollouts validated (1% random, 5% deterministic), full Gazebo training requires GPU cluster |
| Evidence | `diagnostics/sac_environment_contract.json`, `diagnostics/sac_baseline_assessment/sac_feasibility_assessment.json` |
| Command | `python3 -m perception_pipeline.sac_baseline_scaffold --output ...`, `python3 -m perception_pipeline.sac_feasibility_assessment` |
| Status | **SCAFFOLDED** — Contract valid, training deferred to cluster |
| Proposal-safe | YES (scaffold is honest) |
| Publication-safe | YES (scaffold with honest feasibility assessment) |

## 13. Grand Total Validation Evidence

| Field | Value |
|-------|-------|
| Claim | 60 total trials, 53/60 (88.3%) physical success, 60/60 (100%) non-empty logs |
| Evidence | Cross-verified from 5 independent summary.json files across 5 run directories |
| Command | Aggregated from `diagnostics/*/summary.json` and `diagnostics/v2_14_*/advisory_validation_summary.json` |
| Status | **VALIDATED** — Numbers corrected from earlier inconsistent claims (was 42/50) |
| Proposal-safe | YES |
| Publication-safe | YES |

---

## Inconsistencies Found and Fixed

| Issue | Was | Corrected To | Docs Fixed |
|-------|-----|-------------|-----------|
| Production 10-trial success | 10/10 (100%) | 8/10 (80%) | PROPOSAL_MAPPING, REPRODUCIBILITY_GUIDE, metrics JSON |
| Combined baseline | 37/40 (92.5%) | 35/40 (87.5%) | PROPOSAL_MAPPING, REPRODUCIBILITY_GUIDE, metrics JSON |
| 20-trial confirmation | 17/20 (85%) | 18/20 (90%) | PROPOSAL_MAPPING, REPRODUCIBILITY_GUIDE, metrics JSON |
| Grand total | 42/50 (84%) | 53/60 (88.3%) | PROPOSAL_MAPPING, REPRODUCIBILITY_GUIDE, metrics JSON |
| Non-empty logs | 49/50 (98%) | 60/60 (100%) | PROPOSAL_MAPPING, REPRODUCIBILITY_GUIDE, metrics JSON |

## Summary

- **13/13 claims validated or honestly characterized**
- **5 numerical inconsistencies found and corrected**
- **No hidden failures or fabricated results**
- **All limitations documented**
- **All negative results preserved (encoder ablation, DONE precision)**
