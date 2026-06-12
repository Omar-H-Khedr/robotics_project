# Cross-Scenario v2_14 Evaluation Results

Date: 2026-06-12
Status: COMPLETED (honest negative result)

## Executive Summary

Cross-scenario v2_14 evaluation with row-level context data reveals that the
v2_14 classifier does NOT generalize well across geometry/tolerance scenarios
with limited training data. The geometry-only feasibility classifier, however,
performs excellently. This is an honest negative result that informs the
project's direction.

## Dataset

- 17,651 context vectors from 7 scenarios
- 68-dim raw context (RGB/depth empty in Gazebo)
- 14 trials across all scenarios
- Mixed success/failure outcomes

## Mode A: Mixed-Scenario Random Split (80/20)

| Metric | Value |
|--------|-------|
| Accuracy | 38.9% |
| Macro F1 | 8.3% |
| Weighted F1 | 21.9% |
| SEARCH recall | 0.0% |
| RETREAT recall | 0.0% |
| DONE recall | 1.0% |

### Per-Scenario Accuracy (Mixed Split)

| Scenario | Accuracy | Rows |
|----------|----------|------|
| baseline_loose | 24.6% | 1,193 |
| clearance_medium | 35.3% | 1,220 |
| clearance_tight | 71.7% | 198 |
| large_peg_large_hole | 26.9% | 424 |
| misaligned_baseline | 98.5% | 133 |
| small_peg_small_hole | 82.8% | 145 |
| tight_plus_misaligned | 65.6% | 218 |

## Mode B: Held-Out Scenario Evaluation (Leave-One-Out)

| Metric | Value |
|--------|-------|
| Mean accuracy | 20.9% +/- 27.2% |
| Min accuracy | 0.0% (misaligned_baseline) |
| Max accuracy | 75.1% (clearance_tight) |
| Mean macro F1 | 5.6% |

### Per-Held-Out-Scenario Results

| Held Out | Train Scenarios | Accuracy | Macro F1 |
|----------|----------------|----------|----------|
| baseline_loose | 6 | 3.5% | 1.2% |
| clearance_medium | 6 | 46.3% | 9.2% |
| clearance_tight | 6 | 75.1% | 10.5% |
| large_peg_large_hole | 6 | 21.3% | 5.2% |
| misaligned_baseline | 6 | 0.0% | 0.0% |
| small_peg_small_hole | 6 | 0.3% | 0.2% |
| tight_plus_misaligned | 6 | 0.0% | 0.0% |

## Mode C: Geometry-Only Feasibility Classifier

| Metric | Value |
|--------|-------|
| Accuracy | 96.4% |
| False-safe rate | 0.0% (0 cases) |
| Fail-closed detection | 100.0% |
| False-block rate | 6.6% |
| ML classifier accuracy | 100.0% |

## Mode D: Safety-Gated Advisory Mode

| Metric | Value |
|--------|-------|
| Raw ML accuracy | 13.5% |
| Advisory accuracy | 30.8% |
| Fallback rate | 100.0% |
| INSERT deferred | 0 |
| DONE blocked | 4 |
| Feasibility override | 416 |
| Low confidence fallback | 3,111 |
| Unsafe advice blocked | 4 |
| SEARCH recall | 0.0% |
| RETREAT recall | 0.0% |
| DONE recall | 0.0% |

## Key Findings

### 1. v2_14 Cross-Scenario Generalization Is POOR

The v2_14 classifier achieves only 38.9% accuracy on mixed-scenario splits
and 20.9% mean accuracy on held-out scenarios. This is significantly worse
than the 99.98% offline accuracy on baseline-only data.

**Root causes:**
- Empty RGB/depth features (Gazebo D405 limitation)
- Only 3 trials per scenario (insufficient training data)
- Domain shift between scenarios (different clearance, offset, geometry)
- Phase distribution imbalance across scenarios

### 2. Geometry-Only Feasibility Classifier Is EXCELLENT

The geometry-only feasibility classifier achieves 96.4% accuracy with 0%
false-safe rate. This proves that geometry parameters (clearance, offset)
are sufficient for envelope-aware advisory, even without neural network features.

### 3. Safety-Gated Advisory Falls Back to Deterministic

The safety-gated advisory mode has a 100% fallback rate, meaning ALL
predictions are deferred to the deterministic controller. This is because:
- Low confidence threshold triggers fallback for most predictions
- Feasibility classifier overrides out-of-envelope predictions
- DONE is blocked from ML (structural limitation)

### 4. Cross-Scenario Evaluation Is Now POSSIBLE

With the multi-scenario row-level dataset, true cross-scenario evaluation
is now possible. The poor results are a legitimate finding, not a limitation
of the evaluation framework.

## Implications for Thesis

1. **v2_14 is limited to baseline/envelope advisory** — cross-scenario generalization
   requires significantly more training data and/or valid RGB features

2. **Geometry-only feasibility is the practical advisory approach** — it achieves
   96.4% accuracy with 0% false-safe rate using only geometry parameters

3. **SAC/meta-RL may improve cross-scenario generalization** — scenario randomization
   during training could learn transferable policies

4. **Hardware validation will face similar challenges** — real-world data will be
   needed for practical cross-scenario generalization

## Honest Assessment

| Claim | Status |
|-------|--------|
| v2_14 generalizes across scenarios | FALSE — 38.9% mixed, 20.9% held-out |
| v2_14 is a practical advisory layer | PARTIAL — only for baseline/envelope |
| Geometry-only feasibility works | TRUE — 96.4% accuracy, 0% false-safe |
| Safety-gated advisory is conservative | TRUE — 100% fallback rate |
| Cross-scenario evaluation is complete | TRUE — with honest negative results |
