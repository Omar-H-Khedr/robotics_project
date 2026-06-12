# Current Project Status

Date: 2026-06-12

## What Is Implemented and Validated

| Component | Status | Evidence |
|-----------|--------|----------|
| Gazebo robotic cell (KUKA LBR iisy 6 R1300) | VALIDATED | 200+ trials, Gazebo starts reliably |
| Deterministic admittance controller | VALIDATED | 72/140 (51%) full matrix, 90% at loose clearance |
| SEARCH spiral convergence | VALIDATED | 100% convergence rate (clearance-aware threshold) |
| INSERT recenter + side-load recovery | VALIDATED | Recovery mechanisms exercised |
| Perception logger (DDS SHM fix) | VALIDATED | 140/140 non-empty logs |
| v2_14 safety-gated classifier | VALIDATED | 99.98% offline accuracy |
| v2_14 shadow-mode validation | VALIDATED | 92.2% agreement, 9/10 success |
| v2_14 guarded advisory integration | VALIDATED | 24 unit tests, all safety invariants hold |
| Encoder negative ablation | VALIDATED | 99.91% raw vs 92.4% encoder |
| SAC environment contract | SCAFFOLD ONLY | Mock rollouts validated, no training |
| Research baseline launch system | VALIDATED | Parameterized, reproducible |
| Geometry/tolerance scenario matrix | VALIDATED | 140 trials across 7 scenarios (Stage C) |
| Clearance-aware convergence | VALIDATED | max(clearance, 1mm) threshold works |
| Cross-scenario dataset | COLLECTED | 17,651 context vectors from 7 scenarios |
| Cross-scenario v2_14 evaluation | COMPLETED | 38.9% mixed, 20.9% held-out (negative result) |
| Geometry-only feasibility classifier | EXCELLENT | 96.4% accuracy, 0% false-safe rate |
| SAC scenario-randomization scaffold | CLUSTER-READY | Not trained (requires GPU cluster) |
| Hardware validation protocol | CREATED | FUTURE WORK |
| Sim-to-real transfer plan | CREATED | FUTURE WORK |

## What Is NOT Implemented (Scope Gaps)

| Item | Status | Severity | What Remains |
|------|--------|----------|-------------|
| Sub-mm clearance insertion | HARD LIMIT | CRITICAL | Tracking noise floor (~0.5mm) prevents insertion at ≤0.5mm clearance |
| Different peg geometries | NOT IMPLEMENTED | MEDIUM | Create 2+ SDF variants, run matrix |
| Different hole geometries | NOT IMPLEMENTED | MEDIUM | Create 2+ hole variants, run matrix |
| Product/tolerance variation | NOT IMPLEMENTED | MEDIUM | Add friction/size variation |
| Trained SAC policy | SCAFFOLD ONLY | HIGH | GPU cluster training (scenario-randomization scaffold implemented) |
| Trained meta-RL policy | NOT IMPLEMENTED | HIGH | Full implementation + training |
| Context-based meta-RL | NOT IMPLEMENTED | HIGH | PEARL/MAML/RL2 or equivalent |
| Full comparison (4 methods) | PARTIAL (2/4) | MEDIUM | Need SAC + meta-RL |
| Hardware KUKA validation | NOT IMPLEMENTED | HIGH | Physical robot deployment |
| Sim-to-real transfer | NOT IMPLEMENTED | HIGH | Domain randomization + real trials |

## Key Numbers

- **140** geometry/tolerance trials (Stage C, full matrix)
- **72/140 (51%)** overall success across 7 scenarios
- **72/80 (90%)** success at loose clearance (1.0mm)
- **0/60 (0%)** success at tight/medium clearance (≤0.5mm)
- **100%** SEARCH convergence rate (clearance-aware threshold)
- **0** contact-guided insertions (all successes via SEARCH convergence)
- **60** baseline trials
- **53/60 (88.3%)** baseline success rate
- **24** unit tests passing
- **3** peg sizes validated (22/25/28mm at 1.0mm clearance)
- **3** clearance levels tested (0.25/0.5/1.0mm)
- **7** scenarios validated
- **84.3%** feasibility classifier accuracy (default profile)
- **0.0%** false-safe rate (conservative profile, but blocks all feasible insertions)
- **96.4%** geometry-only feasibility classifier accuracy (row-level evaluation)
- **0.0%** false-safe rate (geometry-only, row-level evaluation)
- **38.9%** v2_14 cross-scenario accuracy (mixed split, honest negative result)
- **20.9%** v2_14 cross-scenario held-out mean accuracy (honest negative result)
- **Advisory-only** classifier role enforced
- **100%** safety-gated advisory fallback rate

## Geometry/Tolerance Stage C Results (Final)

| Scenario | Peg | Hole | Clearance | Offset | N | Success | Rate |
|----------|-----|------|-----------|--------|---|---------|------|
| baseline_loose | 25mm | 27mm | 1.0mm | 0mm | 20 | 16 | 80% |
| clearance_medium | 25mm | 26mm | 0.5mm | 0mm | 20 | 0 | 0% |
| clearance_tight | 25mm | 25.5mm | 0.25mm | 0mm | 20 | 0 | 0% |
| large_peg_large_hole | 28mm | 30mm | 1.0mm | 0mm | 20 | 19 | 95% |
| small_peg_small_hole | 22mm | 24mm | 1.0mm | 0mm | 20 | 20 | 100% |
| misaligned_baseline | 25mm | 27mm | 1.0mm | 1mm | 20 | 17 | 85% |
| tight_plus_misaligned | 25mm | 25.5mm | 0.25mm | 1mm | 20 | 0 | 0% |

**Key finding**: Clearance > 2× tracking noise (~0.5mm) is required for reliable insertion. Sub-millimeter clearance is a hard physical limit of the current sensor stack.

## Next Milestone

**Cross-scenario generalization** — train on some scenarios, test on held-out
- See `docs/CROSS_SCENARIO_GENERALIZATION_PLAN.md`
- Validates that learning generalizes across geometry variations
- Requires multi-scenario dataset from Stage C (now complete)
- Row-level data collection planned for v2_14 cross-scenario evaluation
- SAC training with scenario randomization requires GPU cluster
