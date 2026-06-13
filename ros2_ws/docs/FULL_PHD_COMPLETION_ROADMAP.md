# Full PhD Completion Roadmap and Simulation Framework Decision

Date: 2026-06-13
Branch: `robot-review`
Planning basis: commit `39f34d6` plus the local evidence and planning documents listed below.

## Evidence Base

This roadmap synthesizes:

- `docs/LOCAL_PROJECT_RESEARCH_AUDIT.md`
- `docs/SIMULATION_FRAMEWORK_REDESIGN_PLAN.md`
- `docs/LITERATURE_AWARE_UPGRADE_PLAN.md`
- `docs/NEXT_SIMULATION_SPRINT_PLAN.md`
- `docs/GITHUB_ISSUES_BACKLOG.md`
- `docs/CURRENT_PROJECT_STATUS.md`
- `docs/PROPOSAL_IMPLEMENTATION_MAPPING.md`
- `docs/CLAIMS_VS_EVIDENCE_AUDIT.md`
- `docs/FINAL_LIMITATIONS_AND_NEXT_WORK.md`
- `docs/OPERATING_ENVELOPE_ANALYSIS.md`
- `docs/CROSS_SCENARIO_ROW_LEVEL_DATASET.md`
- `docs/CROSS_SCENARIO_V2_14_EVALUATION.md`
- `docs/metrics/comprehensive_validation_metrics.json`
- `docs/context/proposal_context.md`

This document is strategic planning only. It does not authorize immediate SAC/meta-RL training, hardware execution, Gazebo migration, controller edits, or new simulation batches.

## Executive Summary

The project has reached a strong simulation-only checkpoint. The current defensible PhD contribution is a ROS 2/Gazebo safety-gated peg-in-hole assembly framework with a measured geometry/tolerance operating envelope, conservative fail-closed behavior, and honest negative evidence for cross-scenario neural generalization.

The best final simulation framework for the whole PhD is a staged hybrid architecture:

1. **ROS 2/Gazebo remains the authoritative validation harness.**
   It is the source of current evidence, controller validation, launch reproducibility, KUKA task-level integration, logging, and thesis system claims.

2. **MuJoCo should be added for fast contact-rich SAC/meta-RL training after the simulator-neutral contract and observation/contact validity gates are complete.**
   It should not replace Gazebo. It should produce candidate policies or residuals that must be re-evaluated in Gazebo before becoming thesis claims.

3. **Isaac Sim should be added only for the visual/depth and synthetic-data track if GPU resources and setup time are available.**
   It is the right tool for D405-like RGB/depth, segmentation, camera-domain randomization, and perception-data generation. It should not be the first full-stack migration target.

4. **The final authority remains deterministic safety.**
   ML may provide advisory decisions, feasibility estimates, residual corrections, or bounded policy proposals. It must not override the deterministic safety gate, authorize out-of-envelope insertion, or take unsupervised authority during safety-critical INSERT.

The main stop condition for the next technical phase is clear: do not train SAC or meta-RL until the observation validity, contact-label reconciliation, and simulator-neutral scenario/result contract are in place.

## Current Project Status

### Defensible Evidence

| Area | Status | Evidence |
|---|---:|---|
| ROS 2/Gazebo KUKA cell | Validated | 60 baseline/advisory trials and 140 Stage C trials |
| Deterministic full task controller | Validated | 53/60 grand-total success, all failures safe |
| Geometry/tolerance operating envelope | Validated | 72/80 success at 1.0 mm clearance, 0/60 at <=0.5 mm |
| SEARCH convergence | Validated | 100% convergence in Stage C |
| Safety-gated v2_14 advisory | Validated inside baseline distribution | 24 unit tests, all safety invariants hold |
| Encoder ablation | Validated negative result | Raw 68-dim beats encoder; encoder gives 0% SEARCH recall |
| Cross-scenario v2_14 evaluation | Validated negative result | 38.9% mixed split, 20.9% held-out mean |
| Geometry-only feasibility classifier | Strong | 96.4% accuracy, 0.0% false-safe rate |
| SAC/meta-RL | Scaffold only | Contracts and cluster scripts exist, no trained policy |
| Hardware/sim-to-real | Protocol only | No hardware validation executed |

### Key Numbers

- Stage C geometry/tolerance matrix: 140 trials across 7 scenarios.
- Robust envelope: 72/80 successes, 90%, at 1.0 mm radial clearance.
- Marginal/tight envelope: 0/60 successes at <=0.5 mm clearance.
- Tracking noise floor: approximately 0.5 mm.
- Practical rule: reliable insertion currently requires clearance greater than about 2x the tracking noise floor.
- Cross-scenario row-level dataset: 17,651 rows, 7 scenarios, 14 trials.
- Row-level visual limitation: RGB/depth features are empty or non-informative in current Gazebo data.
- Row-level force limitation: F/T bridge is not usable for ML features; task-side control receives separate force information.
- v2_14 cross-scenario result: poor neural generalization, useful as a negative result.
- Geometry-only feasibility result: currently the most reliable envelope-aware advisory signal.

### Core Scientific Interpretation

The project should not be presented as solved universal peg-in-hole adaptation. It should be presented as:

- a reproducible safety-gated simulation framework,
- a measured clearance/noise operating-envelope study,
- an honest demonstration that per-tick baseline-trained neural advisory does not generalize across geometry/tolerance scenarios under weak visual/contact observations,
- a strong basis for later SAC/meta-RL once simulator/data validity is fixed.

## Final Recommended Simulation Framework

### Decision

Use **ROS 2/Gazebo + MuJoCo + Isaac Sim as a staged hybrid architecture**, with strict role separation:

- Gazebo is the authoritative ROS 2 system-validation simulator.
- MuJoCo is the fast RL/meta-RL training simulator for compact contact-rich state/control experiments.
- Isaac Sim is the synthetic perception and camera-domain randomization simulator.
- A shared scenario/data/action/evaluation contract prevents result drift across simulators.

This is the best final framework because the proposal needs three different capabilities that no single simulator currently provides well enough for this project:

1. ROS 2/KUKA integration and reproducible controller evidence.
2. Fast large-scale learning for SAC/meta-RL.
3. Credible RGB/depth synthetic data for D405-like perception and domain randomization.

Gazebo already solves the first. MuJoCo is the best first addition for the second. Isaac Sim is the best later addition for the third.

### Resource-Constrained Variant

If GPU/Isaac setup is not feasible, the minimum credible final framework becomes:

- ROS 2/Gazebo for validation.
- MuJoCo for fast RL/meta-RL.
- No strong visual-learning claims until real or Isaac-quality RGB/depth data exists.

That variant can still support a strong PhD centered on safety-gated contact-rich assembly, operating-envelope modeling, and learning under structured geometry/contact randomization. It cannot honestly claim full RGB-D visual generalization.

## Simulator Comparison Table

| Option | Best use | Strengths | Weaknesses | Final decision |
|---|---|---|---|---|
| ROS 2/Gazebo only | System validation, KUKA launch, controller regression | Already implemented, evidence continuity, ROS 2-native, reproducible trial artifacts | Too slow for large RL, current RGB/depth weak, contact-topic evidence incomplete | Keep as validation authority, not the only final simulator |
| ROS 2/Gazebo + MuJoCo | Validation plus fast contact-rich RL/meta-RL | Preserves current evidence while enabling fast training and randomized rollouts | Does not solve high-fidelity RGB/depth by itself; requires model conversion and adapter | Best learning-focused hybrid if perception is not central |
| ROS 2/Gazebo + Isaac Sim | Validation plus synthetic perception | Strong RGB/depth, segmentation, camera/material randomization, proposal alignment | Heavier setup, GPU/version sensitivity, not the best first path for millions of RL steps | Good if visual dataset quality is the next major contribution |
| ROS 2/Gazebo + MuJoCo + Isaac Sim | Full staged PhD stack | Covers integration, fast learning, and synthetic perception with clear authority boundaries | Highest integration complexity; requires strict contracts | Recommended final architecture |
| Drake | Contact modeling studies and hydroelastic analysis | Strong analytical contact tools and reproducibility | High integration cost; not ideal as main deep-RL or visual simulator | Reserve for optional contact-fidelity analysis, not main stack |
| PyBullet | Cheap prototype or CI smoke tests | Easy URDF import, low cost, fast enough for simple tests | Contact fidelity risk for tight peg-in-hole; weak publication basis for contact claims | Do not use as final simulator; optional toy baseline only |
| Full Isaac migration | One simulator for proposal alignment and visuals | Strong perception and synthetic data story | Breaks current evidence continuity; high migration risk; still needs ROS validation | Not recommended |
| Full Gazebo migration away from current stack | None | Could modernize internals | High cost with little scientific benefit | Not recommended |

## What Gazebo Should Be Used For

Gazebo should remain the project's **authoritative validation harness**:

- ROS 2 integration and launch reproducibility.
- KUKA task-level validation.
- `ros2_control` controller validation.
- deterministic task-controller regression.
- safety-gate validation and fail-closed testing.
- Stage C operating-envelope reproduction.
- short smoke tests for perception/contact logging.
- final replay/evaluation of any learned policy or residual before thesis claims.
- artifact generation: outcome JSON, row-level logs, scenario manifests, metrics summaries.

Gazebo should not be forced to become the high-throughput RL training engine. It is scientifically valuable because it validates the system stack, not because it is the fastest simulator.

## What MuJoCo Should Be Used For

MuJoCo should be added after the contract and data validity sprint for:

- fast contact-rich SAC training.
- PEARL-style or recurrent context/meta-RL training.
- dense domain randomization over clearance, offset, friction, damping, compliance, and noise.
- policy pretraining.
- reward, termination, and safety-ablation sweeps.
- tolerance variation sweeps around the 0.5-1.5 mm transition region.
- curriculum learning before Gazebo replay.

MuJoCo outputs should be treated as candidate policies, not final system evidence. A policy only becomes thesis evidence after it passes Gazebo evaluation under the same scenario/result contract.

## What Isaac Sim Should Be Used For

Isaac Sim should be added if the PhD keeps a serious RGB-D/visual-perception contribution:

- synthetic RGB/depth data.
- camera realism closer to D405-style perception.
- segmentation masks for peg, hole, fixture, and workspace.
- randomized lighting, materials, textures, camera pose, and occlusions.
- visual-domain randomization for perception robustness.
- visual feature pretraining or validation before Gazebo/hardware replay.

Isaac Sim should not be used as the first full-stack replacement for Gazebo. It should be a perception-data environment connected through the same scenario and dataset schema.

## What Should Not Be Used and Why

### Drake

Drake is not the best main simulator for this PhD stack. It is valuable for optional contact-model studies, especially if the thesis needs a deeper hydroelastic/contact-fidelity appendix or paper. It is not the best primary path because the project needs ROS 2 validation continuity, large-scale RL, and synthetic RGB/depth data. Drake would add integration cost without replacing Gazebo, MuJoCo, or Isaac Sim cleanly.

### PyBullet

PyBullet should not be used for final contact-rich peg-in-hole claims. It is useful for quick prototypes, CI smoke tests, or simple policy-interface experiments, but its tight-clearance contact fidelity is too risky for the core thesis evidence. Using it as the main simulator would weaken the contact-fidelity argument.

### Full Gazebo-Only Final Stack

A Gazebo-only final stack is conservative but incomplete. It preserves evidence continuity, but it does not solve high-throughput RL or the current RGB/depth limitations. It is acceptable only if the final PhD scope drops trained SAC/meta-RL and strong visual-learning claims.

### Full Isaac Sim Migration

A full Isaac migration is not recommended now. It would align superficially with the proposal but would risk losing the validated Gazebo evidence base, controller behavior, launch reproducibility, and existing diagnostics. Isaac should enter as a targeted perception-data track, not as a replacement for the current validation harness.

## Full Phased Roadmap

### Phase 1: Current Evidence Consolidation

| Field | Plan |
|---|---|
| Objective | Consolidate the current simulation-only evidence into a publication-ready, claims-safe baseline package. |
| Scientific importance | Establishes the validated operating envelope and prevents overclaiming before new learning work. |
| Proposal relation | Supports foundations, representation, safety-gated advisory, and benchmark preparation. |
| Recommended framework | ROS 2/Gazebo only; no new simulator yet. |
| Inputs | Current docs, Stage C outcomes, row-level dataset, v2_14/v2_15 results, metrics JSON. |
| Implementation tasks | Freeze evidence tables, align metrics across docs, add confidence intervals, formalize failure taxonomy, preserve negative results. |
| Expected outputs | Evidence index, paper-ready operating-envelope figures/tables, claims-vs-evidence matrix. |
| Validation metrics | 72/80 robust-envelope success, 0/60 tight/medium success, 53/60 baseline/advisory success, v2_14 negative metrics, geometry feasibility metrics. |
| Success criteria | Every thesis claim maps to a local artifact; no unsupported claims about SAC, meta-RL, hardware, contact-topic fidelity, or visual learning. |
| Risks | Numerical inconsistencies across docs; temptation to hide negative results. |
| Stop/go decision | Go to Phase 2 only when evidence and limitations are internally consistent. |
| Local feasibility | Yes. |
| Needs GPU | No. |
| Needs hardware | No. |

### Phase 2: Improved Simulation/Data Validity

| Field | Plan |
|---|---|
| Objective | Repair or explicitly bound the observation/contact evidence before learning. |
| Scientific importance | Learning claims are invalid if visual/depth features are empty and contact labels disagree. |
| Proposal relation | Supports RGB-D + force/proprioceptive observation, contact-state disambiguation, and safety-filter evidence. |
| Recommended framework | ROS 2/Gazebo validation harness. |
| Inputs | 17,651-row cross-scenario dataset, Gazebo D405 topics, task-side wrench/contact logs, scenario YAML, existing diagnostics. |
| Implementation tasks | Observation validity audit, feature-validity masks, contact-label reconciliation, scenario/result contract document, short smoke checks only. |
| Expected outputs | Dataset-quality report, contact-label agreement matrix, `SIMULATION_DATA_CONTRACT.md`, updated limitations. |
| Validation metrics | Nonzero RGB validity where claimed, finite depth ROI, valid feature masks, contact-topic/wrench/geometric/task-state agreement matrix, scenario metadata completeness. |
| Success criteria | Accepted datasets cannot silently treat empty RGB/depth or zero F/T as meaningful perception; contact labels are source-tagged. |
| Risks | Existing Gazebo data may remain visually invalid; contact topics may stay zero. |
| Stop/go decision | If observations/contact cannot be made credible in Gazebo, proceed with Gazebo for validation only and move visual data generation to Isaac Sim. |
| Local feasibility | Yes for audit and short smoke checks. |
| Needs GPU | No. |
| Needs hardware | No. |

### Phase 3: Final Simulation Framework Upgrade

| Field | Plan |
|---|---|
| Objective | Add the simulator-neutral architecture and pilot simulator adapters without replacing Gazebo. |
| Scientific importance | Enables fair comparison across deterministic, advisory, SAC, and meta-RL methods. |
| Proposal relation | Resolves the Gazebo-vs-Isaac mismatch through a scientifically justified hybrid instead of a disruptive migration. |
| Recommended framework | Gazebo authoritative harness; MuJoCo pilot first; Isaac Sim visual pilot if GPU is available. |
| Inputs | Scenario/result contract, Stage C scenarios, SAC scaffold, geometry/tolerance matrix, observation/contact validity results. |
| Implementation tasks | Define shared scenario schema, shared dataset schema, shared action interface, reward/termination parity, adapter test plan, seed/reproducibility rules. |
| Expected outputs | Simulator-neutral contract, MuJoCo pilot design, Isaac Sim visual-data design, adapter acceptance tests. |
| Validation metrics | Lossless representation of Stage C scenarios, deterministic reset in pilot env, reward parity checks, result-schema parity, seed reproducibility. |
| Success criteria | Existing Stage C scenarios can run or be represented through the contract; no simulator-specific claim bypasses Gazebo validation. |
| Risks | Interface drift, model conversion mismatch, too much architecture before evidence. |
| Stop/go decision | Build MuJoCo only if the contract maps current scenarios cleanly; build Isaac only if the visual task remains central and GPU resources are real. |
| Local feasibility | Yes for contract and MuJoCo smoke tests; Isaac may require separate workstation. |
| Needs GPU | No for contract/MuJoCo smoke; yes for Isaac at useful scale. |
| Needs hardware | No. |

### Phase 4: Cross-Scenario Perception/Representation Learning

| Field | Plan |
|---|---|
| Objective | Rebuild cross-scenario representation learning on valid observations and balanced phase coverage. |
| Scientific importance | Turns the current v2_14 negative result into a controlled representation-learning study. |
| Proposal relation | Addresses RGB-D/force/proprioceptive encoding and context-state disambiguation. |
| Recommended framework | Gazebo for validated row-level data; Isaac Sim for synthetic RGB/depth if Gazebo remains visually weak. |
| Inputs | Validity-audited datasets, feature masks, scenario metadata, contact labels, geometry feasibility labels. |
| Implementation tasks | Collect balanced row-level data after audits, add sequence-aware models, compare raw context vs geometry-only vs visual/depth vs temporal context, preserve safety gating. |
| Expected outputs | Representation ablation study, positive or negative visual-depth result, sequence-aware phase classifier result. |
| Validation metrics | Held-out scenario accuracy, macro F1, SEARCH/RETREAT/DONE recall, false-safe rate, fallback rate, visual feature validity coverage. |
| Success criteria | Any visual claim uses non-empty, valid RGB/depth; geometry-only remains the safety fallback; ML does not control INSERT. |
| Risks | Visual data remains unhelpful; phase imbalance persists; sequence model overfits. |
| Stop/go decision | If visual features remain invalid, publish the negative result and shift the learning claim toward geometry/contact/state context. |
| Local feasibility | Yes for small offline models and audits. |
| Needs GPU | Helpful for larger vision/sequence models; required for large Isaac data generation. |
| Needs hardware | No. |

### Phase 5: SAC/Meta-RL Simulation Training

| Field | Plan |
|---|---|
| Objective | Train real SAC and then context/meta-RL policies under scenario randomization. |
| Scientific importance | Provides the missing learned-control baseline and tests the proposal's adaptation hypothesis. |
| Proposal relation | Directly supports context-conditioned SAC/meta-RL and adaptation-speed hypotheses. |
| Recommended framework | MuJoCo for fast training; Gazebo for policy replay/validation; Isaac Sim only for visual-pretraining data if used. |
| Inputs | MuJoCo environment, scenario randomization, reward/termination schema, safety constraints, deterministic baseline metrics. |
| Implementation tasks | SAC multi-seed training, randomized tasks, checkpointing, policy evaluation, residual/advisory interface, then PEARL-style or recurrent context policy after SAC baseline exists. |
| Expected outputs | Trained SAC checkpoints, learning curves, held-out scenario results, meta-RL adaptation curves. |
| Validation metrics | Safe success, unsafe rate, fail-closed rate, sample efficiency, adaptation episodes, peak/cumulative wrench proxies, held-out scenario performance. |
| Success criteria | SAC beats or complements deterministic baseline on a defined target without increasing unsafe behavior; meta-RL adapts faster than SAC on held-out task families. |
| Risks | Learned policy underperforms deterministic controller; simulator mismatch; reward hacking; unsafe residuals. |
| Stop/go decision | Stop meta-RL if SAC does not establish a credible learned baseline; do not promote policies that fail Gazebo validation. |
| Local feasibility | MuJoCo smoke and small training may be local; meaningful training likely not. |
| Needs GPU | Yes for serious multi-seed SAC/meta-RL. |
| Needs hardware | No. |

### Phase 6: Comparison With Deterministic Baseline and v2_14 Advisory

| Field | Plan |
|---|---|
| Objective | Compare deterministic controller, geometry feasibility, v2_14 advisory, SAC, and meta-RL fairly. |
| Scientific importance | Converts separate components into a defensible thesis evaluation. |
| Proposal relation | Supports benchmark and ablation phases, including safety and context comparisons. |
| Recommended framework | Gazebo for final evaluation; MuJoCo for training/evaluation diagnostics. |
| Inputs | Deterministic baseline, v2_14 advisory, geometry feasibility model, trained SAC, trained meta-RL, shared scenario suite. |
| Implementation tasks | Define method table, run matched scenarios/seeds, report confidence intervals, compare safety interventions, evaluate residual-only policies under safety gate. |
| Expected outputs | Full benchmark tables, ablation results, method comparison figures. |
| Validation metrics | Safe success, nominal success, unsafe violations, fallback rate, insertion time, failure mode distribution, adaptation speed, confidence intervals. |
| Success criteria | Claims are limited to methods with real evidence; deterministic safety remains final authority; untrained methods remain labeled as scaffold/future work. |
| Risks | Methods are not comparable due to simulator differences; learned policies lack Gazebo transfer. |
| Stop/go decision | If MuJoCo-trained policy fails Gazebo replay, report simulator-gap result and do not claim learned policy superiority. |
| Local feasibility | Analysis yes; full experiments depend on training results. |
| Needs GPU | Only if additional training is required. |
| Needs hardware | No. |

### Phase 7: Publication-Ready Simulation Study

| Field | Plan |
|---|---|
| Objective | Package the strongest simulation evidence into publishable manuscripts and thesis chapters. |
| Scientific importance | Separates validated contributions from aspirational future work. |
| Proposal relation | Supports final benchmark, manuscript, and thesis milestones. |
| Recommended framework | Gazebo for system/evidence papers; MuJoCo/Isaac only where their data is actually used. |
| Inputs | Operating-envelope results, contact/observation audits, representation results, learned-policy results if available. |
| Implementation tasks | Create paper outlines, statistical analysis, reproducibility package, limitations section, claims table, artifact checklist. |
| Expected outputs | Manuscripts, thesis chapters, reproducibility archive, figures/tables. |
| Validation metrics | Confidence intervals, ablations, scenario coverage, artifact reproducibility, claim-evidence traceability. |
| Success criteria | At least one strong simulation paper can stand without hardware; learning papers only include trained/evaluated methods. |
| Risks | Too many partial stories; overbroad paper claims. |
| Stop/go decision | Publish operating-envelope paper first if learning results are delayed. |
| Local feasibility | Yes. |
| Needs GPU | No unless generating new learning results. |
| Needs hardware | No. |

### Phase 8: Hardware/Sim-to-Real Future Validation Package

| Field | Plan |
|---|---|
| Objective | Prepare a future hardware validation package without executing hardware work in the current planning scope. |
| Scientific importance | Defines what must be true before physical robot claims are made. |
| Proposal relation | Supports transfer and safety phase, but remains future work until approvals and resources exist. |
| Recommended framework | Gazebo-validated controller and policies; optional Isaac camera calibration assets; no hardware execution now. |
| Inputs | Simulation evidence, safety gates, hardware protocol, sim-to-real plan, transfer thresholds. |
| Implementation tasks | Define acceptance gates, required calibration measurements, logging schema, emergency-stop criteria, minimum safe 1.0 mm validation plan. |
| Expected outputs | Hardware-readiness checklist, transfer-risk matrix, calibration protocol, future trial package. |
| Validation metrics | Required future metrics: tracking noise, force noise, depth noise, Safe Success, hard violations, bounded wrench. |
| Success criteria | No hardware trial begins until simulation claims are stable, safety approval exists, and 1.0 mm envelope is the first physical target. |
| Risks | Real robot differs from simulation; force/camera calibration is unavailable; safety approval delay. |
| Stop/go decision | Stop before hardware if real tracking noise exceeds simulated envelope or force sensing cannot be calibrated. |
| Local feasibility | Documentation yes. |
| Needs GPU | No. |
| Needs hardware | No for package creation; yes only for future execution outside this plan. |

## Recommended Hybrid Architecture

### Top-Level Architecture

```text
shared_scenario_yaml
  -> Gazebo adapter
  -> MuJoCo adapter
  -> Isaac Sim adapter

all adapters emit:
  shared_dataset_schema
  shared_result_schema
  shared_metrics_schema

all policies use:
  shared_observation_schema
  shared_action_schema
  deterministic_safety_gate

final claim path:
  train/pilot in fast simulator
  -> safety-gated replay in Gazebo
  -> evidence package
  -> optional future hardware package
```

### Required Components

| Component | Purpose | Authority |
|---|---|---|
| ROS 2/Gazebo validation harness | Runs KUKA task, controller, launch, logging, safety checks | Final simulation authority |
| MuJoCo fast RL environment | Trains SAC/meta-RL and runs randomization sweeps | Training authority only |
| Isaac Sim synthetic perception environment | Generates RGB/depth/segmentation data and visual randomization | Perception-data authority only |
| Shared scenario YAML/config schema | Defines geometry, clearance, offset, friction, noise, sensor state, seed | Cross-simulator contract |
| Shared dataset schema | Stores rows, feature masks, scenario IDs, contact labels, visual validity | Evidence contract |
| Shared policy/action interface | Defines high-level action, Cartesian delta, residual, or advisory output | Safety-mediated interface |
| Shared evaluation metrics | Safe success, fail-closed rate, unsafe rate, force/wrench, time, adaptation speed | Comparison contract |
| Deterministic safety gate | Blocks unsafe actions and authorizes final insertion state | Final authority |
| ML advisory/residual policy | Suggests phase/action/residual only within safety constraints | Never final authority |

### Shared Scenario Fields

Minimum scenario fields:

- scenario ID and seed.
- peg diameter, hole diameter, radial clearance, initial XY offset.
- peg/hole geometry class and chamfer assumptions.
- contact parameters: friction, stiffness/compliance, damping, restitution if available.
- sensor parameters: joint noise, wrench noise/bias, depth validity, RGB validity, camera pose.
- controller parameters and safety thresholds.
- simulator adapter and version.

### Shared Dataset Fields

Minimum dataset fields:

- scenario ID, trial ID, row timestamp, simulator.
- task phase and safety state.
- joint position and velocity.
- wrench/contact fields with source labels.
- RGB/depth features plus feature-validity masks.
- geometry-only feasibility label.
- action/advisory/residual output if present.
- result linkage to outcome JSON.

### Shared Policy/Action Interface

The policy interface should support multiple authority levels:

- `advisory_phase`: direction hint only.
- `feasibility_advice`: in-envelope, marginal, out-of-envelope.
- `cartesian_delta`: bounded proposal under safety projection.
- `residual_delta`: small correction added to deterministic command only under gate.
- `no_action`: fallback to deterministic controller.

INSERT authority remains deterministic unless a future safety case proves otherwise.

## Publication Strategy

### Paper 1: Operating-Envelope Safety Paper

| Field | Plan |
|---|---|
| Main claim | A safety-gated ROS 2/Gazebo peg-in-hole framework can define a measured clearance/noise operating envelope and fail closed when tracking noise exceeds clearance. |
| Additional work | Confidence intervals, failure taxonomy, clearer clearance/noise model, artifact package. |
| Required experiments | Existing Stage C plus optional short ablations around 0.75, 1.25, and 1.5 mm after audits. |
| Target contribution | Safety-envelope characterization for contact-rich industrial assembly. |
| Minimum publishable evidence | 140 Stage C trials, 72/80 robust success, 0/60 fail-closed, failure modes, reproducibility package, explicit simulation-only limitation. |

### Paper 2: Simulation Framework and Contact-Fidelity Paper

| Field | Plan |
|---|---|
| Main claim | Contact-rich learning for peg-in-hole needs a hybrid simulator architecture with Gazebo for ROS validation, MuJoCo for fast training, and Isaac Sim for perception data. |
| Additional work | Contact-label reconciliation, simulator-neutral contract, MuJoCo pilot, optional Isaac visual pilot. |
| Required experiments | Contact-source agreement audit, reward/termination parity, reset determinism, limited cross-simulator scenario replay. |
| Target contribution | Practical simulator-contract methodology for reproducible contact-rich robotics learning. |
| Minimum publishable evidence | Contract maps Stage C scenarios losslessly; contact-label limitations quantified; at least one fast-simulator pilot validates scenario/result parity. |

### Paper 3: Cross-Scenario Representation Learning Result

| Field | Plan |
|---|---|
| Main claim | Baseline-trained per-tick neural advisory fails to generalize across geometry/tolerance scenarios under weak visual/contact observations; geometry-only feasibility is a safer envelope-aware signal. |
| Additional work | Observation validity audit, balanced row-level data, optional sequence-aware model, feature masks. |
| Required experiments | Mixed and held-out scenario evaluation, geometry-only feasibility comparison, sequence model ablation if data supports it. |
| Target contribution | Honest positive/negative representation-learning study for safety-gated assembly. |
| Minimum publishable evidence | Current 38.9% mixed / 20.9% held-out negative result, 96.4% geometry-only feasibility with 0% false-safe, documented feature limitations. |

### Paper 4: SAC/Meta-RL Assembly Paper

| Field | Plan |
|---|---|
| Main claim | Context-conditioned or randomized-policy learning improves adaptation or robustness over single-task SAC under safety constraints. |
| Additional work | MuJoCo environment, real SAC training, multi-seed evaluation, meta-RL after SAC baseline. |
| Required experiments | SAC vs deterministic vs geometry advisory; meta-RL vs SAC on held-out scenario families; Gazebo replay for candidates. |
| Target contribution | Safety-constrained learning for adaptable peg-in-hole assembly. |
| Minimum publishable evidence | Trained policies, multi-seed learning curves, held-out scenario metrics, unsafe/fail-closed rates, Gazebo validation of policy candidates. |

### Paper 5: Sim-to-Real Validation Paper

| Field | Plan |
|---|---|
| Main claim | A safety-gated simulation-trained insertion stack transfers conservatively to a real KUKA cell inside a measured operating envelope. |
| Additional work | Hardware approval, calibration, real tracking-noise measurement, real D405/F/T validation, slow 1.0 mm trials. |
| Required experiments | No-contact repeatability, contact threshold calibration, 1.0 mm physical trials, bounded-force validation. |
| Target contribution | Conservative sim-to-real validation for collaborative assembly. |
| Minimum publishable evidence | Real hardware metrics showing safe success, bounded force, no hard violations, and measured agreement/disagreement with simulation. |

## First Implementation Sprint After This Roadmap

Do not implement this now. The first sprint after this strategic plan should be:

**Observation and contact fidelity audit for hybrid simulation readiness.**

Recommended sprint scope:

1. Add an observation validity audit for row-level datasets.
2. Add feature-validity masks for RGB, depth, wrench/contact, and scenario metadata.
3. Reconcile contact labels: wrench-derived, contact-topic, geometric/proximity, and task-state.
4. Document `SIMULATION_DATA_CONTRACT.md`.
5. Run static checks and only short smoke validation if necessary.

Non-goals for that sprint:

- no SAC training,
- no meta-RL,
- no MuJoCo or Isaac adapter implementation,
- no long Stage C rerun,
- no controller behavior change,
- no hardware execution.

## What Not To Do Yet

- Do not train SAC before simulator/observation/contact validity is fixed.
- Do not start meta-RL before a real SAC baseline exists.
- Do not migrate everything away from ROS 2/Gazebo.
- Do not continue coding inside Gazebo just because it is the current framework.
- Do not add Isaac Sim as a full-stack replacement before defining the shared contract.
- Do not claim full meta-RL before real training and held-out evaluation.
- Do not claim visual learning while RGB/depth features are empty.
- Do not claim force-aware ML while F/T features are zero or contact labels disagree.
- Do not claim tight-clearance success at <=0.5 mm.
- Do not weaken deterministic safety gates to increase apparent success rate.
- Do not claim hardware readiness beyond protocol planning.
- Do not treat v2_14 as a cross-scenario controller; it is advisory and currently fails cross-scenario generalization.
- Do not let ML authorize INSERT or DONE outside deterministic confirmation.
- Do not create GitHub issues or hardware steps until explicitly requested.

## Honest Claims-Vs-Evidence Warning

Safe current claims:

- The ROS 2/Gazebo system is operational and reproducible.
- The deterministic safety-gated controller is validated in simulation.
- The 1.0 mm clearance family is robust in the current simulator.
- The <=0.5 mm clearance family fails closed under the current tracking-noise floor.
- v2_14 is useful only as a guarded advisory inside its validated distribution.
- Geometry-only feasibility is currently the strongest envelope-aware advisory signal.
- SAC/meta-RL infrastructure is scaffolded, not trained.

Unsafe current claims:

- universal peg-in-hole generalization,
- sub-millimeter insertion success,
- trained SAC or meta-RL,
- validated visual-depth learning,
- validated force-aware neural control,
- hardware transfer,
- contact-fidelity proof from Gazebo contact topics alone,
- ML authority during INSERT.

## Final Conclusion

The strongest PhD path is not an immediate coding sprint, not a full simulator migration, and not premature SAC/meta-RL training. The strongest path is:

1. consolidate the operating-envelope evidence,
2. fix or bound observation/contact validity,
3. define a simulator-neutral contract,
4. add MuJoCo for fast learning,
5. add Isaac Sim only for the visual/depth track,
6. keep Gazebo as the final simulation-validation harness,
7. keep deterministic safety as final authority,
8. publish the operating-envelope and negative/positive representation results honestly,
9. train SAC/meta-RL only after the simulator/data foundation can support the claims.

The final recommended framework is therefore a **staged ROS 2/Gazebo + MuJoCo + Isaac Sim hybrid**, not a single-simulator thesis stack.
