# Local Planning Backlog

Date: 2026-06-13
Note: This is a local Markdown backlog only. No GitHub issues have been created.

## Simulation Framework

### Define simulator-neutral scenario/result contract

- Priority: P0
- Labels: `simulation`, `architecture`, `documentation`, `no-code-first`
- Description: Define shared scenario, observation, action, reward, safety, and result schemas before adding MuJoCo or Isaac Sim.
- Acceptance criteria: Stage C scenarios map into the contract; feature validity masks exist; Gazebo remains the authoritative validation harness.

### Keep ROS 2/Gazebo as validation harness

- Priority: P0
- Labels: `gazebo`, `ros2`, `validation`
- Description: Preserve existing Gazebo launch, controller, logger, and diagnostics as the source of system-level evidence.
- Acceptance criteria: Any future fast-simulator policy must be replayed or evaluated in Gazebo before being claimed.

### Plan MuJoCo fast-RL pilot

- Priority: P1
- Labels: `mujoco`, `rl`, `simulation-framework`
- Description: Design a compact MuJoCo peg-in-hole environment for future SAC/meta-RL rollouts, without implementing it in the immediate sprint.
- Acceptance criteria: Model conversion requirements, action/observation schema, reward parity, and Gazebo validation path are documented.

### Plan Isaac Sim perception-data track

- Priority: P2
- Labels: `isaac-sim`, `perception`, `synthetic-data`, `gpu`
- Description: Define when Isaac Sim should be used for RGB/depth/segmentation data and domain randomization.
- Acceptance criteria: GPU requirement, USD asset needs, camera outputs, and Gazebo compatibility path are documented.

## Data Generation

### Add row-level dataset quality audit

- Priority: P0
- Labels: `dataset`, `quality`, `perception`
- Description: Audit context vectors for feature validity, phase balance, scenario coverage, and metadata completeness.
- Acceptance criteria: Report flags empty RGB/depth features, sparse SEARCH rows, missing metadata, and scenario imbalance.

### Expand row-level scenario coverage after audit

- Priority: P2
- Labels: `dataset`, `cross-scenario`, `simulation-only`
- Description: Collect more row-level trials only after validity checks exist.
- Acceptance criteria: Balanced scenario rows, enough SEARCH/DONE examples, and all rows include scenario metadata.

### Add deterministic seed and scenario manifest capture

- Priority: P1
- Labels: `reproducibility`, `dataset`, `scenario`
- Description: Ensure each generated trial records seed, geometry, clearance, offset, contact parameters, sensor validity, and controller config.
- Acceptance criteria: Trial outcome JSON and row-level metadata can be joined by scenario/trial ID.

## Perception Simulation

### Fix empty Gazebo D405 feature generation

- Priority: P0
- Labels: `perception`, `gazebo`, `rgbd`, `bug`
- Description: Current multi-scenario RGB/depth features are empty or non-informative. Diagnose camera placement, topic bridge, image encoding, and ROI extraction.
- Acceptance criteria: Short smoke dataset has nonzero RGB summary, valid depth dimensions, finite ROI depth, and documented camera viewpoint.

### Add visual/depth feature validity masks

- Priority: P0
- Labels: `perception`, `dataset`, `safety`
- Description: Add per-row masks so models know whether RGB/depth/wrench features are valid.
- Acceptance criteria: Classifier training can filter or condition on invalid features without silently treating zeros as real perception.

### Evaluate sequence-aware phase classifier offline

- Priority: P2
- Labels: `perception`, `ml`, `offline`
- Description: Address RETREAT/DONE ambiguity and sparse SEARCH rows with temporal context after data quality is fixed.
- Acceptance criteria: Held-out trial evaluation improves DONE precision and SEARCH recall without changing safety fallbacks.

## Force/Contact Simulation

### Reconcile contact labels

- Priority: P0
- Labels: `contact`, `force`, `diagnostics`
- Description: Compare wrench-derived, contact-topic, geometric, and task-state contact labels.
- Acceptance criteria: Contact reports show agreement/disagreement; contact-topic zero-positive limitation remains visible.

### Add contact parameter randomization plan

- Priority: P1
- Labels: `contact`, `domain-randomization`, `simulation`
- Description: Define friction, stiffness/compliance, damping, restitution, chamfer, and surface variation ranges.
- Acceptance criteria: Randomization ranges are recorded per trial and tied to outcome metrics.

### Study clearance/noise ratio with finer bins

- Priority: P2
- Labels: `operating-envelope`, `contact`, `publication`
- Description: Extend simulation study around 0.75, 1.0, 1.25, and 1.5 mm clearance after the audit sprint.
- Acceptance criteria: Confidence intervals support the transition between fail-closed and robust envelopes.

## RL/Meta-RL Simulation

### Do not train SAC until simulator contract is stable

- Priority: P0
- Labels: `rl`, `blocked`, `research-integrity`
- Description: SAC training should wait until observation validity, contact labels, and scenario/result contracts are in place.
- Acceptance criteria: No training claims appear before contract and smoke checks are complete.

### Build fast RL environment wrapper

- Priority: P1
- Labels: `rl`, `mujoco`, `gymnasium`, `simulation`
- Description: After the next sprint, implement a minimal fast simulator wrapper for SAC smoke tests.
- Acceptance criteria: Reset determinism, action bounds, reward/termination parity, and scenario sampling are tested without long training.

### Prepare SAC multi-seed cluster evaluation

- Priority: P2
- Labels: `rl`, `gpu`, `cluster`, `evaluation`
- Description: Convert scaffold into a real training/evaluation protocol only after the simulator is ready.
- Acceptance criteria: Multi-seed training plan includes deterministic baseline comparison, held-out scenarios, unsafe rate, and fail-closed rate.

### Defer meta-RL until SAC baseline exists

- Priority: P2
- Labels: `meta-rl`, `deferred`, `gpu`
- Description: PEARL/MAML/RL2-style meta-RL should not begin before SAC and scenario randomization are validated.
- Acceptance criteria: Meta-RL issue remains blocked until SAC baseline has real results.

## Documentation

### Publish local research audit

- Priority: P0
- Labels: `documentation`, `audit`, `research`
- Description: Preserve the current strengths, weaknesses, evidence quality, and overclaiming risks.
- Acceptance criteria: Audit explicitly states simulation-only status, tight-clearance failure, weak neural generalization, and scaffold-only SAC/meta-RL.

### Document simulation framework decision

- Priority: P0
- Labels: `documentation`, `simulation-framework`
- Description: Compare Gazebo, Gazebo Sim, Isaac Sim, MuJoCo, Drake, PyBullet, and hybrid approaches.
- Acceptance criteria: Recommendation answers what stays in Gazebo, what can move later, and what should not be implemented yet.

### Keep GitHub backlog local until requested

- Priority: P0
- Labels: `documentation`, `process`
- Description: Maintain this file as local planning only.
- Acceptance criteria: No GitHub issues are created without an explicit request.

## Publication Readiness

### Turn operating envelope into a simulation study

- Priority: P1
- Labels: `publication`, `operating-envelope`, `simulation-study`
- Description: Formalize the Stage C result as a safety-gated tolerance/noise operating-envelope study.
- Acceptance criteria: Study includes factorial design, confidence intervals, failure taxonomy, limitation statements, and reproducible artifacts.

### Preserve negative results as contributions

- Priority: P1
- Labels: `publication`, `negative-results`, `ml`
- Description: Encoder ablation, v2_14 cross-scenario weakness, and tight-clearance fail-closed behavior should be reported honestly.
- Acceptance criteria: Manuscript outline includes these as limitations and scientific findings, not hidden failures.

### Define fair method-comparison table

- Priority: P2
- Labels: `publication`, `evaluation`, `rl`
- Description: Prepare comparison categories for deterministic, geometry-feasibility advisory, v2_14 advisory, SAC, and meta-RL without filling untrained results.
- Acceptance criteria: Untrained methods are marked scaffold/future work until real results exist.

## Direct Answers

1. Best simulation framework for the next phase: ROS 2/Gazebo as the main validation framework, with a hybrid contract prepared for future MuJoCo and Isaac Sim.
2. Keep Gazebo or add Isaac/MuJoCo: Keep Gazebo now. Add MuJoCo later for fast RL. Add Isaac Sim later for visual/depth synthetic data if GPU resources exist.
3. Best way to train SAC/meta-RL later: Use a simulator-neutral gym-style contract, train first in a fast simulator with scenario randomization, then validate in Gazebo. Run multi-seed cluster training only after observation/contact validity is fixed.
4. Better contact/tolerance modeling: Model clearance/noise ratio explicitly; randomize friction, stiffness/compliance, damping, chamfer, sensor noise, and initial offset; keep fail-closed tight-clearance behavior.
5. Useful visual/depth features: Fix camera viewpoint/topic validity, add ROI depth around peg/hole, add feature-validity masks, and consider Isaac Sim synthetic RGB/depth/segmentation if Gazebo remains empty.
6. Publishable operating-envelope study: Present Stage C as a measured safety envelope with confidence intervals, failure taxonomy, and honest negative tight-clearance result.
7. Exact next sprint: Observation and contact fidelity audit for hybrid simulation readiness.
8. What should not be done yet: No hardware execution, no long training, no full simulator migration, no architecture rewrite, no claims of universal tolerance generalization, and no claims that SAC/meta-RL is trained.
