# Literature-Aware Upgrade Plan

Date: 2026-06-13
Scope: simulation and research roadmap before new implementation.

## Literature and Framework Context

The project evidence in this document is local. External context was used only to position simulator and method choices:

- Gazebo ROS 2 integration: https://gazebosim.org/docs/latest/ros2_integration/
- Gazebo sensors: https://gazebosim.org/docs/latest/sensors/
- MuJoCo overview/docs: https://mujoco.readthedocs.io/
- MuJoCo project overview: https://mujoco.org/
- NVIDIA Isaac Sim docs: https://docs.isaacsim.omniverse.nvidia.com/
- Isaac Sim Replicator/domain randomization: https://docs.nvidia.com/learning/physical-ai/getting-started-with-isaac-sim/latest/synthetic-data-generation-for-perception-model-training-in-isaac-sim/03-domain-randomization-with-replicator.html
- Drake hydroelastic contact guide: https://drake.mit.edu/doxygen_cxx/group__hydroelastic__user__guide.html
- Domain randomization paper: https://arxiv.org/abs/1703.06907
- MAML paper: https://arxiv.org/abs/1703.03400
- PEARL paper: https://arxiv.org/pdf/1903.08254
- Contact-rich manipulation sim-to-real work: https://openreview.net/forum?id=gFXVysXh48K

## Classification Summary

| Class | Meaning | Upgrades |
|---|---|---|
| A | Must-have before next coding sprint | Observation validity audit; contact label reconciliation; scenario/result contract |
| B | Best simulation-framework upgrade | Hybrid Gazebo plus MuJoCo pilot plan |
| C | Good for publication strength | Operating-envelope statistical study; sequence-aware perception study; domain randomization design |
| D | Needs GPU cluster | SAC training; PEARL/meta-RL; Isaac synthetic data at scale |
| E | Future hardware work | Real KUKA calibration; real D405/F/T validation; sim-to-real transfer |

## A1. Observation Validity and Visual/Depth Repair Plan

- Classification: A
- Scientific reason: The cross-scenario v2_14 result is weak partly because RGB/depth features are empty or non-informative. A vision-based thesis cannot build on invalid visual observations.
- Relation to proposal: Directly supports visuomotor context learning and D405 perception.
- Modern technique direction: Use feature-validity masks, ROI depth summaries, segmentation-aware synthetic labels, and dataset gates that reject empty images.
- Expected benefit: Turns perception from placeholder features into measurable observation evidence.
- Difficulty: Medium.
- Required resources: Local Gazebo, existing D405 topics, perception logger, short smoke trials.
- Simulation-only: Yes.
- Needs GPU: No for validity checks; optional later for synthetic data.
- Needs hardware later: Yes, for real D405 calibration.
- Deliverable: A documented observation-quality report and pass/fail checks for RGB/depth context rows.
- Validation metric: 100% of accepted dataset rows have nonzero RGB validity, nonzero depth dimensions, finite ROI depth, and scenario-linked metadata.

## A2. Contact Label Reconciliation

- Classification: A
- Scientific reason: Contact-rich insertion claims need reliable contact evidence. Current task-side wrench and Gazebo contact-topic evidence disagree.
- Relation to proposal: Supports virtual-force safety and contact-aware assembly.
- Modern technique direction: Multi-source contact labels combining geometry, contact topics, wrench thresholds, and phase-state transitions.
- Expected benefit: Stronger contact-fidelity claims and better RL rewards.
- Difficulty: Medium.
- Required resources: Existing contact observers, wrench logs, outcome JSONs, short Gazebo diagnostics.
- Simulation-only: Yes.
- Needs GPU: No.
- Needs hardware later: Yes, to calibrate real F/T thresholds.
- Deliverable: Contact-label schema and local audit of contact-topic, wrench-derived, and geometry-derived agreement.
- Validation metric: Agreement matrix over contact labels; no publication claim unless contact source agreement and known exceptions are reported.

## A3. Simulator-Neutral Scenario and Result Contract

- Classification: A
- Scientific reason: Adding a fast simulator without a shared contract will create incomparable results.
- Relation to proposal: Enables systematic multi-scenario learning and fair baseline comparisons.
- Modern technique direction: Gymnasium-style environment contracts, deterministic seeds, structured scenario metadata, and feature validity masks.
- Expected benefit: Allows Gazebo, MuJoCo, and future Isaac Sim to share scenarios and metrics.
- Difficulty: Medium.
- Required resources: Existing scenario YAML, SAC contract, row-level dataset metadata.
- Simulation-only: Yes.
- Needs GPU: No.
- Needs hardware later: No, but the same schema can support hardware logs later.
- Deliverable: Contract document and schema checklist before implementation.
- Validation metric: Existing Stage C scenarios can be represented losslessly in the contract.

## B1. Hybrid ROS 2/Gazebo plus MuJoCo RL Pilot

- Classification: B
- Scientific reason: Gazebo validates the system but is inefficient for millions of RL steps. MuJoCo is well suited for fast articulated dynamics and contact-rich RL prototyping.
- Relation to proposal: Supports SAC/meta-RL training without overloading ROS launch loops.
- Modern technique direction: Train residual or high-level policies in MuJoCo, validate candidates back in Gazebo.
- Expected benefit: Practical path to SAC/meta-RL while preserving Gazebo evidence continuity.
- Difficulty: Medium-high.
- Required resources: MJCF/URDF model conversion, compact peg/hole contact model, gym wrapper, scenario contract.
- Simulation-only: Yes.
- Needs GPU: Optional for large training; CPU can support smoke tests.
- Needs hardware later: Only after Gazebo validation.
- Deliverable: Design spec first, then a minimal no-training environment smoke test in a later sprint.
- Validation metric: Reset determinism, scenario sampling correctness, reward parity with Gazebo outcome schema, no performance claims.

## B2. Isaac Sim Visual/Depth Data Track

- Classification: B and D
- Scientific reason: Isaac Sim is stronger than the current Gazebo setup for synthetic RGB/depth, segmentation, and domain randomization.
- Relation to proposal: Supports visuomotor context features and eventual sim-to-real perception.
- Modern technique direction: Replicator-style randomized lighting, materials, camera pose, segmentation masks, and depth supervision.
- Expected benefit: Replaces empty D405 features with realistic synthetic perception data.
- Difficulty: High.
- Required resources: NVIDIA GPU, Isaac Sim install, USD assets, camera calibration workflow.
- Simulation-only: Yes.
- Needs GPU: Yes.
- Needs hardware later: Yes, for real camera validation.
- Deliverable: Future synthetic data generation plan and small pilot dataset.
- Validation metric: Depth/segmentation validity, domain coverage, classifier improvement on held-out visual scenarios.

## C1. Publishable Operating-Envelope Simulation Study

- Classification: C
- Scientific reason: The strongest local result is the measured clearance/noise envelope.
- Relation to proposal: Provides the safety and tolerance-generalization foundation for adaptive assembly.
- Modern technique direction: Factorial simulation design, confidence intervals, failure-mode taxonomy, and safety-envelope modeling.
- Expected benefit: Converts Stage C from an internal validation into a publication-ready result.
- Difficulty: Medium.
- Required resources: Existing Stage C data, analysis scripts, possibly short additional simulation-only ablations.
- Simulation-only: Yes.
- Needs GPU: No.
- Needs hardware later: Optional follow-up, not required for simulation paper.
- Deliverable: Study design and paper-ready figure/table checklist.
- Validation metric: Confidence intervals for 1.0 mm vs <=0.5 mm families; explicit no-universal-generalization statement.

## C2. Sequence-Aware Perception Upgrade

- Classification: C
- Scientific reason: DONE vs RETREAT ambiguity and sparse SEARCH rows expose the limits of per-tick classification.
- Relation to proposal: Improves context-based advisory without giving ML unsafe INSERT authority.
- Modern technique direction: Temporal CNN, GRU/LSTM, Transformer encoder, hidden Markov smoothing, or simple phase-transition filters.
- Expected benefit: Better phase recognition and fewer structurally impossible predictions.
- Difficulty: Medium.
- Required resources: Row-level data with phase sequences, additional balanced DONE/SEARCH rows.
- Simulation-only: Yes.
- Needs GPU: Helpful but not required for small models.
- Needs hardware later: Optional.
- Deliverable: Offline sequence-classification ablation.
- Validation metric: Improved RETREAT/DONE precision/recall and SEARCH recall on held-out trials, with safety fallback unchanged.

## C3. Domain Randomization Design and Dry-Run Plan

- Classification: C
- Scientific reason: Sim-to-real transfer and robust learning require parameter variation, but randomization must be measured rather than assumed.
- Relation to proposal: Supports adaptable assembly under product and sensor variation.
- Modern technique direction: Domain randomization over dynamics, contact, sensor noise, camera pose, geometry tolerances, and materials.
- Expected benefit: More credible generalization and later SAC training.
- Difficulty: Medium.
- Required resources: Scenario generator, SDF parameter injection, noise nodes or log-level perturbations.
- Simulation-only: Yes.
- Needs GPU: No for dry-runs; yes for learning at scale.
- Needs hardware later: Yes for range calibration.
- Deliverable: Randomization matrix and short no-training validation plan.
- Validation metric: Randomized parameters are recorded per trial and outcome variance is analyzable.

## D1. SAC Training at Scale

- Classification: D
- Scientific reason: SAC can learn continuous control policies but needs many simulator interactions.
- Relation to proposal: Required for the SAC baseline comparison.
- Modern technique direction: Off-policy SAC with scenario randomization, safety-aware reward, fail-closed termination, and deterministic baseline comparison.
- Expected benefit: Provides the missing learned-control baseline.
- Difficulty: High.
- Required resources: GPU cluster, fast simulator or efficient Gazebo wrapper, seeded evaluation, checkpointing.
- Simulation-only: Yes.
- Needs GPU: Yes for serious training.
- Needs hardware later: Only after simulation validation.
- Deliverable: Trained SAC checkpoints and multi-seed evaluation.
- Validation metric: Success rate, unsafe rate, fail-closed rate, sample efficiency, and held-out scenario performance versus deterministic baseline.

## D2. PEARL-Style Context-Based Meta-RL

- Classification: D
- Scientific reason: The proposal is about context-based meta-RL. PEARL-style off-policy context inference is a better fit than starting with expensive on-policy meta-RL.
- Relation to proposal: Directly supports context-conditioned adaptation.
- Modern technique direction: Latent context encoder conditioned on recent transitions, task geometry, contact outcomes, and safety status; policy adapts across scenario families.
- Expected benefit: Stronger adaptation story than fixed SAC if enough randomized tasks exist.
- Difficulty: Very high.
- Required resources: Fast simulator, many randomized tasks, GPU cluster, careful evaluation splits.
- Simulation-only: Yes initially.
- Needs GPU: Yes.
- Needs hardware later: Eventually.
- Deliverable: Meta-train/meta-test study with held-out geometries.
- Validation metric: Faster adaptation and better held-out scenario performance than SAC and deterministic baseline, without unsafe insertions.

## E1. Real KUKA Tracking-Noise and Contact Calibration

- Classification: E
- Scientific reason: The current hard limit is tracking noise versus clearance. Hardware must measure the real noise floor before transfer claims.
- Relation to proposal: Required for real assembly validation.
- Modern technique direction: No-contact repeatability tests, contact threshold calibration, force-bias compensation, and slow guarded insertion.
- Expected benefit: Determines whether the 1.0 mm simulation envelope transfers.
- Difficulty: High.
- Required resources: KUKA access, fixture, calibrated F/T sensor, D405 mount, safety approval.
- Simulation-only: No.
- Needs GPU: No.
- Needs hardware later: Yes.
- Deliverable: Hardware calibration report.
- Validation metric: Measured XY noise, force noise, depth noise, and safe 1.0 mm guarded trial outcomes.

## Upgrade Ordering

1. A1, A2, A3 before any coding sprint that claims improved learning or simulation fidelity.
2. C1 in parallel with the next sprint because it leverages existing data.
3. B1 after the contract and observation/contact validity gates are defined.
4. B2 only if GPU and Isaac Sim setup time are available.
5. D1 before D2.
6. E1 only after simulation-only evidence is stable and supervisor safety approval exists.

## What Not To Do Yet

- Do not train SAC/meta-RL before the simulator/data contract is stable.
- Do not migrate the whole stack to Isaac Sim.
- Do not pursue hardware execution as the next main task.
- Do not claim visual/depth generalization while features are empty.
- Do not relax tight-clearance safety gates to turn fail-closed trials into nominal successes.
