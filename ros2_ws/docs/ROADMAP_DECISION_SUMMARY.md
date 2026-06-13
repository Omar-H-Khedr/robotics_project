# Roadmap Decision Summary

Date: 2026-06-13
Basis:

- `docs/FULL_PHD_COMPLETION_ROADMAP.md`
- `docs/SIMULATION_FRAMEWORK_REDESIGN_PLAN.md`
- `docs/NEXT_SIMULATION_SPRINT_PLAN.md`
- `docs/LITERATURE_AWARE_UPGRADE_PLAN.md`
- `docs/LOCAL_PROJECT_RESEARCH_AUDIT.md`

This is a decision summary. It does not authorize implementation, Gazebo runs,
training, hardware execution, or new experiments.

## 1. Final Recommended Simulation Framework

Use a staged hybrid framework:

**ROS 2/Gazebo + MuJoCo + Isaac Sim**

with strict role separation:

- ROS 2/Gazebo remains the authoritative system-validation harness.
- MuJoCo becomes the fast contact-rich RL/meta-RL training simulator after the
  simulator-neutral contract and data-validity gates exist.
- Isaac Sim is added only for the RGB-D/synthetic-perception track if GPU,
  installation, and setup resources are available.
- Deterministic safety remains the final authority in every simulator.

Resource-constrained fallback:

**ROS 2/Gazebo + MuJoCo**

This fallback is acceptable only if the thesis does not make strong visual
learning or RGB-D generalization claims.

## 2. Why This Framework Is the Best Choice

No single simulator currently covers the full PhD need.

ROS 2/Gazebo is best for:

- current evidence continuity,
- ROS 2 launch reproducibility,
- KUKA task integration,
- `ros2_control` validation,
- deterministic controller regression,
- safety-gated system evaluation.

MuJoCo is best for:

- fast contact-rich rollouts,
- SAC training,
- later PEARL-style or recurrent meta-RL,
- dense domain randomization,
- reward and termination ablations.

Isaac Sim is best for:

- D405-like RGB/depth synthetic data,
- segmentation masks,
- lighting/material/camera domain randomization,
- perception-data generation where current Gazebo data is weak.

The project already has strong Gazebo evidence. Replacing Gazebo would destroy
comparability. Keeping Gazebo alone would leave the project weak on fast RL and
visual data. The staged hybrid gives the project the missing capabilities while
preserving the validated evidence base.

## 3. What Remains in ROS 2/Gazebo

Keep these in ROS 2/Gazebo:

- KUKA LBR iisy 6 R1300 workcell validation.
- Existing launch orchestration, especially the research baseline.
- `ros2_control` controller checks.
- Deterministic task-controller regression.
- Safety-gate validation.
- Fail-closed behavior validation.
- Stage C operating-envelope reproduction.
- Short observation/contact smoke checks.
- Final replay of learned policies or residuals before thesis claims.
- Outcome JSON, row-level logs, scenario manifests, and metrics summaries.
- System-level evidence used in thesis claims.

Gazebo is the final simulation authority. A MuJoCo or Isaac result is not a
system-level thesis result until it is replayed or validated through the Gazebo
harness under the shared scenario/result contract.

## 4. What Moves to MuJoCo

Move these to MuJoCo after the contract and validity sprint:

- Fast SAC rollouts.
- Later context/meta-RL training.
- PEARL-style or recurrent adaptation experiments.
- Compact contact-rich peg-in-hole environments.
- Domain randomization over clearance, offset, friction, damping, compliance,
  contact parameters, and noise.
- Reward, termination, and safety-ablation sweeps.
- Curriculum learning.
- Tolerance sweeps around the 0.5 mm to 1.5 mm transition region.
- Policy pretraining before Gazebo replay.

MuJoCo outputs are candidate policies or candidate residuals. They are not final
evidence until they pass Gazebo validation.

## 5. What Moves to Isaac Sim

Move these to Isaac Sim only if the visual/perception contribution remains
important and GPU resources are available:

- Synthetic RGB data.
- Synthetic depth data.
- Segmentation masks for peg, hole, fixture, table, and workspace.
- Camera-pose randomization.
- Lighting, material, texture, and occlusion randomization.
- D405-style perception-data generation.
- Visual feature pretraining or perception-data validation.

Isaac Sim should not replace Gazebo. It is a perception-data environment, not
the first full-stack simulator migration target.

## 6. What Should Not Be Migrated

Do not migrate these away from ROS 2/Gazebo:

- Existing validated system-level evidence.
- KUKA launch and task orchestration.
- Safety-gate authority.
- Deterministic task controller validation.
- Final thesis-level simulation evaluation.
- Stage C operating-envelope evidence.
- Logging and artifact-generation paths used by the current evidence base.

Do not migrate the whole project to Isaac Sim. Do not use PyBullet for final
tight-clearance contact claims. Do not make Drake the main simulator. Drake may
be useful only for optional contact-model analysis.

## 7. Complete Project Roadmap

### Phase 1: Consolidate Current Evidence

Freeze the current simulation-only evidence into a claims-safe package.

Outputs:

- operating-envelope tables and figures,
- confidence intervals,
- failure taxonomy,
- claims-vs-evidence matrix,
- explicit limitations.

Do not hide the negative results:

- 0/60 Stage C successes at <=0.5 mm clearance,
- weak v2_14 cross-scenario neural generalization,
- empty or weak RGB/depth features,
- incomplete contact-topic evidence,
- no trained SAC/meta-RL,
- no hardware validation.

### Phase 2: Fix or Bound Observation and Contact Validity

Before learning, audit the data.

Required outputs:

- observation-validity audit,
- feature-validity masks,
- RGB/depth validity report,
- contact-label reconciliation,
- source-tagged contact labels,
- clear statement of which features are invalid.

Contact labels must separate:

- wrench-derived contact,
- Gazebo contact-topic contact,
- geometric/proximity contact,
- task-state contact.

### Phase 3: Define the Simulator-Neutral Contract

Create the shared scenario, observation, action, reward, termination, safety,
dataset, and result schema.

The contract must represent Stage C scenarios losslessly before any simulator
adapter becomes research infrastructure.

### Phase 4: Add MuJoCo Pilot

Add a minimal MuJoCo environment only after the contract is stable.

First MuJoCo success means:

- deterministic reset works,
- scenario sampling matches the contract,
- reward and termination fields match the Gazebo result schema,
- no training or performance claim is made from the pilot alone.

### Phase 5: Add Isaac Visual Track if Needed

Add Isaac Sim only if the thesis keeps a serious visual/RGB-D contribution and
resources are available.

First Isaac success means:

- valid RGB/depth frames,
- valid segmentation or object labels,
- recorded camera and domain-randomization parameters,
- scenario metadata compatible with the shared contract.

### Phase 6: Rebuild Representation Learning

Use only validity-audited data.

Compare:

- raw context,
- geometry-only feasibility,
- valid visual/depth features,
- contact-aware features,
- sequence-aware models if row sequences are adequate.

Keep ML advisory. Do not give ML unsafe INSERT authority.

### Phase 7: Train SAC, Then Meta-RL

Train SAC only after:

- observation/contact validity is handled,
- the shared contract exists,
- MuJoCo pilot behavior is deterministic enough,
- reward and termination semantics are stable.

Train meta-RL only after SAC is a real baseline. Stop meta-RL if SAC does not
establish a credible learned-control baseline.

### Phase 8: Fair Benchmark

Compare only methods with real evidence:

- deterministic controller,
- geometry feasibility advisory,
- v2_14 advisory within its validated distribution,
- SAC if trained,
- meta-RL if trained.

Report:

- safe success,
- unsafe rate,
- fail-closed rate,
- fallback rate,
- insertion time,
- failure modes,
- confidence intervals,
- held-out scenario performance.

### Phase 9: Publish Simulation Results

Publish the strongest simulation evidence first.

Priority:

1. operating-envelope safety paper,
2. simulation-framework/contact-fidelity paper,
3. representation-learning positive/negative result,
4. SAC/meta-RL paper only after real training and Gazebo validation.

### Phase 10: Hardware Readiness Package

Prepare hardware validation only after simulation claims are stable.

Required before hardware claims:

- real tracking-noise measurement,
- real force/torque calibration,
- real D405 calibration,
- safety approval,
- first physical target limited to the 1.0 mm clearance envelope.

No hardware validation has been executed yet.

## 8. First Implementation Sprint After Planning

The first sprint should be:

**Observation and contact fidelity audit for hybrid simulation readiness.**

Scope:

- add observation-validity audit for row-level datasets,
- add feature-validity masks for RGB, depth, wrench/contact, and metadata,
- reconcile wrench-derived, contact-topic, geometric/proximity, and task-state
  contact labels,
- document `SIMULATION_DATA_CONTRACT.md`,
- run static checks and only short smoke validation if necessary.

Non-goals:

- no SAC training,
- no meta-RL,
- no MuJoCo adapter,
- no Isaac adapter,
- no full Stage C rerun,
- no controller behavior change,
- no hardware execution.

## 9. What Should Not Be Done Yet

Do not:

- train SAC,
- train meta-RL,
- implement MuJoCo before the contract and validity gates,
- implement Isaac Sim before the visual-data need and GPU resources are clear,
- migrate the whole stack away from ROS 2/Gazebo,
- run long Gazebo experiment batches,
- rerun Stage C as a substitute for fixing data validity,
- relax safety gates to improve tight-clearance success,
- claim <=0.5 mm insertion success,
- claim visual learning from empty RGB/depth features,
- claim force-aware ML from zero or inconsistent F/T/contact features,
- claim hardware readiness beyond protocol planning,
- let ML authorize INSERT or DONE outside deterministic confirmation.

## 10. Top 5 Project Risks

1. **Observation invalidity.**
   Current Gazebo RGB/depth features are empty or weak. Visual-learning claims
   are not defensible until this is fixed or moved to Isaac Sim.

2. **Contact-label disagreement.**
   Task-side wrench evidence and Gazebo contact-topic evidence do not yet form a
   clean contact-fidelity story.

3. **Simulator mismatch.**
   MuJoCo-trained policies may not replay successfully in Gazebo. If they fail
   Gazebo replay, report the simulator gap instead of claiming policy success.

4. **Learning underperformance.**
   SAC or meta-RL may fail to beat the deterministic baseline safely. That is a
   valid negative result, but it weakens the learned-control contribution.

5. **Overclaiming.**
   The project is strong as a simulation-only safety-envelope study. It becomes
   weak if it claims trained SAC, meta-RL, visual generalization, contact-topic
   fidelity, sub-millimeter success, or hardware transfer without evidence.

## 11. Top 5 Publication-Strength Upgrades

1. **Operating-envelope statistics.**
   Add confidence intervals, failure taxonomy, and a clearance/noise model around
   the 1.0 mm success and <=0.5 mm fail-closed boundary.

2. **Observation-validity audit.**
   Turn the current RGB/depth weakness into explicit evidence and prevent invalid
   visual features from supporting claims.

3. **Contact-label reconciliation.**
   Quantify agreement and disagreement among wrench, contact-topic, geometry,
   and task-state labels.

4. **Simulator-neutral contract.**
   Make Gazebo, MuJoCo, and Isaac results comparable through shared scenario,
   action, reward, termination, dataset, and metric schemas.

5. **Sequence-aware representation study.**
   Replace per-tick overinterpretation with phase-aware models or filters, while
   preserving deterministic safety fallback.

## 12. Shortest Path to a Publishable Simulation Paper

Write the first paper around the current strongest result:

**Safety-gated ROS 2/Gazebo peg-in-hole framework with a measured
clearance/noise operating envelope.**

Minimum path:

1. Consolidate the 140 Stage C trials.
2. Report 72/80 successes at 1.0 mm clearance.
3. Report 0/60 successes at <=0.5 mm clearance as fail-closed behavior, not as
   solved insertion.
4. Add confidence intervals.
5. Add failure taxonomy.
6. State the tracking-noise interpretation: reliable insertion currently needs
   clearance greater than roughly 2x the tracking noise floor.
7. Include the v2_14 cross-scenario result as a negative learning result.
8. Include geometry-only feasibility as a conservative advisory signal.
9. State clearly that SAC/meta-RL is scaffold-only and hardware validation has
   not been executed.

This paper does not require MuJoCo, Isaac Sim, SAC, meta-RL, or hardware.

## 13. Shortest Path to Completing the Full PhD Contribution

The shortest credible path is:

1. Publish the operating-envelope safety result from current Gazebo evidence.
2. Complete the observation/contact validity sprint.
3. Define the simulator-neutral contract.
4. Add MuJoCo for fast SAC training.
5. Train and evaluate SAC under scenario randomization.
6. Replay candidate SAC policies through Gazebo.
7. Attempt meta-RL only if SAC establishes a real learned baseline.
8. Add Isaac Sim only if visual/RGB-D learning remains a core thesis claim.
9. Benchmark deterministic, advisory, SAC, and meta-RL methods only where each
   method has actual evidence.
10. Keep hardware as a future validation package until real calibration and
    safety approval exist.

The full PhD contribution should be framed as:

- a reproducible safety-gated contact-rich assembly framework,
- a measured operating envelope tied to clearance and tracking noise,
- a claims-safe representation-learning study with honest negative results,
- a simulator-contract architecture for adding fast RL and synthetic perception,
- trained SAC/meta-RL only if those policies are actually trained and validated.

The project should not be framed as already having trained SAC, trained
meta-RL, hardware validation, universal generalization, or solved
sub-millimeter insertion.
