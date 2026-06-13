# Local Project Research Audit

Date: 2026-06-13
Scope: local repository audit only for project evidence. External sources are used only for general simulator and literature context in `docs/LITERATURE_AWARE_UPGRADE_PLAN.md`.

## Evidence Base Read

Primary local evidence:

- `README.md`
- `docs/CURRENT_PROJECT_STATUS.md`
- `docs/PROJECT_CONTEXT.md`
- `docs/PROPOSAL_IMPLEMENTATION_MAPPING.md`
- `docs/CLAIMS_VS_EVIDENCE_AUDIT.md`
- `docs/FINAL_LIMITATIONS_AND_NEXT_WORK.md`
- `docs/FINAL_RELEASE_SUMMARY.md`
- `docs/SUPERVISOR_SUMMARY.md`
- `docs/OPERATING_ENVELOPE_ANALYSIS.md`
- `docs/GEOMETRY_TOLERANCE_VALIDATION_RESULTS.md`
- `docs/CROSS_SCENARIO_ROW_LEVEL_DATASET.md`
- `docs/CROSS_SCENARIO_V2_14_EVALUATION.md`
- `docs/metrics/comprehensive_validation_metrics.json`

Supporting local surfaces inspected:

- `diagnostics/`
- `src/diagnostics/multi_scenario_row_level_dataset/`
- `src/thesis_bringup/config/geometry_tolerance_scenarios.yaml`
- `src/thesis_bringup/launch/`
- `src/thesis_bringup/scripts/`
- `src/kuka_task_control/kuka_task_control/admittance_insertion_node.py`
- `src/perception_pipeline/perception_pipeline/sac_baseline_scaffold.py`
- `src/perception_pipeline/perception_pipeline/sac_scenario_randomization.py`
- `src/perception_pipeline/perception_pipeline/sac_cluster_package/`
- `diagnostics/sac_scenario_randomization_contract.json`

## Executive Assessment

The local project has reached a strong simulation-only proposal checkpoint. The best-supported contribution is not universal peg-in-hole generalization. The best-supported contribution is a safety-gated, ROS 2/Gazebo peg-in-hole framework with a measured geometry/tolerance operating envelope and honest fail-closed behavior when tracking noise is comparable to clearance.

The strongest next research direction is a simulation-framework and data-quality sprint, not immediate SAC/meta-RL training. The current evidence says:

- 1.0 mm clearance family is robust in simulation: 72/80 Stage C successes, 90%.
- 0.5 mm and 0.25 mm clearance families fail closed: 0/60 Stage C successes.
- Tracking noise floor is approximately 0.5 mm, so clearance must exceed about 2x tracking noise for reliable insertion under the current stack.
- Cross-scenario v2_14 neural generalization is weak: 38.9% mixed-scenario accuracy and 20.9% held-out scenario mean accuracy.
- Geometry-only feasibility classification is useful and conservative: 96.4% accuracy and 0.0% false-safe rate in the row-level evaluation.
- SAC/meta-RL is scaffolded and cluster-ready only; no trained policy exists.
- Hardware and sim-to-real protocols exist, but no hardware validation has been executed.

## Current Strengths

1. Reproducible ROS 2/Gazebo integration.
   The project has a working KUKA LBR iisy 6 R1300 cell, `gz_ros2_control`, task launch files, controller configuration, observation logging, scenario scripts, and evidence-preserving diagnostics.

2. Safety-first deterministic task controller.
   The deterministic task sequence, SEARCH, INSERT handoff gates, recenter recovery, side-load recovery, force thresholds, and fail-closed behaviors are documented and backed by trial artifacts.

3. Operating-envelope evidence.
   Stage C has 140 trials across 7 scenarios. The result is interpretable: 1.0 mm clearance works reliably; smaller clearances fail closed because the noise floor is too large.

4. Honest negative ML result.
   The v2_14 classifier performs well on baseline data but does not generalize across scenarios. This is documented, not hidden.

5. Conservative feasibility signal.
   Geometry-only feasibility is practical because clearance and offset explain the dominant failure boundary better than the current empty visual/depth features.

6. Proposal traceability.
   The mapping documents connect local artifacts to proposal phases, current limits, and future work.

7. Cluster-ready learning scaffold.
   SAC environment contracts, scenario randomization, rewards, evaluation scripts, and SLURM package exist. The project correctly labels this as scaffold-only.

## Current Weaknesses

1. Visual/depth features are not research-grade.
   The row-level dataset states that RGB/depth features are empty in Gazebo. This directly limits cross-scenario neural learning and makes the 68-dimensional context less meaningful than its name implies.

2. Force/contact evidence is split and partly unreliable.
   Task-side wrench-derived contact exists, but Gazebo contact-topic positives are repeatedly zero and the F/T bridge has startup crash issues. Contact-rich insertion claims should therefore remain conservative.

3. Current simulator is slow for RL.
   Gazebo is useful for integration validation but poorly suited for millions of RL steps when each trial involves launch, bridge, and ROS 2 orchestration overhead.

4. Tight-clearance performance is not solved.
   The failure at <=0.5 mm clearance is structural under the current stack. It should be framed as a measured safety limit, not as a controller bug that can be tuned away.

5. Neural generalization is not demonstrated.
   The baseline-only v2_14 performance is strong, but cross-scenario results are poor. The classifier should not be presented as a universal adaptive policy.

6. Domain randomization is mostly planned/scaffolded.
   There are ranges and scenario contracts, but no validated automated randomization engine for contact, perception, dynamics, and sensor noise.

7. Hardware relevance remains unvalidated.
   Sim-to-real and hardware protocols are useful, but all results remain simulation-only.

## Evidence Quality

| Evidence area | Quality | Notes |
|---|---:|---|
| Deterministic baseline trials | High | Multiple automated runs, outcome JSONs, summaries, 53/60 grand total success documented. |
| Stage C geometry/tolerance matrix | High | 140 trials with 20 per scenario; enough to support operating-envelope claims. |
| Tight-clearance failure | High | 0/60 at <=0.5 mm is clear and repeated. |
| Cross-scenario row-level dataset | Medium | Dataset exists, but only 14 trials, sparse SEARCH rows, empty RGB/depth, mixed trial quality. |
| v2_14 baseline classifier | High inside baseline distribution | Offline, shadow-mode, advisory, and unit-test evidence exists. |
| v2_14 cross-scenario classifier | High as negative result | Results are poor and documented; do not use as positive generalization evidence. |
| Geometry-only feasibility classifier | Medium-high | Strong row-level result; still needs broader scenario and randomized validation. |
| Force/contact topic fidelity | Medium-low | Task wrench signals exist, but contact topics and F/T bridge issues weaken contact-fidelity claims. |
| SAC/meta-RL | Scaffold-only | Environment and scripts exist; no training evidence. |
| Hardware/sim-to-real | Protocol-only | No execution evidence. |

## Proposal Alignment

Aligned or exceeded:

- Simulation cell, launch orchestration, task control, safety gates, logging, reproducibility.
- Multi-scenario geometry/tolerance matrix.
- Baseline deterministic controller and safety-gated advisory role.
- Context-vector extraction and classifier ablations, including a negative encoder result.
- Initial SAC/meta-RL scaffolding and cluster package.

Partially aligned:

- Domain randomization: described and scaffolded, not executed as a validated automated mechanism.
- Virtual-force/admittance safety: task-side force-aware behavior exists, but proposal-level admittance/force-control claims must stay qualified because Gazebo uses position control and contact/F/T bridging has gaps.
- Multi-geometry generalization: cylindrical size and circular hole variation are covered; non-circular geometry and material/friction tolerance are not.

Not yet aligned:

- Trained SAC policy.
- Trained context-based meta-RL policy.
- Full comparison across deterministic, advisory, SAC, and meta-RL methods.
- Hardware KUKA validation.
- Sim-to-real transfer.

## Simulation Readiness

Ready now:

- Run short Gazebo validation trials.
- Reproduce deterministic baseline checks.
- Analyze existing diagnostics.
- Generate scenario-level and row-level datasets.
- Run offline classifier and feasibility analysis.
- Package learning contracts for later cluster use.

Not ready without another simulation sprint:

- High-quality visual/depth learning from Gazebo D405.
- Validated contact-topic based contact labels.
- Efficient RL training environment.
- Automated domain randomization across geometry, friction, sensor noise, and contact parameters.
- Cross-simulator reproducibility.

## Overclaiming Risks

Do not claim:

- Universal tolerance generalization.
- Successful sub-millimeter insertion.
- A trained SAC or meta-RL result.
- Hardware readiness beyond protocol planning.
- Contact-rich physical fidelity from Gazebo alone.
- Valid visual/depth learning from the current D405 features.
- v2_14 cross-scenario neural generalization.
- ML control authority during INSERT.

Safe claims:

- Simulation-only ROS 2/Gazebo framework is operational.
- Deterministic controller is validated in the measured 1.0 mm clearance envelope.
- The system fails closed at <=0.5 mm clearance under current tracking noise.
- v2_14 is useful only as a guarded advisory inside its validated distribution.
- Geometry-only feasibility is currently the most reliable envelope-aware classifier.
- SAC/meta-RL is scaffolded for future training, not trained.

## Missing Research Components

1. A validated visual/depth simulation pipeline with non-empty, calibrated, scenario-variable depth features.
2. Contact-fidelity validation with contact-topic, wrench-derived, and geometry-derived labels compared.
3. Automated domain randomization.
4. A fast RL simulator or hybrid simulator interface.
5. Learning environment wrapper with deterministic reset, stepped control, and scenario randomization independent of ROS launch overhead.
6. Sequence-aware perception model for RETREAT/DONE ambiguity and sparse SEARCH rows.
7. Publication-quality operating-envelope statistical analysis with confidence intervals and controlled ablations.
8. Non-circular peg/hole geometries and material/friction variations.
9. Hardware-executed calibration and transfer evidence.

## Already Locally Complete

- ROS 2/Gazebo robotic cell and launch system.
- KUKA-like URDF/SDF adaptation and controller configuration.
- Deterministic full-task state machine and safety gates.
- Stage C geometry/tolerance validation.
- Operating-envelope analysis.
- Multi-scenario row-level dataset.
- Cross-scenario v2_14 evaluation.
- Geometry-only feasibility result.
- v2_13/v2_14/v2_15 perception and ablation evidence.
- SAC scenario-randomization contract and cluster package.
- Hardware and sim-to-real planning protocols.
- Supervisor and release summaries.

## Needs GPU Later

- SAC training at meaningful scale.
- Meta-RL training, especially PEARL-style context inference or recurrent/meta-adaptation.
- Isaac Sim synthetic data generation if used at volume.
- Large visual/depth dataset generation and vision-model training.
- Multi-seed learning comparisons with confidence intervals.

## Needs Hardware Later

- Real KUKA tracking-noise measurement.
- Real F/T calibration and contact-threshold tuning.
- Real D405 calibration and minimum-distance/occlusion characterization.
- Physical 1.0 mm clearance validation before any tighter clearance attempt.
- Sim-to-real domain randomization evaluation.
- Safety acceptance tests under real robot constraints.

## Audit Conclusion

The next phase should not begin with SAC/meta-RL training or hardware execution. The scientifically strongest move is a simulation-only framework sprint that fixes the observation/contact bottlenecks and prepares a hybrid simulator contract. The most defensible research line is:

1. Publishable operating-envelope simulation study.
2. Envelope-aware feasibility and fail-closed safety.
3. Simulation-framework upgrade for fast learning and realistic visual/depth data.
4. Later SAC/meta-RL training only after the simulator and data contract are credible.
