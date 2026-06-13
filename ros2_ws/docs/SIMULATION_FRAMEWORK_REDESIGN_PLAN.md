# Simulation Framework Redesign Plan

Date: 2026-06-13
Basis: local ROS 2/Gazebo project evidence plus general simulator knowledge.

## Executive Recommendation

Keep ROS 2/Gazebo as the integration, safety, controller, and evidence-validation harness. Do not migrate the whole project now. Add a simulator-neutral experiment contract, then pilot a fast simulator for learning only after the next simulation-only data/contact sprint.

Recommended architecture:

1. ROS 2/Gazebo remains the authoritative system-level simulator.
2. MuJoCo is the best first candidate for fast SAC/meta-RL training if a compact peg-in-hole environment is needed.
3. Isaac Sim is the best candidate for future visual/depth synthetic data and photorealistic domain randomization if GPU resources are available.
4. Drake is best reserved for contact-model studies and hydroelastic contact analysis, not the main training loop.
5. PyBullet should be treated as a low-cost baseline/prototyping option, not the final contact-rich research simulator.

Most realistic next upgrade:

- Do not replace Gazebo.
- Create a documented hybrid architecture and run the next sprint on Gazebo observation/contact fixes.
- In the following sprint, add a minimal MuJoCo pilot only if the Gazebo data contract is clean.

## Simulator Comparison

| Framework | Contact-rich peg-in-hole suitability | Force/contact fidelity | RL/meta-RL speed | Domain randomization | Visual/depth generation | ROS 2 integration | KUKA-like modeling | Reproducibility | Implementation cost | Risk | Expected benefit |
|---|---|---|---|---|---|---|---|---|---|---|---|
| Current ROS 2/Gazebo setup | Good for system integration and controller validation; proven locally at 1.0 mm clearance | Medium; task wrench works, contact-topic positives and F/T bridge are weak | Low; launch/bridge overhead and real-time dynamics make million-step RL expensive | Partial; scenario YAML/scripts exist, full automated randomization not validated | Weak currently; D405 features are empty/limited | Excellent; already implemented | Good; local KUKA cell exists | High locally; diagnostics and docs are mature | Low for incremental work | Medium; bridge/contact/camera issues persist | Preserves evidence continuity and thesis traceability |
| Gazebo Sim upgrade path | Good for ROS 2-native robotics integration | Medium; better if contact and sensor plugins are fixed and validated | Low-medium; still not ideal for large RL | Medium; SDF/world generation can randomize parameters | Medium if camera pipeline is corrected | Excellent | Good | High if pinned worlds/configs are used | Medium | Medium; may not solve RL speed or visual realism | Best near-term continuity with current assets |
| Isaac Sim | Good for robot simulation, perception, synthetic data, and GPU workflows | Medium-high with PhysX, but contact-rich tight tolerance still needs calibration | Medium-high through Isaac Lab/vectorized workflows, GPU dependent | High; Replicator and USD workflows support synthetic data randomization | High; strongest option for RGB/depth/segmentation data | Good but more integration work than Gazebo | Good if URDF/USD conversion is controlled | Medium; GPU/software version sensitivity | High | High; steep migration and hardware requirements | Best future perception and synthetic-data upgrade |
| MuJoCo | Good for compact contact-rich manipulation and model-based/RL research | Medium-high for fast articulated dynamics with contact, but requires careful contact parameter tuning | High; best first option for SAC/meta-RL speed | High through Python environment wrappers | Low-medium; not the best photorealistic visual source | Medium-low; bridge to ROS 2 is custom | Medium; URDF/MJCF conversion needed | High if environment is pure Python and seeded | Medium | Medium; model mismatch with Gazebo/KUKA stack | Best fast-learning pilot |
| Drake | Strong for contact modeling studies, especially hydroelastic contact | High for analytical contact studies when modeled correctly | Low-medium; not the fastest deep RL platform | Medium; programmatic systems support | Low-medium; not primarily a photorealistic data engine | Medium; custom ROS 2 interfaces | Medium-high; robot models possible but setup intensive | High | High | Medium-high; learning stack friction | Best contact-mechanism analysis and validation |
| PyBullet | Adequate for cheap prototyping | Medium-low for tight tolerance contact; tuning sensitive | High | High | Low-medium | Medium-low; custom bridge needed | Medium; URDF import is easy | Medium | Low | High scientific risk for tight-contact claims | Useful only as a quick baseline or CI smoke simulator |
| Hybrid ROS 2 + fast RL simulator | Best overall architecture if boundaries are clean | Depends on selected fast simulator; Gazebo remains validation check | High if MuJoCo/Isaac Lab handles training | High; scenario contract can be shared | Medium-high if Isaac is added for visuals | Medium; requires explicit adapters | Medium-high with shared geometry spec | High if contract and seeds are versioned | Medium-high | Medium; interface drift between simulators | Best path to train later without losing local validation |

## What Should Stay in ROS 2/Gazebo

Keep these in Gazebo:

- The validated KUKA LBR iisy 6 R1300 workcell.
- `research_baseline.launch.py` and scenario launch orchestration.
- Deterministic controller validation and safety-gate regression tests.
- Stage C operating-envelope reproduction.
- Trial outcome JSON, row-level logging, and evidence packaging.
- ROS 2 topic contracts for joint states, task phase, safety state, wrench, image/depth, and context vectors.
- Any result claimed in the thesis as system-level validation.

Reason:

The local evidence already depends on Gazebo and ROS 2. Replacing the simulator now would break comparability and delay the research. Gazebo should remain the "truth harness" for this codebase, even if it is not the long-term RL training engine.

## What Should Move to a Faster Simulator If Needed

Move later, behind a contract:

- SAC and meta-RL rollouts.
- Dense scenario randomization over clearance, offset, friction, damping, and sensor noise.
- Fast ablations over reward design and fail-closed policies.
- Low-level policy pretraining.
- Curriculum learning over geometry families.

Best first fast simulator:

- MuJoCo for a minimal state/contact/RL environment.

Best future visual simulator:

- Isaac Sim for RGB/depth/segmentation/synthetic-data generation, not as the first migration of the whole control stack.

## How Learning, Controller, Data, and Evaluation Should Connect

Use a shared experiment contract:

```text
scenario_config
  -> simulator adapter
  -> observation schema
  -> action schema
  -> reward and termination schema
  -> trial result schema
  -> evaluator
```

Required shared fields:

- Scenario ID, peg diameter, hole diameter, radial clearance, initial XY offset.
- Contact parameters: friction, stiffness/compliance, damping, restitution if available.
- Sensor parameters: joint noise, wrench noise/bias, RGB/depth validity, camera pose.
- Observation vector version and feature validity mask.
- Action representation: deterministic high-level action, joint velocity, Cartesian delta, or policy residual.
- Safety status and fail-closed reason.
- Success metric: insertion depth, final XY, force limits, time, phase completion.

Data flow:

1. Gazebo produces authoritative validation datasets.
2. MuJoCo/Isaac produce training datasets or policies.
3. All policies are evaluated first in the fast simulator, then replayed in Gazebo through the same scenario contract.
4. Only Gazebo-validated policies become thesis claims before hardware.
5. Hardware remains future work until simulation-only evidence is stable.

## Best Simulation Framework for the Next Phase

Best next phase framework:

- ROS 2/Gazebo as the main framework, plus a planned hybrid interface.

Do not add Isaac Sim or MuJoCo as the main simulator in the immediate sprint. The current bottlenecks are data quality and contact observability, not the absence of a second simulator.

Add MuJoCo after:

- Visual/depth validity checks exist.
- Contact/wrench labels are reconciled.
- Scenario schema is stable.
- A minimal gym-style environment contract can be tested without long training.

Add Isaac Sim after:

- There is a clear visual/depth dataset requirement.
- GPU resources are available.
- The project can afford USD/asset conversion and version-control overhead.

## Contact and Tolerance Modeling Upgrade

Current issue:

- The Stage C result shows the dominant tolerance boundary, but contact simulation is not sufficiently instrumented. Contact-topic positives are repeatedly zero, while task-side wrench-derived contact is used for outcomes.

Better model:

- Treat radial clearance/noise ratio as the primary envelope variable.
- Sample clearance continuously around 0.25, 0.5, 0.75, 1.0, 1.25, and 1.5 mm.
- Randomize friction, damping, contact stiffness/compliance, peg/hole chamfer, and initial XY offset.
- Add measurement noise to joint position and wrench topics.
- Record separate contact labels: geometric overlap/contact, simulated contact topic, wrench-derived threshold, and task-state contact.
- Do not claim improved tight-clearance success unless the measured tracking noise also improves.

## Visual/Depth Feature Upgrade

Current issue:

- Multi-scenario row-level data states that Gazebo D405 RGB/depth features are empty or not meaningful.

Better model:

- Add observation validity checks that fail the dataset if RGB sum/depth dimensions are invalid.
- Make peg, hole, fixture, and workplane visible to the camera from useful viewpoints.
- Generate depth ROI features around the hole and peg tip, not whole-frame summary only.
- Add segmentation or known-geometry labels for synthetic data.
- If Gazebo cannot provide stable D405 data quickly, use Isaac Sim for synthetic RGB/depth/segmentation data while keeping Gazebo for controller validation.

## How the Operating-Envelope Result Becomes Publishable

Frame the paper as:

- "Safety-gated peg-in-hole simulation framework with measured clearance/noise operating envelope."

Minimum publication package:

- Factorial scenario matrix with confidence intervals.
- Explicit clearance/noise ratio model.
- Failure-mode taxonomy: XY exceeded, handoff timeout, side-load, timeout.
- Ablations on tracking noise, clearance gate, contact parameters, and visual/depth validity.
- Positive result: robust 1.0 mm envelope.
- Negative result: fail-closed <=0.5 mm envelope.
- Honest learning result: v2_14 does not generalize across scenarios; geometry-only feasibility is conservative and useful.
- Artifact package: scenario configs, outcome schema, row-level metadata, analysis scripts.

## What Should Not Be Implemented Yet

- Full simulator migration.
- Long SAC/meta-RL training.
- Hardware execution.
- A new architecture that bypasses the existing evidence harness.
- Claims of visual/depth learning from empty features.
- Claims of contact-fidelity improvement without contact-topic/wrench validation.
- A tight-clearance controller that relaxes safety gates merely to increase success rate.

## Final Architecture Choice

Use a staged hybrid architecture:

1. **Now:** ROS 2/Gazebo only, fix observation/contact/data validity.
2. **Next:** Add a simulator-neutral scenario and result contract.
3. **Then:** Add MuJoCo for fast SAC/meta-RL rollouts.
4. **Later:** Add Isaac Sim for visual/depth synthetic data and domain randomization.
5. **Always:** Evaluate claimed policies back in Gazebo before any hardware plan.
