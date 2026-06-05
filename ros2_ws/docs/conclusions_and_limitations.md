# Conclusions and Limitations

This PhD work covers the simulation infrastructure, controller
tuning, and perception/learning pipeline for a KUKA LBR iisy 6
R1300 Gazebo peg-in-hole research baseline. The strongest
historical iisy6 evidence is a controller-driven simulated
insertion-depth event from `diagnostics/research_baseline_insert_sim_time_completion_v4`:
final outcome SUCCESS under older depth/contact criteria,
insertion depth 0.0191 m, task F/T insert-contact evidence 60.1
N, max raw |Fz|=133.33 N, no safety abort or invalid timeout.
That claim is now superseded by a stricter physical-clearance
gate: the trial reached depth 0.0177 m and insert contact 55.4
N, but correctly reported DEGRADED because final insertion XY
error 0.0030 m exceeds the 25 mm peg / 27 mm hole radial
clearance of 0.0010 m.

The current status of the project is therefore: no validated
physical insertion success under the latest 0.0010 m XY gate,
and a documented 1mm/2mm cartesian precision ceiling in the
working joint trajectory controller (JTC) that prevents
unblocking the gate through tuning alone.

## Main result

The main result of this PhD is not a successful peg-in-hole
insertion, but a precise diagnosis of the binding constraint
and a complete Phase 5/6 perception + context-learning pipeline
that can be re-used when the controller is improved.

The diagnostic process, in order:

  1. The 4-lever SEARCH/INSERT matrix (D-term, position gain,
     controller type, velocity state source) was tested against
     the 1mm/2mm gate. None of the levers unblocked it.
  2. The most important finding of the search: lever 4
     ("velocity state injection in default config") was a
     non-test. Between commits `6347194` and `e10960f`, the JTC
     silently failed to activate because the default
     ros2_control YAML declared `velocity` in `state_interfaces`
     while `GazeboSimSystem` does not export velocity state by
     default. The diagnostic matrix measured a non-existent
     controller. The fix is to either use the explicit
     `inject_velocity_state:=true` path with the matching
     `research_baseline_velocity_state.yaml`, or keep the
     default ros2_control YAML with `state_interfaces=[position]`
     only. The latter is the canonical default since `e10960f`.
  3. With the working JTC, the 1mm/2mm cartesian precision is
     unreachable because joint stability at the target pose is
     ~0.5 deg std per joint (0.007-0.011 rad) at the canonical
     gain 1000, and the cumulative cartesian error from 6 joints
     at this stability is ~2mm XY at the end-effector.

The 1mm/2mm precision ceiling is therefore a property of the JTC's
cartesian tracking, not a tuning lever. Improving it requires a
different controller architecture (e.g., an admittance controller
with explicit Cartesian feedback, or a model-predictive
controller with a calibrated kinematic model), which is out of
scope for this PhD work.

## Phase 5/6 perception pipeline

The Phase 5/6 work adds a complete offline perception +
context-learning pipeline:

  v2_11  multimodal observation logger
  v2_12  context vector extractor (74-dim)
  v2_13  self-supervised context encoder (74 -> 32)
  v2_14  context-conditioned action (32 -> 9 + 6)
  v2_15  context-action ablation (with-encoder vs raw 74-dim)

The v2_13 self-supervised encoder (74 -> 32 -> 74 MLP
autoencoder) reaches test_mse=0.0024 on the v3 single-phase
motion trial and test_mse=0.0061 on the synthetic multi-phase
trial. The v2_14 phase classifier on top of the v2_13 encoder
latent reaches 100% test accuracy on the synthetic multi-phase
dataset. The v2_15 ablation reports a null result (encoder
pre-training at parity with the raw-input baseline), which is
expected when the phase label is directly in the 74-dim context
vector.

## Synthetic multi-phase trial

To produce multi-phase labeled data for v2_14 / v2_15 training,
a `synthetic_phase_publisher` node was added to
`thesis_bringup`. It publishes /task_phase on a scripted
schedule (YAML or builtin 6-step default) using sim time.
Wired into `research_baseline.launch.py` as
`enable_synthetic_phases:=true`, it replaces the
admittance_insertion_node as the /task_phase source. The
arm does NOT execute any controller commands; the JTC is
still spawned and activated so /joint_states is real. The
recorded CSV (10 MB, 4305 rows, 7 distinct phases) is in
`diagnostics/perception_pipeline_synthetic_multiphase_v1/`.

The /task_phase labels are time-window proxies, not real
motor-actuated phases. This is a documented limitation, not
a workaround. The synthetic trial is intended for testing the
v2_14 / v2_15 pipeline, not for training a controller.

## Limitations

  1. **No physical insertion success under the latest
     physical-clearance gate.** The 1mm/2mm cartesian precision
     ceiling is a property of the JTC's cartesian tracking, not
     a tuning lever.

  2. **Phase labels in the synthetic trial are time-window
     proxies.** The v2_14 / v2_15 baselines are valid only for
     the synthetic multi-phase dataset. Re-training on a
     real multi-phase trial (which is currently unreachable
     due to limitation 1) would be a future milestone.

  3. **V2_13 encoder is small and shallow.** The 74 -> 32 -> 32
     -> 32 -> 74 MLP has only ~4K parameters. A larger encoder
     might learn a more phase-discriminative representation,
     but it would also require a larger multi-phase dataset to
     avoid overfitting.

  4. **V2_14 regressor head is trivial.** The per-phase target
     joint position is the initial pose (because the synthetic
     trial has a frozen arm). The regressor head learns to
     output the initial pose regardless of context, which is
     not a useful "context-conditioned action" in the
     control-theoretic sense. The head is a placeholder for
     the integration point with a real multi-phase trial.

  5. **V2_15 ablation is a null result.** The encoder
     pre-training is at parity with the raw-input baseline on
     the synthetic dataset. A real ablation requires a
     multi-phase dataset where the phase is IMPLICIT in the
     sensor data, not declared by the publisher.

  6. **No live ROS integration.** The v2_13 encoder and v2_14
     head are intended for integration with the live controller
     in a follow-up sprint. The integration pattern is
     documented in `docs/thesis_chapter_5_phase_5_6.md`
     section 5.9 but not implemented here. (SUPERSEDED —
     see limitation 9 below; live inference IS implemented
     in this PhD work and validated at 62.6% live accuracy,
     but it is a passive inference node, not a closed-loop
     controller.)

  7. **The D405 RGB-D camera is the only perception source.**
     No tactile sensor, no second camera, no force-torque
     gradient analysis. The 74-dim context vector is enough
     to demonstrate the pipeline, but a real peg-in-hole
     controller would benefit from a tactile sensor at the
     peg tip.

  8. **The Python perception pipeline is offline-only.** The
     v2_13/v2_14/v2_15 scripts run as one-shot subprocesses,
     not as live nodes. Real-time inference (e.g., publishing
     a JointTrajectory correction to the JTC at 20 Hz) would
     require rewriting the inference as a live ROS2 node.
     (SUPERSEDED — see limitation 9; the live
     v2_14_inference_node is a 20 Hz ROS2 node that loads
     the encoder + head checkpoints and publishes
     /v2_14/predicted_phase, /v2_14/target_joint_pose, and
     /v2_14/latent in real time.)

  9. **Live integration is a passive inference node, not a
     closed-loop controller.** The live_v2_14_inference_node
     (`src/perception_pipeline/perception_pipeline/
     live_v2_14_inference_node.py`) runs at 20 Hz in the
     research baseline, but it only publishes its outputs
     on observation topics. It does NOT publish
     JointTrajectory corrections to the JTC. The reasons are
     documented in the chapter: (a) the JTC's
     follow_joint_trajectory action server only accepts one
     action client at a time, and the
     admittance_insertion_node already uses it; (b) blending
     the v2_14 predicted target with the admittance target
     is a non-trivial control problem (priority, saturation,
     anti-windup). The live inference is the perception
     half of a closed-loop controller; the control half is
     out of scope for this PhD work.

  10. **Live input distribution shift from training.** The
      v2_13 encoder and v2_14 head were trained on the
      synthetic multi-phase dataset. The live trial produces
      the same phase distribution but a slightly different
      raw sensor distribution (real Gazebo timing jitter,
      real depth-image inf pixels, joint_state_broadcaster
      velocity NaN). The live node sanitizes these (NaN/inf
      -> 0.0), but the live accuracy (62.6%) is below the
      offline test accuracy (100%) because the encoder
      bottleneck cannot perfectly reconstruct the live
      distribution. A larger encoder and a real multi-phase
      dataset would close this gap.

## What the next milestone should be

The next milestone is to either:

  (a) Improve the controller architecture to break the 1mm/2mm
      cartesian precision ceiling. This requires a different
      controller (admittance with explicit Cartesian feedback,
      or MPC with a calibrated kinematic model). Out of scope
      for this PhD work but the natural continuation.

  (b) Integrate the v2_13/v2_14/v2_15 pipeline as a live ROS2
      node that publishes JointTrajectory corrections to the
      JTC. (PARTIALLY COMPLETED: the
      live_v2_14_inference_node IS a 20 Hz live ROS2 node
      that loads the encoder + head checkpoints and publishes
      the predicted phase + target joint pose + latent in
      real time. It is validated at 62.6% live accuracy on
      the synthetic schedule. The remaining work is the
      closed-loop control: a multiplexer that shares the
      JTC's follow_joint_trajectory action server between
      the admittance_insertion_node and the v2_14 policy,
      and a blending law with priority, saturation, and
      anti-windup.)

  (c) Generate a real multi-phase dataset (e.g., by lowering
      the JTC gains and accepting the lower precision, or by
      using a different controller as in (a)) and re-train the
      v2_13/v2_14/v2_15 pipeline on the real data. The
      synthetic trial would then become a baseline for
      comparison.

The decision between (a), (b), and (c) is a project-management
question, not a technical one. The technical foundation for
all three is in place: a working JTC, a working perception
pipeline, a working context encoder, a working context-
conditioned action head, and a working ablation. The next
milestone is to apply them to a real multi-phase trial.

## Status

The PhD work is complete with respect to the Phase 5/6
deliverables: v2_11, v2_12, v2_13, v2_14, v2_15 are all
implemented, tested on canonical Gazebo trial data, and
documented. The 1mm/2mm cartesian precision ceiling is
diagnosed. The synthetic multi-phase trial is generated
and labeled. The v2_15 ablation is run. The live v2_14
inference node is implemented and validated at 62.6%
live accuracy. The next milestone is either (a) improve
the controller architecture to break the precision ceiling,
(b) close the closed-loop control gap (multiplexer + blending
law) for the live v2_14 policy, or (c) generate a real
multi-phase dataset and re-train the pipeline.
