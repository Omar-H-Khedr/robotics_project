# perception_pipeline_labeled_trial_v2

Phase 5/6 v2_12 Option A trial: wire `/task_phase` and `/safety_status`
into the live cell, run a real Gazebo trial, and extract the labeled
context vectors. The wiring is real and verified, but the data the
existing controller stack produces is not yet useful for v2_13.

## Wiring (committed separately)
- `admittance_insertion_node.py` now publishes `/task_phase` as a
  String message, mirroring `/insertion_state` (already published).
- `safety_monitor` was already in the workspace but not started by
  `research_baseline.launch.py`. Added the `safety_monitor` Node to
  the research baseline so `/safety_status` is published live.
- The perception logger now sees real labels.

## Trial config
- `ros2 launch thesis_bringup research_baseline.launch.py
   enable_perception_logging:=true
   perception_log_dir:=diagnostics/perception_pipeline_labeled_trial_v2
   use_gui:=false`
- 180 s wall clock (timeout 180 s).

## Results
- 1595 rows x 31 columns in the multimodal CSV, 4.0 MB.
- task_phase distribution: 51 UNKNOWN, 1544 MOVING_TO_START.
- safety_status distribution: 44 WARNING (waiting_for_task_phase),
  1551 OK (joint_states_valid).
- Extracted context_log.parquet: 1595 rows x 74 dims, 31 KB.
- context_dim 74, no inf, no NaN.

## Critical data-quality issue (not blocking the wiring, blocking v2_13)
The arm did not actually move during the trial:
- joint_2_pos_rad: -0.8000 (initial), std 0
- joint_3_pos_rad: 1.2000 (initial), std 2.2e-16
- joint_5_pos_rad: 0.8000 (initial), std 0
- ft_z_n: 0.0 throughout

The admittance_insertion_node sets the state to MOVING_TO_START and
sends a JointTrajectory, but the JTC never moves the arm in the
research baseline under headless / current defaults. This is the
same root cause as the "1mm window remains unblocked across the four
JTC levers" finding (D-term, position gain, controller type, velocity
source). The phase publisher is honest; the underlying controller is
not delivering the planned motion.

## What this means for v2_13
The 1544 rows of MOVE_TO_START context vectors are nearly identical
(no joint motion, no force). They cannot train a meaningful phase
classifier (one class, no variation). The wiring is real, the
extractor's JSON safety_status parsing works (saw values 1 and 2),
and the v2_12 parquet round-trips cleanly, but **v2_13 needs
trials that actually execute SEARCH / INSERT to produce
multi-phase, motion-rich training data**.

## Next options (require user input)
- A) Bypass the JTC/admittance stack: directly drive the arm with
  MoveIt or with a scripted JointTrajectory, log /task_phase via
  the same publisher, get a multi-phase trial.
- B) Synthesize phases via a publisher (no Gazebo motion), use the
  existing camera to record at known labels, train v2_13 on
  per-phase visual features only.
- C) Accept that v2_13 cannot proceed until the JTC/admittance
  lever is unblocked, pivot Phase 5/6 to focus on unblocking that
  lever (e.g., a different controller architecture) before adding
  learning.
