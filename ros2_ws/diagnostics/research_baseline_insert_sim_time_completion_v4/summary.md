# Research Baseline Insert Sim-Time Completion V4

Date: 2026-06-03

## Purpose

Validate that INSERT completion uses ROS/Gazebo simulation time and that
success classification requires contact evidence during INSERT, not contact
that may occur later during RETREAT.

## Code Changes Validated

- `admittance_insertion_node.py` records the ROS-clock time when the INSERT
  trajectory command is published and waits for `insert_duration + 3.0 s`
  before evaluating insertion depth.
- `research_baseline.launch.py` now passes `use_sim_time` to
  `admittance_insertion_node`, matching Gazebo, controllers, and observers.
- The task node now tracks `max_insert_contact_force_N` separately from global
  contact and requires INSERT-phase contact evidence for final `SUCCESS`.

## Validation Commands

```bash
python3 -m py_compile src/kuka_task_control/kuka_task_control/admittance_insertion_node.py src/thesis_bringup/launch/research_baseline.launch.py
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select kuka_task_control thesis_bringup
source install/setup.bash
timeout 380s ros2 launch thesis_bringup research_baseline.launch.py use_gui:=false joint_damping_scale:=5.0 tracking_log_dir:=diagnostics/research_baseline_insert_sim_time_completion_v4
ros2 run thesis_bringup moving_to_start_tracking_analyzer diagnostics/research_baseline_insert_sim_time_completion_v4
ros2 run thesis_bringup endpoint_hold_dynamics_analyzer diagnostics/research_baseline_insert_sim_time_completion_v4
ros2 run thesis_bringup approach_tracking_analyzer diagnostics/research_baseline_insert_sim_time_completion_v4
ros2 run thesis_bringup insert_retreat_contact_analyzer diagnostics/research_baseline_insert_sim_time_completion_v4
```

## Runtime Outcome

- Final outcome: `SUCCESS`.
- Reason: `Full cycle completed. Insertion depth 0.019m, contact 60.1N during INSERT.`
- Phase sequence: MOVING_TO_START OK, APPROACH OK, SEARCH OK, INSERT OK, RETREAT OK.
- Insertion depth from final JSON: `0.0191 m`.
- Max insert contact from task F/T estimator: `60.1 N`.
- Max global contact from task F/T estimator: `85.0 N`.
- Max raw `|Fz|`: `133.33 N`.
- Max raw force norm: `211.14 N`.
- Pre-insertion XY error: `0.0006 m`.

## Controller-State Timing Evidence

`trajectory_commands.csv` shows:

- INSERT command receipt: `57.608 s`.
- INSERT command duration: `20.000 s`.
- RETREAT command receipt: `80.719 s`.
- Observed INSERT command window: `23.111 s`.

This confirms the task no longer preempts the 20 s insert trajectory after about
10.6 s of controller-state time.

## Offline Analyzer Evidence

`insert_retreat_contact_analyzer` selected command index `2` and reported:

- target peg-tip pose: `(0.520000, -0.200000, 0.790000)`;
- final feedback pose: `(0.518615, -0.199574, 0.790557)`;
- final Cartesian error norm: `0.001552 m`;
- minimum feedback Z: `0.785650 m`;
- maximum physical depth: `0.024350 m`;
- final physical depth: `0.019443 m`;
- INSERT window samples: `5778`;
- p95 max joint position error during INSERT: `0.008321 rad`.

Contact-topic limitation:

- The passive Gazebo contact observer recorded positive contact rows only in
  `RETREAT` for this run, not in `INSERT`.
- Task-side F/T contact evidence during INSERT was recorded by
  `max_insert_contact_force_N=60.1`.
- RETREAT contact-topic rows reached `249.593329 N`, mainly
  `grasped_peg_collision_2 <-> target_plate::plate_right_collision`.

## Interpretation

This is the first current iisy6 baseline run in this sequence with a full
controller-driven cycle, meaningful physical insertion depth, insert-phase F/T
contact evidence, no safety abort, and no invalid timeout. It is a single-trial
success, not a robustness claim.

Remaining limitation: RETREAT still produces contact-topic evidence after
insertion. The clearance lift substantially improved prior failed-insert
retreat contact, but successful-insert retreat needs a follow-up reduction or
withdrawal strategy before claiming robust safe insertion.
