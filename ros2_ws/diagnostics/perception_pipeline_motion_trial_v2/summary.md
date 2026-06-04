# perception_pipeline_motion_trial_v2

First trial in which the arm actually moved.

## Root cause of "arm doesn't move" in previous trials
The velocity state diagnostic in commit `6347194` added `velocity` to
`state_interfaces` in `research_baseline_ros2_control.yaml`. The intent
was to give the JTC access to real Gazebo joint velocity for the
D-term. But `gz_ros2_control/GazeboSimSystem` does not export the
velocity state interface by default, so the JTC could not activate:

```
[controller_manager]: Unable to activate controller
'joint_trajectory_controller' since the state interface
'joint_1/velocity' is not available.
```

When the JTC fails to activate, the admittance_insertion_node still
publishes /task_phase=MOVING_TO_START and a JointTrajectory, but
nothing drives the arm. Joints stay at the initial pose forever and
the trial times out at 120 s with ABORT. The 4-lever search tuning
matrix (D-term, position gain, controller type, velocity source) was
all run against this non-functional JTC. The "1mm window remains
unblocked across 4 levers" finding is therefore a measurement of a
non-existent controller, not the search tuning problem we thought.

## Fix
- Reverted `state_interfaces` in
  `src/thesis_bringup/config/research_baseline_ros2_control.yaml`
  from `[position, velocity]` back to `[position]` only. The
  velocity_state variant (`research_baseline_velocity_state.yaml`)
  is left in place for explicit `inject_velocity_state:=true`
  trials, but it is no longer the default. The URDF injection
  script (`inject_velocity_state_urdf.py`) was also rechecked and
  works as intended when used with the velocity_state config (the
  previous failure was the default config carrying `velocity`
  without the URDF injection, not the injection itself).
- No other code change. The admittance_insertion_node, safety
  monitor, perception logger, and v2_12 extractor all work
  unchanged.

## Trial config
- `source /opt/ros/jazzy/setup.bash && source /tmp/pos_controllers_setup.bash && source install/setup.bash`
- `ros2 launch thesis_bringup research_baseline.launch.py
   enable_perception_logging:=true
   perception_log_dir:=diagnostics/perception_pipeline_motion_trial_v2
   use_gui:=false`
- 300 s wall clock.

## Results (after the JTC fix)
- 2701 rows in 300 s (~9 Hz effective, the logger is 20 Hz but the
  admittance node only emits /task_phase updates on state changes).
- joint_2: -0.8 -> -1.03 (motion confirmed)
- joint_3: 1.2 -> 2.37 (motion confirmed; near soft limit 2.7)
- joint_5: 0.8 -> -1.35 (motion confirmed)
- ft_z: 0.0 throughout (no contact yet; SEARCH not reached)
- task_phase: 53 UNKNOWN, 2392 MOVING_TO_START, 256 ABORT
- safety_status: 44 WARNING (waiting_for_task_phase), 1628 OK
  (joint_states_valid), 1029 WARNING (joint_limit_warning when
  joint_3 climbed above ~2.0).

## Why it ABORTed at 300 s
The admittance's MOVE_TO_START convergence check requires
`xy_err <= APPROACH_START_XY_TOLERANCE = 0.002 m` for
`STABILIZE_TICKS = 5` consecutive ticks within
`MOVING_TO_START_TIMEOUT_S = 120 s`. The arm reached the target in
joint space but the JTC's cartesian tracking at 2 mm XY does not
stabilize fast enough. This is the same XY-tolerance ceiling we
tested at SEARCH time (the "1mm window" lever matrix), now showing
up at MOVE_TO_START time too. With a working JTC, the binding
constraint is the JTC's tracking accuracy, not the search tuning
or the D-term.

## Next step
Re-test the SEARCH tolerance gates with the working JTC. The 4-lever
matrix needs to be re-run because the previous results were against
a broken controller. The motion_trial_v2 data is the first usable
JTC-driven trial; use it to verify the 1mm SEARCH window becomes
reachable with the correct JTC defaults before proceeding to v2_13
encoder training.
