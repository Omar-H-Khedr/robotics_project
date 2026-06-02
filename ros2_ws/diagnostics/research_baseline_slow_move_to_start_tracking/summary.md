# Slow Move-To-Start Timing Experiment

Date: 2026-06-02

## Milestone

`research_baseline_slow_move_to_start_rejected`

## Purpose

Test whether a slower, denser no-contact `MOVING_TO_START` trajectory improves
stable above-hole convergence without changing the strict 2 mm descent gate.

## Temporary Change Tested

The experiment temporarily changed `MOVING_TO_START` timing from the retained
baseline:

- baseline duration: `max(15.0, min(40.0, dist * 30.0))`;
- tested duration: 30-70 s, using `dist * 70.0`;
- baseline waypoint spacing: about 0.08 rad;
- tested waypoint spacing: about 0.04 rad.

This change was reverted after validation because it did not improve the gate.

## Commands

```bash
cd /home/omar/code/robotics_project/ros2_ws
python3 -m py_compile src/kuka_task_control/kuka_task_control/admittance_insertion_node.py
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select kuka_task_control thesis_bringup
mkdir -p /tmp/ros2_logs /tmp/gz_home diagnostics/research_baseline_slow_move_to_start_tracking
export ROS_LOG_DIR=/tmp/ros2_logs
export HOME=/tmp/gz_home
source install/setup.bash
timeout 150s ros2 launch thesis_bringup research_baseline.launch.py use_gui:=false tracking_log_dir:=diagnostics/research_baseline_slow_move_to_start_tracking
```

## Runtime Result

The temporary slower trajectory was active:

- `MOVING_TO_START: trajectory published. duration=45.6s, waypoints=17, dist=0.6517`

The task remained a bounded safety failure:

- `Outcome: ABORTED`;
- `Reason: MOVING_TO_START timeout/failure (90.0s)`;
- `cart_err=0.012 m`;
- `xy_err=0.011 m`;
- `joint_err=0.015 rad`;
- `stable=0/5`;
- `Depth: 0.0000 m`;
- `Max Fz: 169.4 N`;
- no descent, contact search, or insertion was attempted.

## Tracking Evidence

The compact generated tracking summary reported:

- observed trajectory commands: 1;
- direct JTC state samples: 0;
- command-vs-joint-state samples: 6,201;
- max absolute position error: 0.044994 rad;
- mean max absolute position error: 0.015939 rad;
- p95 max absolute position error: 0.026097 rad;
- final max absolute position error: 0.014105 rad.

The observer did not capture the initial move command in this run and captured
the retreat command, so this tracking evidence is only partial. The task logs
are still sufficient to reject the timing change because the above-hole gate
did not improve and the robot reached the target neighborhood later.

## Decision

Rejected and reverted. Slowing the whole no-contact trajectory delayed arrival
near the above-hole pose and did not reduce final XY error or satisfy the
`STABILIZE_TICKS` gate. The retained baseline is unchanged.

The next control milestone should focus on final hold/stabilization at the
above-hole target or controller/physics parameters, not on a globally slower
move-to-start trajectory.
