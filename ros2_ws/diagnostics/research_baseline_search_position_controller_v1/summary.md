# Search Position Controller

Milestone: `research_baseline_search_position_controller_v1`, `research_baseline_search_position_controller_v2`

Status: `validated_failed_closed`

## Purpose

Isolate the effect of switching the underlying ros2_control controller
type from `joint_trajectory_controller/JointTrajectoryController` (JTC)
to `position_controllers/JointGroupPositionController` (joint-group
position controller). The position controller takes
`std_msgs/Float64MultiArray` (one position per joint) instead of
`trajectory_msgs/JointTrajectory` and does no internal trajectory
interpolation. A new `trajectory_position_bridge` node was added that
subscribes to the JTC-style trajectory topic, stores the active
multi-point trajectory, and at the controller update rate (250 Hz)
linearly interpolates the current reference and republishes it as a
`Float64MultiArray`. This is a diagnostic bridge: it preserves the
JTC's linear-interpolation semantics so the comparison between JTC
and JointGroupPositionController isolates the controller-type change.

The position controller's plugin is loaded from
`/tmp/ros_install/opt/ros/jazzy/lib/libposition_controllers.so` because
the system `ros-jazzy-position-controllers` package is not installed
and could not be installed without sudo. The extracted library is
sourced via `/tmp/pos_controllers_setup.bash` (sets
`AMENT_PREFIX_PATH` and `LD_LIBRARY_PATH`).

## Commands

```bash
source /tmp/pos_controllers_setup.bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
mkdir -p diagnostics/research_baseline_search_position_controller_v1
export ROS_LOG_DIR=/tmp/ros2_logs
export HOME=/tmp/gz_home
timeout 430s ros2 launch thesis_bringup research_baseline.launch.py \
  use_gui:=false \
  use_position_controller:=true \
  position_gain:=2000.0 \
  position_derivative_gain:=0.0 \
  joint_damping_scale:=5.0 \
  tracking_log_dir:=diagnostics/research_baseline_search_position_controller_v1
ros2 run thesis_bringup xy_stability_analyzer diagnostics/research_baseline_search_position_controller_v1
ros2 run thesis_bringup hold_window_reference_analyzer diagnostics/research_baseline_search_position_controller_v1
ros2 run thesis_bringup search_tracking_sensitivity_analyzer diagnostics/research_baseline_search_position_controller_v1
```

## Result

- Syntax check passed.
- Targeted `colcon build --symlink-install --packages-select
  kuka_task_control thesis_bringup` passed.
- Headless Gazebo launched, spawned `lbr_iisy6_r1300`, activated
  `joint_state_broadcaster`, then activated the new
  `position_controller` (loaded from the extracted
  `ros-jazzy-position-controllers` deb). The
  `trajectory_position_bridge` started and the admittance_insertion
  node proceeded through MOVING_TO_START, APPROACH, and SEARCH.

## Runtime Outcome (v1, v2)

- v1 outcome: `ABORTED`
- v1 reason: `SEARCH timeout (45s). XY error 0.0014m remains above physical clearance 0.0010m.`
- v1 max raw `|Fz|`: `127.4 N`
- v1 max raw force norm: `204.7 N`
- v1 contact-topic samples: `0`
- v1 observed trajectory commands: `10`
- v1 trajectory_tracking_samples: `48573` (over 194.302 s)
- v1 final max abs joint-position error (from /joint_states): `0.006608 rad`
- v1 SEARCH recenter attempts: `7`
- v1 final SEARCH XY: `0.0014 m` (reporter) / `0.001782 m` (xy_stability mean over window)

- v2 outcome: `ABORTED`
- v2 reason: `SEARCH timeout (45s). XY error 0.0038m remains above physical clearance 0.0010m.`
- v2 max raw `|Fz|`: see wrench_state_summary.md
- v2 SEARCH recenter attempts: see wrench_state_summary.md
- v2 final SEARCH XY: `0.0024 m` (reporter) / `0.003757 m` (xy_stability mean over window)

## Analyzer Evidence

`xy_stability_analysis.md`:

| Run | SEARCH mean XY m | SEARCH final XY m | SEARCH best 1 mm window | SEARCH best 2 mm window |
| --- | ---: | ---: | ---: | ---: |
| position_controller v1 | 0.002366 | 0.001782 | 2 | 12 |
| position_controller v2 | 0.002285 | 0.003757 | 4 | 10 |

`hold_window_reference_analysis.md`:

- The position controller does not publish
  `control_msgs/JointTrajectoryControllerState` on
  `/position_controller/controller_state` (the
  `ForwardCommandController` base does not include a state publisher by
  default), so the controller-state-samples CSV is empty for this path.
  The `trajectory_tracking_observer` therefore logs to
  `trajectory_tracking_samples.csv` (sourced from `/joint_states`) but
  the centered-hold diagnostic cannot compute a JTC joint error from
  the controller state.

`search_tracking_sensitivity_analysis.md`:

- `centered_hold_like_command_count: 0` for the same reason. The
  controller-state CSV is empty, so the passive linearized-Jacobian
  diagnostic has no feedback to compare against the JTC-style
  reference.

## Comparison vs JTC 2x2 Grid

| Run | Controller | SEARCH best 1 mm window | SEARCH best 2 mm window | SEARCH final XY m |
| --- | --- | ---: | ---: | ---: |
| streak-preservation | JTC (gain 2000, D 0) | 3 | n/a | 0.002779 |
| derivative_gain_v3 | JTC (gain 3000, D 10) | 3 | 10 | 0.002974 |
| derivative_gain_v4 | JTC (gain 2000, D 10) | 3 | 7 | 0.000342 |
| gain3000_v1 | JTC (gain 3000, D 0) | 2 | 4 | 0.002071 |
| position_controller v1 | position_controller (gain 2000) | 2 | 12 | 0.001782 |
| position_controller v2 | position_controller (gain 2000) | 4 | 10 | 0.003757 |

## Interpretation

Switching the underlying ros2_control controller type from
`joint_trajectory_controller/JointTrajectoryController` to
`position_controllers/JointGroupPositionController` (driven by a 250 Hz
linear-interpolation bridge) does not unblock the SEARCH 1 mm sustained
window. Across two runs, the position controller SEARCH 1 mm window
ranges from 2 to 4 ticks (JTC range was 2 to 3), the 2 mm window ranges
from 10 to 12 ticks (JTC range was 4 to 10), and the SEARCH final XY
ranges from `0.0018 m` to `0.0038 m` (JTC range was `0.0003 m` to
`0.0030 m`). The 2 mm window is the best seen in this line of work
(v1: 12 ticks, v2: 10 ticks), but the 1 mm sustained window binding
constraint remains unmet.

The position controller's state is not published on
`/position_controller/controller_state` by default (the
`ForwardCommandController` base does not include a state publisher), so
the controller-state-samples CSV is empty for this path and the
centered-hold JTC joint error diagnostic cannot run. The
`trajectory_tracking_samples.csv` (sourced from `/joint_states`) shows
the final max abs joint-position error is `0.006608 rad`, which is
better than the best JTC derivative-gain run (`0.008472 rad` in v4).

Both diagnostic runs are fail-closed and safe. The position controller
path is a viable parallel diagnostic to the JTC path, and the launch
arg `use_position_controller` selects between them. No insertion
success is claimed.

## Notes

- The system `ros-jazzy-position-controllers` package is not installed
  and could not be installed without sudo. The plugin libraries were
  extracted from the deb files to `/tmp/ros_install` and the
  `AMENT_PREFIX_PATH` and `LD_LIBRARY_PATH` are pointed at them via
  `/tmp/pos_controllers_setup.bash`. This is a diagnostic workaround,
  not a system install. To make this path permanent, install
  `ros-jazzy-position-controllers` and `ros-jazzy-forward-command-controller`
  at the system level.
- The 1 mm sustained window is still the binding constraint. Even with
  a different controller type and 250 Hz bridge interpolation, the
  best 1 mm SEARCH window observed is 4 ticks (well below the required
  8). The next iteration should look at the 1 mm binding constraint
  specifically (e.g., higher JSB state rate, decoupled state interface,
  or moving the hold-tracking window into the bridge's interpolation
  rate).
