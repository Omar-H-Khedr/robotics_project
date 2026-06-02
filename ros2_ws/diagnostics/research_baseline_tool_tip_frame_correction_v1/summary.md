# research_baseline_tool_tip_frame_correction_v1

Date: 2026-06-02

## Purpose

Fix and validate the modeled research gripper peg-tip frame after contact-pair attribution showed `link_5_collision` hitting the target plate during `MOVING_TO_START`.

## Change Under Test

The gripper model previously placed `peg_tip` at the near-palm end of the 110 mm peg. That forced the controller to drive the wrist close to the target plate when it tried to place the peg tip above the hole.

The model now places the peg and fingers on the negative local tool-Z side:

- gripper fingers: `z=-0.055 m`
- grasped peg center: `z=-0.075 m`
- `gripper_tcp` and `peg_tip`: `z=-0.130 m`

The local `RobotKinematics` peg-tip transform was updated to match the URDF: `link6-flange` plus `ft_sensor_joint` plus `palm->peg_tip` gives a net tool offset of `-0.125 m` after the F/T mount.

## Static Checks

```bash
python3 -m py_compile \
  src/kuka_task_control/kuka_task_control/robot_kinematics.py \
  src/kuka_task_control/kuka_task_control/clearance_path_analyzer.py

source /opt/ros/jazzy/setup.bash
source install/setup.bash
xacro src/peg_in_hole_description/urdf/lbr_iisy6_r1300_research_gripper.urdf.xacro

colcon build --symlink-install \
  --packages-select peg_in_hole_description kuka_task_control thesis_bringup
```

Result: passed.

## Offline Clearance Result

Command:

```bash
ros2 run kuka_task_control clearance_path_analyzer \
  diagnostics/research_baseline_tool_tip_frame_correction_v1 \
  --stride 10
```

Evidence:

- `clearance_path_analysis.md`
- `clearance_path_analysis.json`

Result:

- planned `SAFE_HOME -> AXIS_ALIGN_POSE`: `0/201` sampled `link_5` target-plate intersections;
- runtime feedback samples: `0/1338` sampled `link_5` target-plate intersections;
- closest sampled planned clearance: `0.129226 m`;
- closest sampled runtime-feedback clearance: `0.117041 m`.

## Runtime Validation

Command:

```bash
mkdir -p /tmp/ros2_logs /tmp/gz_home diagnostics/research_baseline_tool_tip_frame_correction_v1
export ROS_LOG_DIR=/tmp/ros2_logs
export HOME=/tmp/gz_home
source install/setup.bash
timeout 240s ros2 launch thesis_bringup research_baseline.launch.py \
  use_gui:=false \
  tracking_log_dir:=diagnostics/research_baseline_tool_tip_frame_correction_v1
```

Result:

- outcome: `ABORTED`
- reason: `MOVING_TO_START timeout/failure (120.0s)`
- final phase Cartesian error: `0.015238 m`
- final phase joint error: `0.022126 rad`
- final logged XY error: about `0.014 m`
- insertion depth: `0.0000 m`
- max raw `|Fz|`: `172.83 N`
- max raw force norm: `270.82 N`
- contact observer samples: `0`

## Conclusion

The tool-tip correction removes the reproduced `link_5` target-plate clearance collision in both offline planned-path analysis and the new runtime feedback samples. Runtime force also stayed below the global hard-force abort threshold.

This is not insertion success. The run still failed honestly before descent because `MOVING_TO_START` did not satisfy the strict 2 mm no-contact XY stability gate for five consecutive ticks. The next milestone should focus on final above-hole XY stabilization/settling with the corrected tool frame, preserving the no-contact gate and hard-force abort.
