# Research Baseline Retreat Clearance Lift v1

Date: 2026-06-03

## Milestone

`research_baseline_retreat_clearance_lift_v1`

## Change

`RETREAT` now starts with a vertical peg-tip clearance lift before moving toward `SAFE_HOME`.

Implementation details:

- builds axis-constrained vertical lift waypoints at the current peg XY;
- lifts to at least `RETREAT_CLEARANCE_Z = AXIS_ALIGN_POSE.z = 0.885 m`;
- then interpolates from the lifted posture to `SAFE_HOME`;
- falls back to the old joint-space retreat if clearance-lift IK fails;
- does not loosen INSERT success criteria, force gates, XY gates, or hard-force abort.

## Commands

```bash
cd /home/omar/code/robotics_project/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select kuka_task_control thesis_bringup
source install/setup.bash
ros2 launch thesis_bringup research_baseline.launch.py use_gui:=false joint_damping_scale:=5.0 tracking_log_dir:=diagnostics/research_baseline_retreat_clearance_lift_v1
ros2 run thesis_bringup moving_to_start_tracking_analyzer diagnostics/research_baseline_retreat_clearance_lift_v1
ros2 run thesis_bringup endpoint_hold_dynamics_analyzer diagnostics/research_baseline_retreat_clearance_lift_v1
ros2 run thesis_bringup approach_tracking_analyzer diagnostics/research_baseline_retreat_clearance_lift_v1
ros2 run thesis_bringup insert_retreat_contact_analyzer diagnostics/research_baseline_retreat_clearance_lift_v1
```

## Runtime Result

The run reached `DONE` before the outer launch wrapper timeout.

- outcome: `DEGRADED`;
- reason: `Phase(s) failed: INSERT`;
- depth: `0.0008 m`;
- max raw `|Fz|`: `128.1 N`;
- max raw force norm: `208.5 N`;
- max contact force from task log: `99.1 N`;
- final XY error: `0.0011 m`;
- phase results: MOVING_TO_START OK, APPROACH OK, SEARCH OK, INSERT FAIL, RETREAT OK.

This is not physical insertion success.

## Retreat Evidence

The task published the new retreat trajectory:

- `Retreating to SAFE_HOME via clearance lift. duration=25.0s`;
- lift waypoints: `8`;
- home waypoints: `10`;
- observed command: `18` points, `25.000 s`.

Contact evidence improved compared with `research_baseline_approach_z_precondition_gate_v1`:

| Run | RETREAT contact samples | RETREAT max contact force N | Dominant pair |
|---|---:|---:|---|
| approach_z_precondition_gate_v1 | 7776 | 1970.434828 | peg/right-finger vs target plate |
| retreat_clearance_lift_v1 | 4 | 36.335073 | peg vs plate rear collision |

Wrench evidence also improved:

- previous RETREAT max raw `|Fz|`: `594.283889 N`;
- new RETREAT max raw `|Fz|`: `128.114936 N`;
- new RETREAT max force norm: `200.880897 N`.

## Insert Evidence

Insert/retreat analyzer:

- INSERT command index: `2`;
- target peg-tip pose: `(0.520004, -0.200001, 0.790008)`;
- final feedback pose: `(0.518882, -0.198642, 0.815626)`;
- missing descent to target: `0.025618 m`;
- minimum feedback Z: `0.809164 m`;
- max physical depth: `0.000836 m`;
- final physical depth: `0.000000 m`.

The retreat safety issue is substantially reduced, but the insertion controller still does not reach meaningful depth and must not be counted as a physical success.

## Next Status

The next technical blocker is insertion-depth realization, not retreat collision. Future work should investigate why the 20 s single-point INSERT command is superseded by RETREAT before the controller reaches the final insertion target and why the peg remains above or barely below the plate top despite a target near `z=0.790 m`.
