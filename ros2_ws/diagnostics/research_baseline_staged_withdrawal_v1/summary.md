# Research Baseline Staged Withdrawal V1

Date: 2026-06-03

## Purpose

Test whether splitting RETREAT into a vertical clearance lift followed by a
separate home trajectory reduces contact after a successful insertion.

## Validation Commands

```bash
python3 -m py_compile src/kuka_task_control/kuka_task_control/admittance_insertion_node.py
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select kuka_task_control thesis_bringup
source install/setup.bash
timeout 430s ros2 launch thesis_bringup research_baseline.launch.py use_gui:=false joint_damping_scale:=5.0 tracking_log_dir:=diagnostics/research_baseline_staged_withdrawal_v1
ros2 run thesis_bringup moving_to_start_tracking_analyzer diagnostics/research_baseline_staged_withdrawal_v1
ros2 run thesis_bringup endpoint_hold_dynamics_analyzer diagnostics/research_baseline_staged_withdrawal_v1
ros2 run thesis_bringup approach_tracking_analyzer diagnostics/research_baseline_staged_withdrawal_v1
ros2 run thesis_bringup insert_retreat_contact_analyzer diagnostics/research_baseline_staged_withdrawal_v1
```

## Runtime Outcome

- Final outcome: `SUCCESS`.
- Reason: `Full cycle completed. Insertion depth 0.021m, contact 59.5N during INSERT.`
- Final insertion depth from console: `0.0208 m`.
- Max insert contact from task F/T estimator: `59.5 N`.
- Max raw `|Fz|`: `153.0 N`.
- Max raw force norm: `285.8 N`.
- Phase sequence: MOVING_TO_START OK, APPROACH OK, SEARCH OK, INSERT OK, RETREAT OK.

## Staged Retreat Behavior

The modified source published:

- a vertical clearance trajectory at command stamp `80.422 s`, with `11` points and `20.856819 s` duration;
- a second home trajectory at command stamp `99.702 s`, with `10` points and `25.000 s` duration.

Console evidence:

- `Retreating vertically to clearance. duration=20.9s, lift_waypoints=10, target_z=0.8850m`
- `RETREAT clearance reached: peg_z=0.8779m, target_z=0.8850m.`
- `Retreating from clearance to SAFE_HOME. duration=25.0s, home_waypoints=10`

## Analyzer Result

`insert_retreat_contact_analyzer` reported:

- final physical depth `0.020862 m`;
- maximum physical depth `0.024773 m`;
- INSERT observed window `23.115 s`;
- RETREAT contact rows `307`;
- RETREAT max contact force `486.746287 N`;
- top RETREAT pair: `grasped_peg_collision_2 <-> target_plate::plate_left_collision`;
- RETREAT max raw force norm `285.766425 N`.

## Decision

Rejected. The staged withdrawal preserved insertion success but worsened
RETREAT contact compared with `research_baseline_insert_sim_time_completion_v4`:

- v4 RETREAT contact rows: `41`;
- staged v1 RETREAT contact rows: `307`;
- v4 RETREAT max contact force: `249.593329 N`;
- staged v1 RETREAT max contact force: `486.746287 N`;
- v4 max raw force norm: `211.14 N`;
- staged v1 max raw force norm: `285.8 N`.

The staged source change was removed. Future withdrawal work should not simply
split lift and home trajectories; it should diagnose fixture/hole contact
geometry during vertical extraction and reduce lateral or angular peg load
while the peg is still engaged.
