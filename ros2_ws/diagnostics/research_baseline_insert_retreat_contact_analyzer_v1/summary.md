# Research Baseline Insert / Retreat Contact Analyzer v1

Date: 2026-06-03

## Milestone

`research_baseline_insert_retreat_contact_analyzer_v1`

## Change

Added `thesis_bringup.insert_retreat_contact_analyzer`, an offline diagnostic that:

- selects the INSERT command by FK target near the canonical final insertion pose;
- prefers `trajectory_controller_state_samples.csv` when available;
- computes final and minimum peg-tip Z against `HOLE_TOP_Z=0.810 m`;
- reports missing descent to the commanded final insertion target;
- summarizes contact-topic collision pairs by state;
- summarizes wrench peaks by state.

The analyzer is passive. It does not publish commands or change task safety gates.

## Commands

```bash
cd /home/omar/code/robotics_project/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select thesis_bringup
source install/setup.bash
ros2 run thesis_bringup insert_retreat_contact_analyzer diagnostics/research_baseline_approach_z_precondition_gate_v1
```

## Result

Evidence written in:

- `diagnostics/research_baseline_approach_z_precondition_gate_v1/insert_retreat_contact_analysis.md`
- `diagnostics/research_baseline_approach_z_precondition_gate_v1/insert_retreat_contact_analysis.json`

Analyzer result:

- tracking source: `trajectory_controller_state_samples.csv`;
- selected INSERT command index: `2`;
- command target: `(0.520004, -0.200001, 0.790008)`;
- final feedback: `(0.518227, -0.201338, 0.813985)`;
- missing descent to target: `0.023977 m`;
- minimum feedback Z: `0.811899 m`;
- hole top Z: `0.810000 m`;
- max physical insertion depth: `0.000000 m`;
- final physical insertion depth: `0.000000 m`;
- contact-topic rows were only attributed to `RETREAT`;
- RETREAT max contact force: `1970.434828 N`;
- top RETREAT collision pair: `lbr_iisy6_r1300::ft_sensor_link::ft_sensor_link_fixed_joint_lump__grasped_peg_collision_2 <-> target_plate::plate_link::target_plate_collision`;
- second high-force pair includes `gripper_right_finger_collision_4 <-> target_plate_collision`.

## Interpretation

`HOLE_TOP_Z=0.810 m` is consistent with the world/model target plate top. The zero physical depth is therefore not explained by a stale depth constant in this evidence. The INSERT command target asks for a peg-tip Z near `0.790 m`, but controller feedback never goes below `0.811899 m`, leaving about `24 mm` of missed descent to the commanded final target and about `1.9 mm` above the hole top.

The next safety-critical implementation milestone should address insert/retreat motion behavior:

- prevent retreat from sweeping the peg or gripper laterally through the plate after a failed insert;
- keep or add an upward clearance segment before moving toward `SAFE_HOME`;
- continue to reject physical success unless insertion depth, contact evidence, and final outcome criteria are all met.
