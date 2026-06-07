# Robot Datasheet Check: KUKA LBR iisy 6 R1300

## Target Specification (KUKA LBR iisy 6 R1300)

| Property | Datasheet Value | Current Implementation | Status |
|---|---|---|---|
| Rated payload | 6 kg | 6 kg (iisy6_R1300 macro) | ✅ Match |
| Max payload | 6.9 kg | N/A (simulation) | ✅ |
| Max reach | 1300 mm | 1300 mm (iisy6_R1300) | ✅ Match |
| Number of axes | 6 | 6 | ✅ Match |
| Pose repeatability | ±0.05 mm | N/A (simulation) | ✅ |
| Footprint | 275 × 275 mm | Same mechanical interface | ✅ Match |
| Mounting | Floor | Floor (pedestal model) | ✅ Match |
| Robot weight | ~46.3 kg | 46.3 kg (mass_scale=0.772) | ✅ Match |
| Controller | KR C5 micro | joint_trajectory_controller + Gazebo | ✅ Simulation match |
| Protection | IP54 | Not modeled | N/A (simulation) |

## Robot Model: lbr_iisy6_r1300

The upstream `kuka_lbr_iisy_support` package does not include a dedicated
`lbr_iisy6_r1300` macro. A project-local custom macro was created at
`kuka_lbr_iisy_support/urdf/lbr_iisy6_r1300_macro.xacro`, derived from the
iisy11 R1300 macro with the following adaptations:

| Property | iisy11 R1300 (source) | iisy6 R1300 (custom) |
|---|---|---|
| Mass | ~60 kg | ~46.3 kg (`mass_scale=0.772`) |
| Payload | 11 kg | 6 kg |
| Link inertias | Full values | Scaled by 0.772 |
| Torque limits | Full values | Scaled by 0.545 |
| Kinematics | 1300mm reach, 6-axis | Identical (same chain) |
| Joint limits | Per datasheet | Identical |
| Meshes | Real STL files | Symlinked (same external geometry) |

## Chain Verification: flange → tool → peg

The URDF uses `research_parallel_gripper.xacro` with `has_peg="true"` to define
the fixed-joint chain:

```
base_link → link_1 → link_2 → link_3 → link_4 → link_5 → link_6 → flange → gripper_body → left_finger, right_finger, peg (grasped)
```

- `peg` link is a child of `gripper_body` via a **fixed joint** (no fake-object
  scripts or separate spawning).
- `peg_tip` frame marks the bottom-center of the grasped peg for Cartesian
  targeting.
- The peg moves only when the robot joints move.

## Workspace Reachability

Robot base position: `x=0.80, y=-0.75, z=0.735, yaw=1.5708` (pedestal-mounted).

Hole center (world frame): `[0.520, -0.200, 0.810]`

Distance from base_link to hole_center:
- dx = 0.520 − 0.800 = −0.280 m
- dy = −0.200 − (−0.750) = 0.550 m
- dz = 0.810 − 0.735 = 0.075 m
- Euclidean distance = √(0.280² + 0.550² + 0.075²) ≈ **0.62 m**

This is well within the 1300 mm reach → **reachable**.

## Files

| File | Purpose |
|---|---|
| `kuka_lbr_iisy_support/urdf/lbr_iisy6_r1300_macro.xacro` | Custom iisy6 R1300 macro (scaled masses/torques from iisy11) |
| `kuka_lbr_iisy_support/urdf/lbr_iisy6_r1300.urdf.xacro` | Upstream-style entry point for iisy6 |
| `kuka_lbr_iisy_support/config/lbr_iisy6_r1300_joint_limits.yaml` | Joint limits config for iisy6 |
| `kuka_lbr_iisy_support/meshes/lbr_iisy6_r1300/` | Symlinks to iisy11 meshes (identical external geometry) |
| `peg_in_hole_description/urdf/lbr_iisy6_r1300_research_gripper.urdf.xacro` | Research baseline URDF using iisy6 + gripper + peg + camera |
