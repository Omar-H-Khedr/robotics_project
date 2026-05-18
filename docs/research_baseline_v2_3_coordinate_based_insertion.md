# Research Baseline v2.3 Coordinate-Based Insertion Diagnostics

v2.3 stops random joint-space tuning for peg/hole insertion validation. The
insertion workflow now starts from object and robot frames such as `hole_center`,
`insertion_axis`, `tool0`, and `pre_insertion_pose`, rather than manually chosen
joint values.

The foundation is diagnostic-only:

- `peg_hole_insertion_validation_world.sdf` exposes fixed visual markers and
  SDF frame anchors for the hole center, insertion axis, pre-insertion pose, and
  final insertion pose.
- `peg_hole_cartesian_targets.yaml` records Cartesian targets in the `world`
  frame with positions, orientations, and the insertion direction.
- `cartesian_insertion_diagnostics` reads TF for `world -> base_link` and
  `base_link -> tool0` when available, compares the current tool pose against
  the configured Cartesian targets, and publishes JSON diagnostics on
  `/cartesian_insertion_diagnostics`.

No trajectory command is sent in v2.3. No aggressive insertion is attempted.
Joint trajectories should only be generated after the target frames and
distances are validated from TF and world geometry.

IK and MoveIt integration are intentionally deferred to the next step. That
future layer should consume these validated Cartesian targets, solve for robot
motion, and keep the existing low-force segmented contact validation separate
from coordinate-based peg/hole insertion development.
