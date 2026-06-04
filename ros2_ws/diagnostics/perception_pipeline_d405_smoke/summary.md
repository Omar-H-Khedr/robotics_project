# perception_pipeline_d405_smoke

Phase 5/6 v2_11 smoke test: confirm the multimodal observation logger
subscribes to the D405 RGB-D topics that already exist in
`peg_in_hole_world.sdf` and that are bridged to ROS by
`research_baseline_bridge.yaml`.

## Trial config
- `ros2 launch thesis_bringup research_baseline.launch.py enable_perception_logging:=true perception_log_dir:=diagnostics/perception_pipeline_d405_smoke use_gui:=false`
- 25 s wall clock (timeout 25 s; Gazebo, controllers, and the logger were
  up; trial ended at the move-to-start hold, not at task completion).
- Use_sim_time: true.
- Logger: `perception_pipeline/multimodal_observation_logger`, 20 Hz.

## Outputs
- `multimodal_observation_log.csv` (425 KB, 183 rows, 31 columns).
  - 182 / 182 data rows have a non-empty `rgb_b64_png` (column 23).
  - 182 / 182 data rows have a finite `depth_max_m` (column 26).
  - Joint velocities are NaN throughout (no JTC active in this short
    smoke run; expected).
  - `task_phase=UNKNOWN`, `safety_status=UNKNOWN` (those topics are not
    yet published by any active node; not blocking for v2_11 because the
    logger tolerates absent topics by writing the UNKNOWN sentinel).

## What this proves
- D405 RGB + depth topics publish in `peg_in_hole_world.sdf` and are
  bridged to ROS by `research_baseline_bridge.yaml`. No Gazebo change
  was required to enable logging.
- `multimodal_observation_logger` starts and writes real data with the
  same launch invocation that real Phase 5/6 trials will use.
- The `enable_perception_logging` and `perception_log_dir` launch
  arguments are wired into `research_baseline.launch.py` and accepted
  by the ros2 launch parser.

## Known limitations
- The D405 has no scene-relevant content during the move-to-start hold,
  so depth values are near the camera's near plane. Once SEARCH begins
  (pegs hovers ~10 mm above the hole), the depth values will reflect
  the workspace.
- `task_phase` and `safety_status` are not yet wired up. Phase 5/6 v2_12
  needs those to label rows. They are not blockers for v2_11 (logging
  works without them).
