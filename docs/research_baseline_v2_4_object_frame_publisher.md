# Research Baseline v2.4 Object Frame Publisher

v2.3 made Cartesian peg/hole insertion diagnostic-only. It measured the current
robot/tool Cartesian pose when TF was available, loaded `hole_center_world` and
`pre_insertion_pose_world` from YAML, and reported distances without commanding
robot motion.

v2.4 adds explicit object-frame publication for coordinate-based insertion. The
new `peg_hole_frame_publisher` node reads
`kuka_task_control/config/peg_hole_cartesian_targets.yaml` and publishes static
TF frames from `world` to:

- `hole_center`
- `pre_insertion_pose`
- `insertion_touch_pose`
- `insertion_hold_pose`
- `final_insertion_pose`
- `insertion_axis_marker`

It also publishes `/peg_hole_frame_status` as JSON with
`status=object_frames_published`, the configured `world_frame`, the list of
published frames, and the published target count.

`cartesian_insertion_diagnostics` now resolves target poses from TF first and
uses YAML only as a fallback. Its JSON diagnostics include `frame_source` values
of `tf` or `yaml_fallback` for each target pose, while preserving
`current_tool_pose_world`, `distance_tool_to_hole`,
`distance_tool_to_pre_insertion`, and `status=diagnostic_only_no_motion`.

This remains a no-motion baseline. Future insertion motion must consume these
named object frames through IK or a Cartesian planner. It must not use manually
guessed joint-space values as the source of insertion behavior.
