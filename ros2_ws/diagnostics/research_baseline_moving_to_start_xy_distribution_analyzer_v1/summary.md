# research_baseline_moving_to_start_xy_distribution_analyzer_v1

Status: analyzer enhancement validated on canonical post-command-capture data.

This milestone extends `moving_to_start_tracking_analyzer` so command-attributed
`MOVING_TO_START` tracking evidence reports the whole XY-error distribution, not
only one final FK sample. The analyzer now records:

- command-window min/mean/p95/max XY error;
- strict 2 mm sample count and fraction;
- final one-second XY min/mean/max.

Validation:

```bash
python3 -m py_compile src/thesis_bringup/thesis_bringup/moving_to_start_tracking_analyzer.py
colcon build --symlink-install --packages-select thesis_bringup
ros2 run thesis_bringup moving_to_start_tracking_analyzer diagnostics/research_baseline_canonical_after_command_capture_v1
```

Canonical post-command-capture result:

- command-attributed result: `OK`
- command-window minimum XY error: `0.000115 m`
- command-window mean XY error: `0.144945 m`
- command-window p95 XY error: `0.383919 m`
- command-window max XY error: `0.415979 m`
- strict 2 mm samples: `138 / 15524`
- strict 2 mm sample fraction: `0.008889`
- final one-second XY min: `0.000575 m`
- final one-second XY mean: `0.007944 m`
- final one-second XY max: `0.018345 m`

Interpretation: the canonical command can pass through the strict XY band, but
it does not hold there. This explains why a single final or minimum sample can
look promising while the task state machine correctly rejects the phase.
