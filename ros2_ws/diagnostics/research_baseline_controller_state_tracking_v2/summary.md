# Research Baseline Controller-State Tracking V2

Date: 2026-06-03

## Purpose

Validate the enhanced passive trajectory observer against the canonical
headless research baseline. This milestone adds direct
`/joint_trajectory_controller/controller_state` CSV recording and makes the
MOVING_TO_START analyzer prefer controller-state reference/feedback/error when
available.

No task targets, controller gains, safety gates, contact thresholds, success
criteria, or robot geometry were changed.

## Commands

```bash
python3 -m py_compile src/thesis_bringup/thesis_bringup/trajectory_tracking_observer.py src/thesis_bringup/thesis_bringup/moving_to_start_tracking_analyzer.py
colcon build --symlink-install --packages-select thesis_bringup
ros2 launch thesis_bringup research_baseline.launch.py use_gui:=false tracking_log_dir:=diagnostics/research_baseline_controller_state_tracking_v2
ros2 run thesis_bringup moving_to_start_tracking_analyzer diagnostics/research_baseline_controller_state_tracking_v2
```

The launch was wrapped with a 190 s shell timeout; the task node reached its
own final `ABORTED` outcome before the wrapper ended the still-running launch
processes.

## Runtime Result

- outcome: `ABORTED`
- final reason: `MOVING_TO_START timeout/failure (120.0s). cart_err=0.018m, xy_err=0.013m, joint_err=0.017rad, stable=0/5, tolerance=0.050m`
- insertion depth: `0.0000 m`
- descent/search/insert reached: no
- max raw `|Fz|`: `171.7 N`
- max raw `|F|`: `272.8 N`
- contact-topic samples: `0`

## Tracking Evidence

- observed trajectory commands: `2`
- JTC state samples/records: `23878 / 23878`
- analyzer tracking source: `trajectory_controller_state_samples.csv`
- axis-align command duration: `40.000 s`
- post-command hold before retreat command: `23.719 s`
- JTC p95 max joint-position error during axis-align window: `0.023935 rad`
- worst joint by p95 error: `joint_3` at `0.021702 rad`
- final command-attributed XY error: `0.009709 m`
- strict XY threshold: `0.002 m`
- strict XY samples: `143 / 15930`
- strict XY sample fraction: `0.008977`
- final 1 s XY range: `0.000589 m` to `0.023077 m`

## Conclusion

The instrumentation is valid and now records the controller's own reference,
feedback, and error by joint name. The canonical baseline still fails honestly
before descent because the peg tip only crosses the 2 mm no-contact XY gate
transiently and does not satisfy the required five consecutive 10 Hz stable
ticks. This is not physical insertion success.
