# Research Baseline Joint Damping Scale 5.0 V1

Date: 2026-06-03

## Purpose

Test a targeted damping-authority diagnostic after the endpoint hold analyzer
showed multi-centimeter post-command oscillation in the canonical baseline.
This run uses `joint_damping_scale:=5.0` only. It does not change targets,
success criteria, no-contact gates, approach Z preconditions, contact
thresholds, hard-force aborts, or robot geometry.

## Commands

```bash
python3 -m py_compile src/thesis_bringup/thesis_bringup/approach_tracking_analyzer.py
colcon build --symlink-install --packages-select thesis_bringup
ros2 launch thesis_bringup research_baseline.launch.py use_gui:=false joint_damping_scale:=5.0 tracking_log_dir:=diagnostics/research_baseline_joint_damping_scale_5p0_v1
ros2 run thesis_bringup moving_to_start_tracking_analyzer diagnostics/research_baseline_joint_damping_scale_5p0_v1
ros2 run thesis_bringup endpoint_hold_dynamics_analyzer diagnostics/research_baseline_joint_damping_scale_5p0_v1
ros2 run thesis_bringup approach_tracking_analyzer diagnostics/research_baseline_joint_damping_scale_5p0_v1
```

The launch was wrapped with a 190 s shell timeout. The task node reached its
own final `ABORTED` outcome before the wrapper ended the still-running launch
processes.

## Runtime Result

- outcome: `ABORTED`
- final reason: `INSERT blocked: peg_z 0.8534m is above force-safe precondition 0.8450m. Approach did not reach the hole surface reliably.`
- insertion depth: `0.0000 m`
- MOVING_TO_START phase: `OK`, final `cart_err=0.0006 m`
- APPROACH phase: `OK` by current phase logger, final `cart_err=0.0234 m`
- INSERT reached: no, blocked by Z precondition
- max raw `|Fz|`: `129.8 N`
- max raw `|F|`: `209.2 N`
- contact-topic samples: `0`

## Tracking Evidence

- JTC controller-state records: `20832`
- trajectory commands captured: `3`
- global JTC p95 max joint-position error: `0.012689 rad`
- MOVING_TO_START analyzer source: `trajectory_controller_state_samples.csv`
- MOVING_TO_START final 1 s XY range: `0.000088-0.004176 m`
- endpoint hold duration before APPROACH: `1.107 s`
- endpoint hold X/Y/Z ranges: `0.008135 / 0.007048 / 0.009094 m`
- approach command index: `1`
- approach command target: `0.520000, -0.200000, 0.830000 m`
- approach final feedback: `0.521268, -0.202477, 0.849622 m`
- approach missing descent: `0.019622 m`
- approach p95 max joint-position error: `0.008192 rad`

## Conclusion

This is a substantial safety-preserving improvement over the canonical run:
the robot satisfies the strict no-contact start gate and reaches the approach
phase with lower raw wrench and lower joint tracking error. It is still not
insertion success. The remaining blocker is approach depth realization and the
force-safe Z precondition before INSERT.

Do not claim physical success from this run, and do not loosen the Z or
no-contact gates to force insertion.
