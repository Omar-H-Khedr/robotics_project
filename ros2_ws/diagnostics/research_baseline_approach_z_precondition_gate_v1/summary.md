# Research Baseline Approach Z-Precondition Gate v1

Date: 2026-06-03

## Milestone

`research_baseline_approach_z_precondition_gate_v1`

## Change

`APPROACH` completion now requires the same force-safe peg-Z precondition used before `INSERT`:

- peg Z must be `<= 0.8450 m`;
- Cartesian error must still be below the existing `0.050 m` tolerance;
- the strict no-contact start and insert XY gates were not loosened;
- hard-force abort behavior was not loosened.

This prevents the controller from reporting `APPROACH complete` while the peg is still too high for the insertion precondition.

## Commands

```bash
cd /home/omar/code/robotics_project/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select kuka_task_control thesis_bringup
source install/setup.bash
ros2 launch thesis_bringup research_baseline.launch.py use_gui:=false joint_damping_scale:=5.0 tracking_log_dir:=diagnostics/research_baseline_approach_z_precondition_gate_v1
ros2 run thesis_bringup moving_to_start_tracking_analyzer diagnostics/research_baseline_approach_z_precondition_gate_v1
ros2 run thesis_bringup endpoint_hold_dynamics_analyzer diagnostics/research_baseline_approach_z_precondition_gate_v1
ros2 run thesis_bringup approach_tracking_analyzer diagnostics/research_baseline_approach_z_precondition_gate_v1
```

## Result

The validation was bounded by the outer launch timeout after `RETREAT` started, but it produced useful evidence:

- `MOVING_TO_START` completed at the `joint_damping_scale:=5.0` diagnostic setting.
- `APPROACH` did not complete at `z=0.8452 m` because the preserved `<= 0.8450 m` precondition was still false.
- `APPROACH` completed only after the peg reached `z=0.8417 m`.
- `SEARCH` ran because pre-insertion XY was `0.0027 m`, then converged to about `0.0010 m`.
- `INSERT` executed, but reported `physical_depth=0.0000 m`.
- Console-reported INSERT contact force peaked around `95.0 N`; raw wrench summary recorded INSERT max force norm `209.244996 N`.
- Contact-topic rows were not observed during `INSERT`; they began in `RETREAT`.
- `RETREAT` contact rows reached `max_contact_force_n=1970.434828` and wrench summary reached `max_abs_fz_n=594.283889`, so retreat collision is now a safety-critical blocker.

This is not insertion success. It is a state-machine honesty and safety-gating correction that exposes the next blocker: zero physical insertion depth plus high retreat contact/collision after the insert trajectory.

## Tracking Evidence

MOVING_TO_START analyzer:

- source: `trajectory_controller_state_samples.csv`;
- command target: `(0.520000, -0.200000, 0.885001)`;
- final feedback: `(0.520075, -0.200808, 0.884704)`;
- final XY error: `0.000812 m`;
- final Cartesian error: `0.000864 m`;
- final one-second XY range: `0.000173-0.004883 m`;
- strict XY samples: `149 / 10282`.

Endpoint hold analyzer:

- hold duration: `1.128 s`;
- XY mean: `0.002252 m`;
- XY p95: `0.004252 m`;
- XY max: `0.005295 m`;
- strict 10 Hz bins: `0`;
- largest feedback-range joint: `joint_6`, `0.019993 rad`.

Approach analyzer:

- source: `trajectory_controller_state_samples.csv`;
- command index: `1`;
- target: `(0.520000, -0.200000, 0.830000)`;
- final feedback: `(0.517594, -0.202990, 0.840723)`;
- missing descent: `0.010723 m`;
- p95 max joint error: `0.008323 rad`;
- worst p95 joint: `joint_6`.

## Limitation

Because the outer `timeout` ended the launch after `RETREAT` had started, no final trial JSON was recorded for this run. The runtime console and passive observers are still sufficient to validate the approach gate behavior and identify the next blocker, but this run must not be counted as a completed physical success trial.
