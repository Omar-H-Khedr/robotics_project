# Research Baseline Raw Wrench Abort

Date: 2026-06-02

Status: implemented and validated as a safety/diagnostic improvement; not
insertion success.

## Purpose

The previous `MOVING_TO_START` runs reported only the controller's sampled
positive Fz peak, while the passive wrench observer showed sub-control-period
raw wrench spikes in free-space motion. This milestone adds:

- passive `wrench_state_observer` logging for `/ft_sensor_wrench` by
  `/insertion_state` and peg pose;
- callback-level raw wrench peak tracking in `admittance_insertion_node`;
- hard-force abort latching on raw `|Fz|` or force norm in active task states,
  including `MOVING_TO_START`;
- outcome metrics for `max_abs_fz_N` and `max_force_norm_N`, while preserving
  legacy `max_fz_N` for the repeat validator.

## Validation Command

```bash
cd /home/omar/code/robotics_project/ros2_ws
mkdir -p /tmp/ros2_logs /tmp/gz_home diagnostics/research_baseline_raw_wrench_abort
export ROS_LOG_DIR=/tmp/ros2_logs
export HOME=/tmp/gz_home
source install/setup.bash
timeout 150s ros2 launch thesis_bringup research_baseline.launch.py use_gui:=false tracking_log_dir:=diagnostics/research_baseline_raw_wrench_abort
```

## Runtime Result

The trial aborted safely during `MOVING_TO_START`:

- `Outcome: ABORTED`
- `Reason: Hard force abort: raw wrench exceeded 1000.0N in state MOVING_TO_START; |Fz|=1943.3N, |F|=2936.5N.`
- `Depth: 0.0000 m`
- `Max |Fz|: 1943.3 N`
- `Max |F|: 2936.5 N`
- `Baseline: 64.1 N`
- `Contact: 104.6 N`

The final outcome JSON recorded:

- `max_fz_N: 1943.29`
- `max_abs_fz_N: 1943.29`
- `max_force_norm_N: 2936.48`
- `insertion_depth_m: 0.0`

## Observer Evidence

`wrench_state_summary.md` reported:

- samples: `7455`
- max_abs_fz_n: `1943.293077`
- max_force_norm_n: `2936.479541`
- `MOVING_TO_START` max abs Fz: `1943.293077 N`
- `MOVING_TO_START` max force norm: `2936.479541 N`
- `MOVING_TO_START` min XY: `0.000065 m`

This confirms that the spike occurred before descent/contact insertion and was
not a successful insertion event.

## Decision

The safety/diagnostic code is retained. The baseline must now treat high
free-space raw wrench spikes as a first-class blocker before further descent,
search, insertion, or learning milestones.

The next technical problem is to determine whether these spikes come from
hidden contact, FT sensor semantics, inertial dynamics from the free-space move,
or simulation/controller physics. The safety gate should not be weakened.
