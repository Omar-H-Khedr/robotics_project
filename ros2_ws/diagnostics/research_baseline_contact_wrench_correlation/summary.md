# Research Baseline Contact-Wrench Correlation

Date: 2026-06-02

Status: implemented and validated as passive diagnostics; not insertion success.

## Purpose

Correlate raw F/T spikes with Gazebo contact sensor evidence. The previous raw
wrench milestone proved that the controller can see large wrench spikes during
`MOVING_TO_START`, but it did not determine whether the canonical
`/gazebo/contacts/{peg,hole,target}` topics also report contact.

This milestone adds passive `contact_state_observer` logging for:

- `/gazebo/contacts/peg`
- `/gazebo/contacts/hole`
- `/gazebo/contacts/target`

The observer groups messages by `/insertion_state` and writes compact CSV and
Markdown summaries. It does not publish commands or alter controller behavior.

## Validation Command

```bash
cd /home/omar/code/robotics_project/ros2_ws
mkdir -p /tmp/ros2_logs /tmp/gz_home diagnostics/research_baseline_contact_wrench_correlation
export ROS_LOG_DIR=/tmp/ros2_logs
export HOME=/tmp/gz_home
source install/setup.bash
timeout 150s ros2 launch thesis_bringup research_baseline.launch.py use_gui:=false tracking_log_dir:=diagnostics/research_baseline_contact_wrench_correlation
```

## Runtime Result

The trial aborted safely during `MOVING_TO_START`:

- `Outcome: ABORTED`
- `Reason: Hard force abort: raw wrench exceeded 1000.0N in state MOVING_TO_START; |Fz|=1396.8N, |F|=2624.1N.`
- `Depth: 0.0000 m`
- `max_abs_fz_N: 1396.75`
- `max_force_norm_N: 2624.11`

## Contact Evidence

`contact_state_summary.md` reported:

- samples: `0`
- positive_contact_samples: `0`
- max_contact_force_n: `0.000000`

No messages were observed on the three bridged canonical contact topics during
this run. This is not proof that every possible collision pair was contact-free;
it means the canonical peg/hole/target contact sensors did not provide positive
contact evidence for the raw wrench spike.

## Wrench Evidence

`wrench_state_summary.md` reported:

- samples: `7228`
- max_abs_fz_n: `1396.750557`
- max_force_norm_n: `2624.114841`
- `MOVING_TO_START` max abs Fz: `1396.750557 N`
- `MOVING_TO_START` max force norm: `2624.114841 N`

## Decision

The passive contact observer is retained. The next blocker is now narrower:
large free-space F/T spikes occur without corresponding messages on the
canonical contact topics. The likely causes to investigate are FT sensor
semantics, inertial/dynamic loads from the free-space trajectory, uninstrumented
collision pairs, or Gazebo/controller physics. The hard-force abort remains
correct and should not be weakened.
