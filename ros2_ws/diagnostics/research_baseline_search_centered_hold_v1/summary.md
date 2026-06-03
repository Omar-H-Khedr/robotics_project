# Rejected SEARCH Centered Hold Diagnostic

Milestone: `research_baseline_search_centered_hold_v1`

Date: 2026-06-03

## Purpose

Test whether a no-contact centered hold at the current SEARCH height can
produce sustained `0.0010 m` physical-clearance alignment before spiral search
offsets begin.

## Tested Change

The task controller was temporarily changed to publish one centered hold command
at current peg Z on SEARCH entry:

- target XY: hole center;
- target Z: current peg Z;
- duration: `3.0 s`;
- sustained SEARCH gate preserved: `8` consecutive control ticks inside
  `0.0010 m`;
- INSERT handoff-settle, pre-contact clearance, side-load, and hard-force aborts
  preserved.

The source change was reverted after validation because it did not achieve
sustained alignment.

## Validation Commands

```bash
python3 -m py_compile src/kuka_task_control/kuka_task_control/admittance_insertion_node.py
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select kuka_task_control thesis_bringup
source install/setup.bash
timeout 430s ros2 launch thesis_bringup research_baseline.launch.py use_gui:=false joint_damping_scale:=5.0 tracking_log_dir:=diagnostics/research_baseline_search_centered_hold_v1
awk -F, 'NR>1 && $2=="SEARCH" {n++; xy=$14+0; if (xy<=0.001) {inside++; run++; if (run>maxrun) maxrun=run} else {run=0}; if (n==1 || xy<min) min=xy; if (xy>max) max=xy; sum+=xy} END {printf "search_samples=%d\ninside_1mm=%d\nlongest_consecutive_inside=%d\nmin_xy=%.6f\nmean_xy=%.6f\nmax_xy=%.6f\n", n, inside, maxrun, min, (n?sum/n:0), max}' diagnostics/research_baseline_search_centered_hold_v1/wrench_state_samples.csv
```

## Runtime Result

The run reached `DONE` and failed closed in SEARCH:

- final outcome: `ABORTED`;
- reason: `SEARCH timeout (45s). XY error 0.0048m remains above tolerance.`;
- insertion depth: `0.0000 m`;
- no INSERT phase was entered;
- no final descent-to-`z=0.790 m` command was published;
- max raw `|Fz|`: `130.8 N`;
- max raw force norm: `208.5 N`;
- contact-topic samples: `0`;
- phase sequence: `MOVING_TO_START OK`, `APPROACH OK`, `SEARCH FAIL`.

As with the preceding canonical launch validations, this launch path did not
leave a `trial_outcome.json` in the tracking directory before the validation
wrapper ended after `DONE`; the outcome above is transcribed from the task
node's final console report.

## SEARCH Stability Evidence

From `wrench_state_samples.csv`:

- SEARCH samples: `4500`;
- SEARCH samples inside `0.0010 m`: `365`;
- longest consecutive inside-clearance run: `4` observer samples;
- min SEARCH XY error: `0.000062 m`;
- mean SEARCH XY error: `0.003157 m`;
- max SEARCH XY error: `0.008670 m`.

The centered hold command was observed as command index `2` with duration
`3.000 s`, followed by spiral search commands and abort retreat.

## Decision

Rejected. The centered hold was safe, but it still failed to meet the sustained
SEARCH clearance gate and timed out with worse final SEARCH XY (`0.0048 m`) than
`research_baseline_search_sustained_clearance_v1` (`0.0028 m`).

The active source is reverted to the sustained SEARCH clearance gate without
the centered-hold attempt.
