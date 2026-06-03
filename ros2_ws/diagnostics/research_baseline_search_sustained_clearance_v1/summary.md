# Sustained SEARCH Clearance Validation

Milestone: `research_baseline_search_sustained_clearance_v1`

Date: 2026-06-03

## Purpose

Prevent SEARCH from handing off to INSERT after a single transient
inside-clearance sample. SEARCH must now hold feedback inside the physical
`0.0010 m` radial clearance for `8` consecutive control ticks before INSERT may
start.

## Implementation

- Added `SEARCH_CONVERGENCE_TICKS = 8`.
- SEARCH now counts consecutive feedback ticks inside physical clearance.
- The counter resets on any outside-clearance feedback sample and after each
  SEARCH command.
- INSERT entry, handoff-settle, pre-contact clearance, side-load, and hard-force
  aborts remain active.

## Validation Commands

```bash
python3 -m py_compile src/kuka_task_control/kuka_task_control/admittance_insertion_node.py
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select kuka_task_control thesis_bringup
source install/setup.bash
timeout 430s ros2 launch thesis_bringup research_baseline.launch.py use_gui:=false joint_damping_scale:=5.0 tracking_log_dir:=diagnostics/research_baseline_search_sustained_clearance_v1
awk -F, 'NR>1 && $2=="SEARCH" {n++; xy=$14+0; if (xy<=0.001) {inside++; run++; if (run>maxrun) maxrun=run} else {run=0}; if (n==1 || xy<min) min=xy; if (xy>max) max=xy; sum+=xy} END {printf "search_samples=%d\ninside_1mm=%d\nlongest_consecutive_inside=%d\nmin_xy=%.6f\nmean_xy=%.6f\nmax_xy=%.6f\n", n, inside, maxrun, min, (n?sum/n:0), max}' diagnostics/research_baseline_search_sustained_clearance_v1/wrench_state_samples.csv
```

## Runtime Result

The run reached `DONE` and failed closed in SEARCH:

- final outcome: `ABORTED`;
- reason: `SEARCH timeout (45s). XY error 0.0028m remains above tolerance.`;
- insertion depth: `0.0000 m`;
- no INSERT phase was entered;
- no final descent-to-`z=0.790 m` command was published;
- max raw `|Fz|`: `129.1 N`;
- max raw force norm: `211.4 N`;
- contact-topic samples: `0`;
- phase sequence: `MOVING_TO_START OK`, `APPROACH OK`, `SEARCH FAIL`.

As with the preceding canonical launch validation, this launch path did not
leave a `trial_outcome.json` in the tracking directory before the validation
wrapper was stopped after `DONE`; the outcome above is transcribed from the
task node's final console report.

## SEARCH Stability Evidence

From `wrench_state_samples.csv`:

- SEARCH samples: `4500`;
- SEARCH samples inside `0.0010 m`: `308`;
- longest consecutive inside-clearance run: `2` samples;
- min SEARCH XY error: `0.000037 m`;
- mean SEARCH XY error: `0.003108 m`;
- max SEARCH XY error: `0.008754 m`.

The task log also showed transient values such as `xy_error=0.0009 m` with
`stable=2/8` and `xy_error=0.0002 m` with `stable=1/8`, but these did not
persist long enough for a safe INSERT handoff.

## Interpretation

This is a safety improvement, not insertion success. The previous SEARCH logic
could hand off after one favorable sample; the new logic prevents that and
keeps the robot from entering INSERT when no-contact alignment is not
sustained.

The next blocker is not the gate itself. The SEARCH motion/control behavior
cannot hold the peg tip inside the `0.0010 m` physical clearance near the hole
surface. The next implementation should improve bounded no-contact centering
stability, likely by adding a centered hold/settle strategy after each search
candidate or by moving sustained alignment higher above the workpiece before
the final approach.
