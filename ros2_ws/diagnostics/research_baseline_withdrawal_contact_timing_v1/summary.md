# Withdrawal Contact Timing V1

Milestone: `research_baseline_withdrawal_contact_timing_v1`

Purpose: determine whether successful-insert RETREAT contact occurs during
vertical extraction, later lateral home motion, or from an already side-loaded
peg before changing the motion policy again.

## Commands

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run thesis_bringup withdrawal_contact_timing_analyzer diagnostics/research_baseline_insert_sim_time_completion_v4
ros2 run thesis_bringup withdrawal_contact_timing_analyzer diagnostics/research_baseline_staged_withdrawal_v1
```

## Evidence

- `diagnostics/research_baseline_insert_sim_time_completion_v4/withdrawal_contact_timing_analysis.md`
- `diagnostics/research_baseline_insert_sim_time_completion_v4/withdrawal_contact_timing_analysis.json`
- `diagnostics/research_baseline_staged_withdrawal_v1/withdrawal_contact_timing_analysis.md`
- `diagnostics/research_baseline_staged_withdrawal_v1/withdrawal_contact_timing_analysis.json`

## Result

The v4 success had all `41` positive contact samples during `RETREAT_1`, the
single post-insert command to `SAFE_HOME`. The first contact arrived at
`81.273 s`, about `0.554 s` after the RETREAT command receipt (`80.719 s`),
while the peg tip was still inserted `0.022622 m`. The highest force event was
`249.593329 N` at depth `0.003927 m` and XY error `0.005688 m`, attributed to
peg versus target plate right collision.

The rejected staged-withdrawal run had all `307` positive contact samples
during `RETREAT_1`, the vertical lift command, and no contact rows attributed
to the later `RETREAT_2` home command. The first contact arrived at `82.187 s`,
about `1.765 s` after the vertical RETREAT command receipt (`80.422 s`), while
the peg tip was still inserted. The highest force event was `486.746287 N` at
depth `0.019732 m` and XY error `0.005979 m`, attributed to peg versus target
plate left collision.

## Interpretation

The withdrawal blocker is not primarily late lateral home motion. Contact is
already generated during the first post-insert extraction command, with roughly
5-7 mm lateral peg/hole offset while the peg is still inside or near the hole.
The next implementation should reduce side-loaded extraction by improving
centering/contact relaxation before lift, limiting initial extraction speed or
distance, or correcting fixture/hole collision geometry. Another blind split of
RETREAT into lift and home stages is not justified by this evidence.

This diagnostic is offline only. It reads recorded command, contact, and
tracking CSVs and does not publish commands or change task safety gates.
