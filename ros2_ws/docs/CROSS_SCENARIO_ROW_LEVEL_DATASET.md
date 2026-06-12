# Cross-Scenario Row-Level Dataset

Date: 2026-06-12
Status: COLLECTED (minimum viable dataset)

## Overview

Multi-scenario row-level context vector dataset collected from Gazebo trials
with perception logging enabled. This dataset enables true cross-scenario
v2_14 evaluation for the first time.

## Dataset Statistics

| Metric | Value |
|--------|-------|
| Total rows | 17,651 |
| Context dim | 68 |
| Scenarios | 7 |
| Trials | 14 (some scenarios incomplete) |
| Phases observed | 8 (UNKNOWN, MOVING_TO_START, APPROACH, SEARCH, INSERT, RETREAT, DONE, ABORT) |
| Parquet size | 2.5MB |
| CSV size | 21MB |

## Scenarios

| Scenario | Trials | Context Vectors | Success Rate | Envelope |
|----------|--------|-----------------|--------------|----------|
| baseline_loose | 3 | 6,267 | 1/3 (33%) | robust |
| clearance_medium | 3 | 5,902 | 1/3 (33%) | marginal |
| clearance_tight | 1 | 1,005 | 0/1 (0%) | out_of_envelope |
| large_peg_large_hole | 2 | 2,113 | 0/2 (0%) | robust |
| misaligned_baseline | 2 | 634 | 0/2 (0%) | robust |
| small_peg_small_hole | 1 | 638 | 0/1 (0%) | robust |
| tight_plus_misaligned | 2 | 1,092 | 0/2 (0%) | out_of_envelope |

## Phase Distribution

| Phase | Rows | Percentage |
|-------|------|------------|
| MOVING_TO_START | 6,785 | 38.4% |
| INSERT | 6,866 | 38.9% |
| APPROACH | 2,415 | 13.7% |
| DONE | 885 | 5.0% |
| RETREAT | 481 | 2.7% |
| UNKNOWN | 177 | 1.0% |
| ABORT | 13 | 0.1% |
| SEARCH | 29 | 0.2% |

## Feasibility Distribution

| Label | Rows | Percentage |
|-------|------|------------|
| in_envelope | 9,652 | 54.7% |
| marginal | 5,902 | 33.4% |
| out_of_envelope | 2,097 | 11.9% |

## Context Vector Layout

```
[ 0:48]  rgb: 8x6 grayscale (empty in Gazebo — D405 limitation)
[48:54]  depth: (w, h, min_m, max_m, roi_min_m, roi_max_m)
[54:60]  joint_position: (j1..j6) in rad
[60:66]  joint_velocity: (j1..j6) in rad/s
[ 66 ]   phase_int: task phase integer encoding
[ 67 ]   safety_int: safety status integer encoding
```

## Collection Method

- Gazebo Harmonic with KUKA LBR iisy 6 R1300
- Perception logging enabled (multimodal_observation_logger)
- Context vector extraction (context_vector_extractor.py)
- Raw CSV cleanup after extraction (save disk space)
- 300s timeout per trial

## Known Limitations

1. **RGB/depth features are empty** — Gazebo D405 camera does not produce meaningful data
2. **F/T sensor features are zeros** — ft_sensor_bridge SIGSEGV at startup
3. **Only 3 trials per scenario** (minimum viable for cross-scenario evaluation)
4. **Some scenarios have incomplete trials** — extraction failures on empty CSVs
5. **Trial outcomes are mixed** — some timeout, some succeed, some abort
6. **No DONE phase training data in some scenarios** — DONE only appears in successful trials
7. **SEARCH phase has very few rows** — SEARCH is brief (~0.16s)

## Impact on Cross-Scenario Evaluation

The empty RGB/depth features significantly limit the neural network's ability
to discriminate phases. The v2_14 classifier was originally trained on baseline
data WITH valid RGB features. Cross-scenario evaluation with empty RGB features
produces poor results (38.9% mixed split accuracy, 20.9% held-out mean accuracy).

The geometry-only feasibility classifier, however, performs excellently (96.4%
accuracy, 0% false-safe rate) because it uses only clearance and offset parameters.

## Files

| File | Description |
|------|-------------|
| multi_scenario_context_vectors.parquet | Main dataset (17,651 rows, 68-dim context) |
| multi_scenario_context_vectors.csv | CSV version for inspection |
| dataset_quality_metrics.json | Quality metrics |
| dataset_metadata.json | Full metadata |
| cross_scenario_v2_14_evaluation.json | Cross-scenario evaluation results |
