# Operating Envelope Analysis

Date: 2026-06-12
Stage: C (full matrix, 20 trials per scenario = 140 total)
Dataset: `diagnostics/geometry_tolerance_matrix_stage_c/`

## Purpose

This analysis defines the **measured clearance/noise operating envelope** of the
deterministic admittance controller for peg-in-hole assembly. The Stage C scenario
matrix systematically varies peg/hole geometry, radial clearance, and initial offset
to identify where the controller is robust and where it fails closed.

## Scenario Matrix

| Scenario | Peg | Hole | Clearance | Offset | Tier |
|----------|-----|------|-----------|--------|------|
| baseline_loose | 25mm | 27mm | 1.00mm | 0.0mm | clearance_sweep |
| clearance_medium | 25mm | 26mm | 0.50mm | 0.0mm | clearance_sweep |
| clearance_tight | 25mm | 26mm | 0.25mm | 0.0mm | clearance_sweep |
| large_peg_large_hole | 28mm | 30mm | 1.00mm | 0.0mm | size_variation |
| small_peg_small_hole | 22mm | 24mm | 1.00mm | 0.0mm | size_variation |
| misaligned_baseline | 25mm | 27mm | 1.00mm | 1.0mm | combined_difficulty |
| tight_plus_misaligned | 25mm | 26mm | 0.25mm | 1.0mm | combined_difficulty |

## Results Summary

| Scenario | N | Success | Rate | SEARCH Conv. | CG | XY Exceeded | Handoff TO | Sideload | Envelope |
|----------|---|---------|------|-------------|-----|-------------|------------|----------|----------|
| baseline_loose | 20 | 16 | 80% | 100% | 0 | 0 | 0 | 3 | robust |
| clearance_medium | 20 | 0 | 0% | 100% | 0 | 14 | 6 | 0 | marginal |
| clearance_tight | 20 | 0 | 0% | 100% | 0 | 14 | 6 | 0 | out_of_envelope |
| large_peg_large_hole | 20 | 19 | 95% | 100% | 0 | 0 | 1 | 0 | robust |
| small_peg_small_hole | 20 | 20 | 100% | 100% | 0 | 0 | 0 | 0 | robust |
| misaligned_baseline | 20 | 17 | 85% | 100% | 0 | 2 | 0 | 1 | robust |
| tight_plus_misaligned | 20 | 0 | 0% | 100% | 0 | 20 | 0 | 0 | out_of_envelope |

## Operating Envelope Classification

The operating envelope is defined by the ratio of **radial clearance** to **tracking noise floor**.

- **Tracking noise floor**: ~0.5mm (estimated from XY error distributions)
- **Robust zone**: clearance >= 2x noise floor (>= 1.0mm)
- **Marginal zone**: clearance >= 1x noise floor (>= 0.5mm)
- **Out-of-envelope zone**: clearance < 1x noise floor (< 0.5mm)

### Robust Zone (>= 1.0mm clearance)

- **baseline_loose**: 16/20 (80%), SEARCH converge 100%
- **large_peg_large_hole**: 19/20 (95%), SEARCH converge 100%
- **small_peg_small_hole**: 20/20 (100%), SEARCH converge 100%
- **misaligned_baseline**: 17/20 (85%), SEARCH converge 100%
- **Total**: 72/80 (90%)

### Marginal Zone (0.5mm clearance)

- **clearance_medium**: 0/20 (0%), failure modes: {'xy_exceeded': 14, 'handoff_settle_timeout': 6}

### Out-of-Envelope Zone (< 0.5mm clearance)

- **clearance_tight**: 0/20 (0%), failure modes: {'handoff_settle_timeout': 6, 'xy_exceeded': 14}
- **tight_plus_misaligned**: 0/20 (0%), failure modes: {'xy_exceeded': 20}

## Failure Mode Analysis

### xy_exceeded (precontact clearance gate trigger)

The INSERT phase checks `xy_error > INSERT_FINAL_XY_TOLERANCE` (radial clearance)
before each descent. When tracking noise >= clearance, this gate triggers on every
tick, exhausting the recenter budget and causing abort.

| Clearance | xy_exceeded count | Mechanism |
|-----------|-------------------|-----------|
| 0.5mm | 14 | noise=clearance |
| 0.25mm | 14 | noise<clearance |
| 1.0mm | 2 | noise<clearance |
| 0.25mm | 20 | noise<clearance |

### handoff_settle_timeout

The peg fails to stabilize during the handoff from SEARCH to INSERT.
This occurs at all clearance levels but dominates at medium/tight clearance
where the peg cannot find a stable insertion pose.

### sideload_abort

The peg becomes side-loaded during insertion descent.
This is a rare failure mode observed only at loose clearance (baseline_loose, misaligned_baseline).

## Tracking Noise / XY Error Distribution

### Successful Trials — Final XY Error

| Scenario | Clearance | Avg Final XY Error | Max Final XY Error |
|----------|-----------|-------------------|-------------------|
| baseline_loose | 1.0mm | 0.0004m | — |
| large_peg_large_hole | 1.0mm | 0.0005m | — |
| small_peg_small_hole | 1.0mm | 0.0004m | — |
| misaligned_baseline | 1.0mm | 0.0006m | — |

### Clearance-to-Noise Ratio

| Scenario | Clearance (mm) | Noise (mm) | Ratio | Zone |
|----------|---------------|------------|-------|------|
| baseline_loose | 1.0 | 0.5 | 2.0 | robust |
| clearance_medium | 0.5 | 0.5 | 1.0 | marginal |
| clearance_tight | 0.25 | 0.5 | 0.5 | out_of_envelope |
| large_peg_large_hole | 1.0 | 0.5 | 2.0 | robust |
| small_peg_small_hole | 1.0 | 0.5 | 2.0 | robust |
| misaligned_baseline | 1.0 | 0.5 | 2.0 | robust |
| tight_plus_misaligned | 0.25 | 0.5 | 0.5 | out_of_envelope |

## Conclusions

1. **The controller operates reliably when radial clearance >= 2x tracking noise floor.**
   At 1.0mm clearance (2.0x noise), the success rate is 90% (72/80).

2. **The controller fails closed when radial clearance < tracking noise floor.**
   At 0.5mm clearance (1.0x noise), the success rate is 0% (0/20).
   At 0.25mm clearance (0.5x noise), the success rate is 0% (40/40).

3. **This is a measured operating envelope, not a bug.**
   The precontact clearance safety gate is working as designed — it prevents
   insertion attempts when the peg position is not within the physical clearance.
   At tight clearance, tracking noise makes this condition unsatisfiable.

4. **Fail-closed behavior is a safety-relevant property.**
   The system correctly refuses to attempt insertion when positioning accuracy
   is insufficient. This prevents jamming, damage, and unsafe contact forces.

5. **The project demonstrates geometry/tolerance generalization inside the
   validated envelope, not universal tolerance generalization.**
   Claims are limited to scenarios where clearance >= 1.0mm.

## Implications for Thesis

- The deterministic controller establishes a **measurable operating envelope**
- Cross-scenario generalization is validated for the 1.0mm clearance family
- Sub-0.5mm clearance requires either improved tracking or different control strategy
- The SAC/meta-RL learner can target improvement inside the envelope
- Out-of-envelope detection is a valuable advisory capability
- The fail-closed property is a positive contribution to safe assembly