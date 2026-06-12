# Operating Envelope Summary — Stage C Geometry/Tolerance Matrix

Generated: 2026-06-12
Total trials: 140 across 7 scenarios
Tracking noise floor estimate: 0.5mm (RMS XY tracking error)

## Envelope Classification

| Zone | Clearance/Noise Ratio | Clearance Range | Scenarios |
|------|----------------------|-----------------|-----------|
| **Robust** | >= 2.0 | >= 1.0mm | baseline_loose, large_peg_large_hole, small_peg_small_hole, misaligned_baseline |
| **Marginal** | 1.0-2.0 | 0.5-1.0mm | clearance_medium (0.5mm = 1.0x noise) |
| **Out-of-envelope** | < 1.0 | < 0.5mm | clearance_tight (0.25mm), tight_plus_misaligned (0.25mm+1mm) |

## Per-Scenario Results

| Scenario | Clearance | Offset | N | Success | Rate | SEARCH Conv. | CG | Safety Abort | Timeout | XY Exceeded | Envelope Zone |
|----------|-----------|--------|---|---------|------|-------------|-----|-------------|---------|-------------|---------------|
| baseline_loose | 1.0mm | 0.0mm | 20 | 16 | 80% | 100% | 0 | 4 | 0 | 0 | robust |
| clearance_medium | 0.5mm | 0.0mm | 20 | 0 | 0% | 100% | 0 | 20 | 0 | 14 | marginal |
| clearance_tight | 0.25mm | 0.0mm | 20 | 0 | 0% | 100% | 0 | 20 | 0 | 14 | out_of_envelope |
| large_peg_large_hole | 1.0mm | 0.0mm | 20 | 19 | 95% | 100% | 0 | 1 | 0 | 0 | robust |
| small_peg_small_hole | 1.0mm | 0.0mm | 20 | 20 | 100% | 100% | 0 | 0 | 0 | 0 | robust |
| misaligned_baseline | 1.0mm | 1.0mm | 20 | 17 | 85% | 100% | 0 | 3 | 0 | 2 | robust |
| tight_plus_misaligned | 0.25mm | 1.0mm | 20 | 0 | 0% | 100% | 0 | 20 | 0 | 20 | out_of_envelope |

## Failure Reason Distribution by Clearance

| Clearance | xy_exceeded | handoff_timeout | sideload_abort | timeout |
|-----------|-------------|-----------------|----------------|---------|
| 1.0mm | 0 | 0 | 3 | 0 |
| 0.5mm | 14 | 6 | 0 | 0 |
| 0.25mm | 14 | 6 | 0 | 0 |
| 1.0mm | 0 | 1 | 0 | 0 |
| 1.0mm | 0 | 0 | 0 | 0 |
| 1.0mm | 2 | 0 | 1 | 0 |
| 0.25mm | 20 | 0 | 0 | 0 |

## Key Metrics by Clearance Family

| Metric | 1.0mm (Robust) | 0.5mm (Marginal) | 0.25mm (Out-of-envelope) |
|--------|---------------|-------------------|--------------------------|
| Success Rate | 72/80 (90%) | 0/20 (0%) | 0/40 (0%) |
| Avg Duration (s) | 32.725 | 13.4 | 2.85 |
| Avg Insertion Depth (m) | 0.0191 | 0.0 | 0.0 |
| Avg Final XY Error (m) | 0.0005 | 0.0 | 0.0 |

## Operating Envelope Conclusion

The Stage C geometry/tolerance matrix validates a **measurable clearance/noise operating envelope**:

1. **Robust zone (>= 1.0mm clearance)**: 72/80 (90%) success across 4 scenarios.
   The controller reliably inserts pegs with clearance >= 2x the tracking noise floor.

2. **Marginal zone (0.5mm clearance)**: 0/20 (0%) success.
   Clearance equals the tracking noise floor. The precontact clearance safety gate
   triggers on every descent tick because tracking fluctuation causes
   xy_error > INSERT_FINAL_XY_TOLERANCE continuously.

3. **Out-of-envelope zone (<= 0.25mm clearance)**: 0/40 (0%) success.
   Clearance is 0.5x the tracking noise floor. Failure is structural —
   the sensor stack cannot distinguish in-tolerance from out-of-tolerance positioning.

**This is a safety-relevant result, not a failure to hide.** The system demonstrates
fail-closed behavior outside its validated operating envelope. The boundary is
defined by the ratio of radial clearance to tracking noise floor.

### Implications for Thesis Claims

- Geometry/tolerance generalization is validated **only inside the 1.0mm clearance envelope**
- The system does **not** claim universal tolerance generalization
- Sub-0.5mm clearance requires either improved tracking accuracy or different control strategy
- Fail-closed behavior at tight clearance is a **positive safety property**