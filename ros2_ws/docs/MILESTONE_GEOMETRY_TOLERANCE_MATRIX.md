# Milestone: Geometry/Tolerance Scenario Matrix

Date: 2026-06-10
Status: PLANNED (not started)

## Motivation

The PhD proposal requires demonstrating generalization across peg/hole geometries and
tolerance levels. All 60 existing trials used a single geometry (25mm cylindrical peg,
27mm circular hole, 1mm radial clearance). This milestone defines the systematic
evaluation matrix.

## Scenario Matrix

### Peg Geometries (3 variants)

| ID | Shape | Diameter/Width | Length | Notes |
|----|-------|---------------|--------|-------|
| P1 | Cylindrical | 25mm | 110mm | Current (validated) |
| P2 | Cylindrical | 22mm | 110mm | Tight tolerance (2mm clearance with H1) |
| P3 | Square | 25mm × 25mm | 110mm | Non-circular (tests orientation) |

### Hole Geometries (3 variants)

| ID | Shape | Diameter/Width | Plate Thickness | Notes |
|----|-------|---------------|----------------|-------|
| H1 | Circular | 27mm | 20mm | Current (validated) |
| H2 | Circular | 23mm | 20mm | Tight tolerance (1mm clearance with P1) |
| H3 | Square | 27mm × 27mm | 20mm | Non-circular (tests orientation) |

### Clearance/Tolerance Levels (3 levels)

| Level | Peg Dia | Hole Dia | Radial Clearance | Description |
|-------|---------|----------|-------------------|-------------|
| Loose | 25mm | 27mm | 1.0mm | Current (validated) |
| Medium | 25mm | 26mm | 0.5mm | Moderate tolerance |
| Tight | 25mm | 25.5mm | 0.25mm | High-precision |

### Start Conditions

| Condition | Description |
|-----------|-------------|
| Centered | XY offset = 0mm (nominal) |
| Misaligned-1mm | XY offset = ±1mm in X and Y |
| Misaligned-2mm | XY offset = ±2mm in X and Y |

### Scenario Combinations

**Tier 1 — Clearance sweep (same geometry, different tolerance):**
- P1 × H1 × Loose (existing baseline)
- P1 × H1 × Medium
- P1 × H1 × Tight

**Tier 2 — Shape variation (different shape, same tolerance):**
- P3 × H3 × Loose (square peg, square hole)
- P1 × H1 × Loose (cylindrical, existing)

**Tier 3 — Cross-geometry generalization:**
- P2 × H2 × Loose (smaller peg, smaller hole)
- P3 × H1 × Loose (square peg, round hole — worst case)

### Validation Protocol

- 10 trials per scenario (centered start)
- 10 trials per scenario (misaligned ±1mm start)
- Total: 7 scenarios × 20 trials = **140 new trials minimum**

### Metrics Per Scenario

| Metric | Definition |
|--------|-----------|
| Success rate | Physical insertion completion / total trials |
| SEARCH convergence rate | Trials entering SEARCH that converge |
| Insertion depth | Final peg depth in hole (m) |
| Final XY error | Lateral alignment after insertion (m) |
| Contact force | Peak force during insertion (N) |
| Safety abort rate | Trials aborted by safety gate |
| Timeout rate | Trials exceeding 600s deadline |

### SDF/URDF Changes Required

1. **Parameterize peg dimensions** in `cylindrical_peg/model.sdf` (radius, length)
2. **Create square peg SDF** (box geometry instead of cylinder)
3. **Parameterize hole dimensions** in `target_plate/model.sdf` (hole radius via box segments)
4. **Create square hole world** (4 box segments with square opening)
5. **Expose clearance parameter** in `research_baseline.launch.py` and `task_geometry.yaml`
6. **Update experiment runner** to accept geometry/clearance launch arguments

### Acceptance Criteria

- All 7 scenarios execute without Gazebo crashes
- Loose/Medium clearance: success rate ≥ 75%
- Tight clearance: success rate ≥ 50% (expected degradation)
- Square geometry: success rate ≥ 60% (expected harder than cylindrical)
- All metrics logged to `diagnostics/geometry_tolerance_matrix/`

### Estimated Effort

- SDF/URDF parameterization: 2-3 days
- Scenario matrix runner: 1-2 days
- 140 Gazebo trials: 3-5 days (parallelized)
- Analysis and documentation: 1-2 days
- **Total: 1-2 weeks**
