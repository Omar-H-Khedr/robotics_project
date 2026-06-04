# perception_pipeline_context_extract_v1

Phase 5/6 v2_12 first end-to-end test: feed the perception logger
output through the new context vector extractor and verify the
parquet is well-formed.

## Trial config
- Input: `diagnostics/perception_pipeline_d405_smoke/multimodal_observation_log.csv`
  (25 s smoke trial, 183 rows, 31 columns, 425 KB)
- Output: `diagnostics/perception_pipeline_context_extract_v1/context_log.parquet`
- Command: `ros2 launch perception_pipeline context_vector_extractor.launch.py
  input_csv:=... output_parquet:=...`

## Context vector layout (74 dims, v2_12)
- `[0:48]` RGB: 64x36 PNG decoded, resized to 8x6 grayscale, normalized [0,1]
- `[48:54]` depth: (w, h, min_m, max_m, roi_min_m, roi_max_m)
- `[54:60]` wrench: (fx, fy, fz, tx, ty, tz)
- `[60:66]` joint position: (j1..j6) rad
- `[66:72]` joint velocity: (j1..j6) rad/s
- `[72]` phase_int (enum: UNKNOWN=0..COMPLETE=8)
- `[73]` safety_int (enum: UNKNOWN=0..ABORT=3)

NaN is replaced with 0.0; +/-inf is clipped to +/- 1e6 m. Empty /
missing fields are filled with 0.0. The schema metadata carries the
full spec and enum maps as `context_spec_json` so downstream
consumers can rebuild the column meaning.

## What this proves
- The CSV produced by the live `multimodal_observation_logger` is
  decodable offline into a fixed-length context vector.
- `context_log.parquet` is well-formed: 182 rows x 74 dims, no inf,
  no NaN, all-zero columns where the source was absent (wrench in
  this smoke run, joint vel because JTC was not active yet).
- RGB block has real variation (std 0.07-0.10 across rows).
- All 182 rows in this smoke have phase_int=0, safety_int=0 because
  no /task_phase or /safety_status topic was being published. Real
  trials with the controller running should produce non-zero phase
  and safety values for at least some rows.

## Known limitations
- The depth image is not stored in the logger (only 6 summary stats),
  so the context vector can only carry those 6 numbers. A future v2_12+
  could swap to storing the depth image (or a small downsample) for a
  richer per-tick depth feature. Not blocking for v2_12.
- Task_phase and safety_status are still absent from the live cell;
  rows are tagged with phase_int=0, safety_int=0 throughout. The
  v2_13 encoder must either (a) wait for those topics to exist or
  (b) train on whatever labeled rows become available.
- RGB resize uses PIL BILINEAR; v2_13 may prefer a different resize
  filter or no resize (use a small CNN instead). Re-evaluate when
  v2_13 is designed.
