# Test 1: Software Verification — Path to Full Run

## Objective

Get `tests/test1_software_verification/` to a state where running `run.py` produces complete, valid results for all 10 objects across both sub-tests (latency + occlusion), and `plot_results.py` generates all 6 figures plus the LaTeX table fragment.

## Current State Assessment

### What exists and works

| Component | Status | Notes |
|-----------|--------|-------|
| `ffi_bridge.py` | Working | Tested: API v5 returns valid grasps in ~110-130ms |
| `view_geometry.py` | Working | Head camera fixed to look at hand area; wrist camera works |
| `occlusion.py` | Working | Frustum cull + depth buffer produce realistic partial views |
| `hand_approaches.py` | Written | All 13 objects have approach poses defined |
| `object_registry.py` | Working | Discovers `.npz` files from `objects/parametric/` and `objects/ycb/` |
| `config/grasp_preshaping.yaml` | Ready | Copy of production config |
| `config/grasp_preshaping_baseline.yaml` | Ready | 100k samples, 10 iterations |
| `run.py` | Partially working | Latency test verified; occlusion test logic complete but untested with baselines |
| `plot_results.py` | Working | All 6 figures + LaTeX table generated from test data |
| `run_tier_b.py` | Skeleton | Placeholder only — not needed for Test 1 |

### What is broken or missing

1. **No `.npz` object files on disk** — the `objects/parametric/` directory is empty (the previous test run's files were cleaned). The parametric objects need to be regenerated.

2. **YCB mesh placement mismatch** — the meshes are at:
   - `objects/ycb/banana.obj` (flat files)
   - `objects/ycb/mug.obj`
   - `objects/ycb/power_drill.obj`
   
   But `generate_objects.py` expects them inside subdirectories:
   - `objects/ycb/011_banana/textured.obj`
   - `objects/ycb/016_mug/textured.obj`
   - `objects/ycb/035_power_drill/textured.obj`
   
   The mesh filenames are `banana.obj`, `mug.obj`, `power_drill.obj` (not `textured.obj`), and they're not in subdirectories. The `_find_mesh()` fallback searches for any `.obj` in the directory, but since the files are directly in `objects/ycb/` rather than in `objects/ycb/011_banana/`, the directory check at `gen_ycb_objects()` line 348 (`os.path.isdir(obj_dir)`) will fail because it looks for `objects/ycb/011_banana/` which doesn't exist.

3. **YCB mesh scale unknown** — the meshes are Blender-exported with vertex values like `-0.050751`. The auto-scaling heuristic in `generate_objects.py` (lines 371-377) checks `max_extent > 1.0` to detect mm, but we don't know if these are already in meters or some other unit until we try.

4. **Hand approach poses are unvalidated** — the previous test showed `cylinder_upright` gets a "lateral" grasp (score 0.217) when it should be "cylindrical". This suggests the approach pose may not place the ROI over the object correctly. All 10 objects need validation.

5. **Baseline subprocess mechanism untested** — `compute_baseline()` spawns a subprocess with a different `GRASP_CONFIG_PATH`. The `OnceLock` config caching in Rust means the config is loaded once per process, so the subprocess approach is correct in principle, but it hasn't been tested with the actual high-fidelity config.

6. **No `trimesh` dependency confirmed** — `generate_objects.py` needs `trimesh` to load `.obj` meshes. It's not clear if this is installed in the Docker image or locally.

7. **Stale CSV results** — `results/` contains CSVs from the previous partial test run (2 objects, 3 reps, no baselines). These should be cleaned before a fresh run.

8. **The `run.py` debug flag depends on `pyyaml`** — the `--debug` path at line 507 imports `yaml` which may not be available.

## Implementation Plan

### Phase 1: Fix YCB Mesh Processing (make the meshes work)

- [ ] **1.1** Update `generate_objects.py` to handle flat-file YCB meshes. Change `gen_ycb_objects()` to also search `YCB_DIR` directly (not just subdirectories) for `.obj` files. The object ID should be derived from the filename (e.g., `banana.obj` → `banana`, `power_drill.obj` → `power_drill`).
  - *Rationale*: The user placed meshes as flat files, not in YCB-style subdirectories. The code should handle both layouts.
  - *Files*: `generate_objects.py:338-386`

- [ ] **1.2** Update `YCB_OBJECTS` dict keys to match the actual filenames. Change from `"011_banana"` to `"banana"`, `"016_mug"` to `"mug"`, `"035_power_drill"` to `"power_drill"`. Alternatively, add a mapping layer that tries both naming conventions.
  - *Rationale*: The `.npz` output name becomes the object name used throughout the test. If the mesh is `banana.obj`, the object should be `banana` (or we add an alias).
  - *Files*: `generate_objects.py:309-322`

- [ ] **1.3** Update `hand_approaches.py` to match the new object names. If we rename from `011_banana` to `banana`, the APPROACHES dict must follow.
  - *Files*: `hand_approaches.py:69-89`

- [ ] **1.4** Verify mesh scale after first generation. Print the bounding box dimensions of each YCB object after centering and scaling, and confirm they're in a realistic range (5-20cm extent). Adjust the scaling heuristic if needed.
  - *Rationale*: Wrong scale = wrong grasp = meaningless test.

### Phase 2: Regenerate All Object Files

- [ ] **2.1** Run `python generate_objects.py` to create the 7 parametric `.npz` files.
  - *Verification*: 7 files appear in `objects/parametric/`, each with ~10000 points.

- [ ] **2.2** Run `python generate_objects.py --ycb` to process the 3 YCB meshes into `.npz` files.
  - *Prerequisite*: Phase 1 changes, `trimesh` installed.
  - *Verification*: 3 additional `.npz` files appear in `objects/ycb/`.

- [ ] **2.3** Install `trimesh` if not available: `pip install trimesh`. Add to Docker image if running inside Docker.
  - *Note*: Only needed for mesh processing, not for the test run itself.

### Phase 3: Validate Hand Approach Poses

This is the most critical and iterative step. For each object, we need to confirm the SMC sampler's ROI prediction covers the object.

- [ ] **3.1** Run a single grasp computation for each object with `--debug` and `--repetitions 1`:
  ```
  GRASP_PRESHAPING_LIB_PATH=... python run.py --objects <name> --repetitions 1 --latency-only
  ```
  Check the output: does it produce a reasonable grasp type? Does the score suggest the planner found the object?

- [ ] **3.2** For any object that returns `score < 0.3` or wrong grasp type, adjust the approach pose in `hand_approaches.py`. Common fixes:
  - Move hand closer/further (`px` value)
  - Adjust height (`pz` value) — especially for tall objects like `tapered_bottle`
  - For objects with specific grasp targets (mug handle, drill handle), offset the approach direction
  
- [ ] **3.3** Validate all 10 objects produce valid grasps (score > 0.3, grasp type matches `expected_grasp`). Document any objects where the approach needed adjustment.

### Phase 4: Run Baseline Computations

- [ ] **4.1** Test the baseline subprocess mechanism with one object first:
  ```
  GRASP_PRESHAPING_LIB_PATH=... python run.py --objects cylinder_upright --repetitions 1 --occlusion-only
  ```
  This will compute the baseline (may take 1-5 minutes with 100k samples) then run single-view and multi-view.

- [ ] **4.2** If the baseline fails, debug the subprocess mechanism. Common issues:
  - `GRASP_CONFIG_PATH` not picked up by the subprocess
  - `.so` library not found in subprocess
  - LUT file not found (different working directory)
  
- [ ] **4.3** If baselines are too slow (>5 min each), reduce `prediction_samples` in the baseline config from 100000 to 50000. This is still 2.5x the production count.

### Phase 5: Full Production Run

- [ ] **5.1** Clean stale results: delete contents of `results/` directory.

- [ ] **5.2** Run the complete test:
  ```
  GRASP_PRESHAPING_LIB_PATH=./src/grasp_preshaping/lib/libgrasp_preshaping.so \
  python run.py --repetitions 10
  ```
  Expected duration: ~2-5 minutes for latency + ~20-50 minutes for baselines + ~5 minutes for occlusion tests.

- [ ] **5.3** Verify CSV outputs:
  - `latency_results.csv`: 100 rows (10 objects × 10 reps)
  - `occlusion_results.csv`: 200 rows (10 objects × 2 conditions × 10 reps)
  - `intent_precision_summary.csv`: 20 rows
  - `intent_precision_delta.csv`: 10 rows

### Phase 6: Generate Figures

- [ ] **6.1** Run `python plot_results.py --format pdf` (or `png`).

- [ ] **6.2** Inspect each figure for correctness:
  - Fig 1 (latency boxplot): all objects under 400ms?
  - Fig 2 (latency summary): P95 annotated?
  - Fig 3 (intent precision): multi-view bars visibly different from single-view?
  - Fig 4 (intent delta): most objects show positive Δ?
  - Fig 5 (pose error scatter): multi-view cluster closer to origin?
  - Fig 6 (cloud coverage): multi-view shows higher coverage?

- [ ] **6.3** Check the LaTeX table fragment in `figures/test1_results_table.tex` renders correctly.

## Verification Criteria

- [ ] All 10 objects produce valid grasps in both latency and occlusion tests
- [ ] Latency P95 ≤ 400ms for all objects (MAR threshold)
- [ ] Intent precision Δ > 0% (MAR) and ideally ≥ 10% (IDE)
- [ ] All 6 figures generated without errors
- [ ] LaTeX table compiles in the thesis document

## Potential Risks and Mitigations

1. **YCB meshes have wrong scale**
   - Mitigation: Print bounding box after loading; add explicit scale parameter per object if auto-detection fails.

2. **Hand approach poses don't work for complex objects (mug, drill)**
   - Mitigation: These objects have non-trivial geometry. May need to approach from a specific angle. Use debug dumps to diagnose.

3. **Baseline computation too slow or runs out of memory**
   - Mitigation: Reduce `prediction_samples` from 100k to 50k. The relative comparison (single vs. multi) is still valid.

4. **Occlusion test shows no difference between single and multi-view**
   - Mitigation: This would indicate the camera geometry or object placement doesn't create meaningful self-occlusion. Would need to adjust camera positions or add more complex objects.

5. **`trimesh` not available in Docker image**
   - Mitigation: Process meshes locally (outside Docker), commit the `.npz` files. The test itself doesn't need `trimesh`.

## Alternative Approaches

1. **Skip YCB meshes entirely, use parametric fallbacks**: If mesh processing proves difficult, the 3 parametric fallbacks (banana_fallback, mug_fallback, drill_fallback) are already implemented. Less realistic but still tests the multi-view advantage.

2. **Run everything inside Docker**: Instead of using `GRASP_PRESHAPING_LIB_PATH` locally, run the entire test inside the Docker container where the `.so` is installed at the standard path. More reproducible but slower iteration.

3. **Separate baseline into a pre-computation step**: Instead of computing baselines during `run.py`, create a separate `compute_baselines.py` that saves results to a JSON file. This makes it easier to retry individual baselines without re-running the whole test.
