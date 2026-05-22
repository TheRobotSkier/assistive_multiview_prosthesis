# Test 1: Software Verification — Path to Full Run (v2)

## Objective

Get `tests/test1_software_verification/` to a state where running `run.py` produces complete, valid results for all 10 objects across both sub-tests (latency + occlusion), and `plot_results.py` generates all 6 figures plus the LaTeX table fragment.

## Current State

### Verified Working

| Component | Status | Evidence |
|-----------|--------|---------|
| `ffi_bridge.py` | Working | API v5, valid grasps in ~110-130ms |
| `view_geometry.py` | Working | Head + wrist cameras produce realistic views |
| `occlusion.py` | Working | Wrist camera sees 17-89% of objects |
| `object_registry.py` | Working | Discovers `.npz` files correctly |
| `run.py` (latency) | Working | CSVs produced for 2 test objects |
| `plot_results.py` | Working | All 6 figures + LaTeX table generated |
| Parametric `.npz` files | **Exist** | 7 files in `objects/parametric/` (binary, not visible to text search) |
| Config files | Ready | Production + baseline configs in place |

### Issues to Fix

**1. YCB mesh processing won't find the meshes (blocking)**

Meshes are flat files:
```
objects/ycb/banana.obj
objects/ycb/mug.obj
objects/ycb/power_drill.obj
```

But `generate_objects.py:347-348` expects subdirectories like `objects/ycb/011_banana/` and the `YCB_OBJECTS` dict keys are `"011_banana"`, `"016_mug"`, `"035_power_drill"` — neither the directory structure nor the naming matches.

**2. No YCB `.npz` files exist yet (blocking)**

The 3 YCB meshes haven't been processed into point clouds. `trimesh` is now installed.

**3. Hand approach poses unvalidated (quality risk)**

`cylinder_upright` gets a "lateral" grasp (score 0.217) instead of "cylindrical". All 10 objects need validation once YCB objects are available.

**4. Baseline subprocess untested (blocking for occlusion test)**

The high-fidelity baseline computation spawns a subprocess with a different config. Never tested.

**5. Stale results in `results/`**

CSVs from the partial test (2 objects, 3 reps, no baselines) should be cleaned before a full run.

## Implementation Plan

### Phase 1: Fix YCB Mesh Processing

- [ ] **1.1** Rewrite `gen_ycb_objects()` in `generate_objects.py` to handle flat `.obj` files directly in `objects/ycb/`. Add a scan for any `.obj` file in `YCB_DIR` itself, deriving the object name from the filename (e.g., `banana.obj` → `"banana"`). Keep the existing subdirectory scan as a fallback for future YCB-style layouts.
  - *Rationale*: The user's meshes are flat files, not in YCB subdirectories. Both layouts should work.
  - *File*: `generate_objects.py:338-386`

- [ ] **1.2** Update `YCB_OBJECTS` dict to use the actual filenames as keys: `"banana"`, `"mug"`, `"power_drill"`. Remove the `011_`/`016_`/`035_` prefixes since the meshes don't use YCB IDs.
  - *Rationale*: Object names must be consistent across `generate_objects.py`, `object_registry.py`, and `hand_approaches.py`.
  - *File*: `generate_objects.py:309-322`

- [ ] **1.3** Update `hand_approaches.py` to rename YCB entries from `"011_banana"` → `"banana"`, `"016_mug"` → `"mug"`, `"035_power_drill"` → `"power_drill"`. Remove the fallback entries since real meshes exist.
  - *File*: `hand_approaches.py:69-109`

### Phase 2: Generate YCB Point Clouds

- [ ] **2.1** Run `python generate_objects.py --ycb` to process the 3 meshes into `.npz` files.
  - *Prerequisite*: Phase 1 changes, `trimesh` installed (done).
  - *Verification*: 3 new `.npz` files in `objects/ycb/`.

- [ ] **2.2** Print and verify bounding box dimensions of each YCB object. The Blender-exported vertices (e.g., `-0.050751`) suggest meters already. Confirm each object is in the 5-20cm extent range. If not, adjust the scaling heuristic.
  - *Rationale*: Wrong scale = wrong grasp = meaningless test.

### Phase 3: Validate Hand Approach Poses

- [ ] **3.1** Run a quick grasp computation for each of the 10 objects (7 parametric + 3 YCB):
  ```
  python run.py --objects <name> --repetitions 1 --latency-only
  ```
  Check: does it produce a reasonable grasp type? Is the score > 0.3?

- [ ] **3.2** For any object with score < 0.3 or wrong grasp type, adjust the approach pose in `hand_approaches.py`. Common fixes:
  - Move hand closer/further (`px`)
  - Adjust height (`pz`) for tall objects
  - Offset approach for objects with specific grasp targets (mug handle, drill handle)

- [ ] **3.3** Document which objects needed adjustment and what the final poses are.

### Phase 4: Test Baseline Subprocess

- [ ] **4.1** Run the occlusion test for one object with baselines enabled:
  ```
  python run.py --objects cylinder_upright --repetitions 1 --occlusion-only
  ```
  This computes the baseline (may take 1-5 min with 100k samples) then runs single-view and multi-view.

- [ ] **4.2** If baseline fails, debug the subprocess. Likely issues:
  - `.so` library path not found in subprocess (set `GRASP_PRESHAPING_LIB_PATH`)
  - Config not picked up (check `GRASP_CONFIG_PATH` env propagation)
  - LUT file path baked in at compile time (should work if `.so` path is correct)

- [ ] **4.3** If baselines are too slow (>5 min each), reduce `prediction_samples` in baseline config from 100000 to 50000.

### Phase 5: Full Production Run

- [ ] **5.1** Clean stale results: delete CSV files in `results/` (keep the directory).

- [ ] **5.2** Run the complete test with all 10 objects:
  ```
  GRASP_PRESHAPING_LIB_PATH=./src/grasp_preshaping/lib/libgrasp_preshaping.so \
  python run.py --repetitions 10
  ```
  Expected: ~2-5 min latency + ~20-50 min baselines + ~5 min occlusion.

- [ ] **5.3** Verify CSV outputs:
  - `latency_results.csv`: ~100 rows (10 objects × 10 reps)
  - `occlusion_results.csv`: ~200 rows (10 objects × 2 conditions × 10 reps)
  - `intent_precision_summary.csv`: 20 rows
  - `intent_precision_delta.csv`: 10 rows

### Phase 6: Generate Figures

- [ ] **6.1** Run `python plot_results.py --format pdf` (or `png`).

- [ ] **6.2** Inspect each figure:
  - Fig 1 (latency boxplot): all objects under 400ms?
  - Fig 2 (latency summary): P95 annotated?
  - Fig 3 (intent precision): multi-view visibly different from single-view?
  - Fig 4 (intent delta): most objects show positive Δ?
  - Fig 5 (pose error scatter): multi-view cluster closer to origin?
  - Fig 6 (cloud coverage): multi-view higher coverage?

- [ ] **6.3** Verify LaTeX table in `figures/test1_results_table.tex` compiles.

## Verification Criteria

- [ ] All 10 objects produce valid grasps in both sub-tests
- [ ] Latency P95 ≤ 400ms for all objects
- [ ] Intent precision Δ > 0% (MAR threshold)
- [ ] All 6 figures generated without errors
- [ ] LaTeX table fragment is correct

## Risks and Mitigations

1. **YCB meshes have wrong scale** → Print bounding box after loading; add per-object scale factor if needed.
2. **Approach poses don't work for mug/drill** → Use debug dumps to diagnose ROI coverage; adjust pose iteratively.
3. **Baseline too slow** → Reduce to 50k samples; relative comparison still valid.
4. **No Δ between single/multi-view** → Adjust camera positions or add more complex objects.
5. **`trimesh` issues with these specific `.obj` files** → The meshes are Blender-exported with normals and materials references; `trimesh.load(path, force='mesh')` should handle this. If not, strip `mtllib` lines or use `force='scene'` then traverse.

## Alternative: Skip YCB, Use Parametric Fallbacks

If mesh processing proves difficult, run `python generate_objects.py --fallbacks` to get 3 parametric approximations. Less realistic but still tests the multi-view advantage. The 7 core parametric objects are already available and sufficient for a first pass.
