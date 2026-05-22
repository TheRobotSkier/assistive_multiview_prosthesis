# Test 1: Software Verification — Readiness Plan (v2)

## Objective

Prepare the Test 1 (Software Verification) suite for a clean, reproducible run that produces publication-quality figures and valid results. The test validates Req 1.6 (autonomous grasp computation), Req 2.4 (pipeline latency ≤ 400 ms MAR / ≤ 100 ms IDE), and Req 2.7 (intent precision Δ > 0% MAR / ≥ 10% IDE).

---

## Current State Assessment (Complete Investigation)

### What exists and works:
- **All 9 Python modules** implemented: `run.py`, `run_tier_b.py`, `ffi_bridge.py`, `occlusion.py`, `view_geometry.py`, `hand_approaches.py`, `object_registry.py`, `generate_objects.py`, `plot_results.py`
- **Previous run completed** — CSV results exist in `results/` with 1000 latency rows (10 objects × 100 reps) and 2000 occlusion rows
- **8 figure functions** in `plot_results.py` (fig1–fig8) plus a LaTeX table generator
- **Latency results PASS** — all objects well under 400 ms MAR (mean 74–142 ms, P95 ≤ 171 ms)
- **Intent Precision Δ = +4.4% mean** — passes MAR (>0%) but **FAILS IDE (≥10%)**
- **Tier B** has been implemented with both EMG and service methods, and has results (25 rows in `tier_b_latency_results.csv`)
- **Config files** properly set up with production and baseline variants
- **Previous restructuring** already applied: grasp-type-only primary metric, 100 repetitions, CDF plots

### Critical issues identified:

#### Issue 1: `.npz` object files are MISSING from disk
The `objects/parametric/` directory is **empty** (no `.npz` files found by search). Only the raw YCB `.obj` meshes exist in `objects/ycb/`. The `object_registry.py` discovers objects by scanning for `.npz` files, so running `run.py` now would produce "No objects available" and exit immediately.

**Root cause**: The `.npz` files were likely generated inside a Docker container and not persisted to the host filesystem (container volumes or build artifacts that were cleaned). The results CSVs in `results/` are from a previous run that had the objects available.

**Fix**: Must run `python generate_objects.py --all` (or `--fallbacks`) before the test can execute.

#### Issue 2: Synthetic camera geometry diverges from the real hardware
The `view_geometry.py` camera definitions are **rough approximations** that do not match the actual D435i mount geometry from `camera_mounts.yaml`:

| Parameter | `view_geometry.py` (synthetic) | Real system (`camera_mounts.yaml`, 8_cm mount) |
|---|---|---|
| Wrist camera position | `(0.08, -0.02, 0.0)` relative to hand | Screw-to-palm: `(0.01585, -0.091762, 0.160955)` with quaternion `(-0.5, -0.5, 0.5, -0.5)` — camera looks **downward**, not forward |
| Wrist camera forward | `(1.0, 0.0, 0.0)` (along +X) | Rotated 180° via quaternion: camera looks **down** toward the hand |
| Head camera position | `(-0.15, 0.0, 0.30)` relative to hand | On forehead — position depends on user head pose, not hand frame |
| Head camera forward | `(0.75, 0.0, -0.65)` (forward+down) | Reasonable approximation |
| FOV | 58° × 45° | Correct for D435 at 640×480 |
| Near/far | 0.05–1.00 m (wrist), 0.10–1.50 m (head) | Wrist camera ~16 cm above hand → far should be ~0.5 m |

The wrist camera's real orientation is the critical discrepancy: it looks **downward** toward the hand from ~16 cm above, not forward. This fundamentally changes which surfaces are visible and which are self-occluded.

#### Issue 3: Wrist orientation errors are extremely high
Existing results show mean wrist rotation errors of **74°–104°** across all objects — essentially random wrist orientation. This suggests the SMC sampler is not converging on wrist pose with partial point clouds.

#### Issue 4: Intent Precision Δ fails IDE threshold
Mean Δ = +4.4% vs. IDE target ≥10%. Only `power_drill` achieves meaningful Δ (+19%). Several objects show negative Δ (cylinder_upright: -2%, tapered_bottle: -3%). Fixing camera geometry may improve this.

#### Issue 5: Figure styling doesn't match project colors
Existing figures use default matplotlib colors (`#4c72b0`, `#55a868`) instead of the project palette (`#5099e9`, `#fffdf6`, `#f4fbf9`). No actual figure PDFs/PNGs exist on disk — only the `.tex` table fragment.

#### Issue 6: No figures exist on disk
The `figures/` directory only contains `test1_results_table.tex`. No PDF or PNG figures were found. They were likely generated inside a container and not persisted.

---

## Implementation Plan

### Phase 0: Ensure the Test Can Run At All (Prerequisite)

- [ ] **0.1** Verify `trimesh` and `matplotlib` are available in the Docker container
  - The Dockerfile (`docker/Dockerfile:31-33`) only installs `dynamixel-sdk` and `requests` via pip
  - Need to add `trimesh` and `matplotlib` — either in the Dockerfile or install at runtime
  - Rationale: `generate_objects.py` requires `trimesh` for YCB mesh processing; `plot_results.py` requires `matplotlib`

- [ ] **0.2** Generate all object point clouds
  - Run `python generate_objects.py --all` inside the container
  - If YCB mesh loading fails, use `python generate_objects.py --fallbacks` as backup
  - This produces 7 parametric `.npz` files in `objects/parametric/` and 3 YCB/fallback `.npz` files
  - Verify: `object_registry.py` discovers all 10 objects
  - Rationale: Without `.npz` files, `run.py` exits immediately with "No objects available"

- [ ] **0.3** Verify the `.so` library is loadable
  - Check that `libgrasp_preshaping.so` exists at one of the search paths in `ffi_bridge.py:126-140`
  - Primary path: `/prosthesis_ws/install/grasp_preshaping/lib/libgrasp_preshaping.so`
  - Test: `python -c "from ffi_bridge import GraspLibrary; lib = GraspLibrary(); print(lib.api_version)"`
  - Rationale: The entire test depends on the Rust FFI library being available

### Phase 1: Fix Synthetic Camera Geometry to Match Real Hardware

- [ ] **1.1** Derive the real wrist camera position and orientation from `camera_mounts.yaml`
  - The 8_cm mount has `screw_to_palm` translation `(0.01585, -0.091762, 0.160955)` with quaternion `(-0.5, -0.5, 0.5, -0.5)`
  - Invert this to get `palm_to_screw`, then compose with `screw_to_link` to get camera position relative to palm
  - The quaternion `(-0.5, -0.5, 0.5, -0.5)` is a 180° rotation that maps palm-X → screw-Z (camera looks down)
  - The resulting forward vector in palm frame is approximately `(0, 0, -1)` (downward)
  - Use the existing `_invert_transform()` from `publish_camera_mounts.py:78-88` as reference
  - Rationale: Test description says "simulated viewpoints representing the head-mounted and prosthesis-mounted camera positions" — must match real geometry

- [ ] **1.2** Update `WRIST_CAMERA_LOCAL` in `view_geometry.py:35-43`
  - Compute actual camera position relative to hand center using the 8_cm mount transform chain
  - Set forward direction to look downward (toward the hand/object), matching the real mount
  - Keep FOV at 58°×45° (correct for D435)
  - Adjust near/far planes: camera ~16 cm above hand → near=0.05 m, far=0.5 m
  - Key: the camera looks DOWN, not forward — this changes which surfaces are visible

- [ ] **1.3** Update `HEAD_CAMERA_LOCAL` in `view_geometry.py:22-31`
  - Position: approximately `(-0.10, 0.0, 0.35)` relative to hand (behind and above) — already reasonable
  - Forward: `(0.75, 0.0, -0.65)` (angled ~40° downward) — already reasonable
  - Adjust near/far: near=0.15 m, far=1.0 m for a head-mounted camera looking at the hand
  - Less critical than wrist camera since head position varies by user

- [ ] **1.4** Add a camera transform validation function
  - Print computed world-frame camera positions and forward vectors for a sample hand pose
  - Compare against known values from `camera_mounts.yaml`
  - Add as `--validate-cameras` flag or standalone diagnostic
  - Rationale: Sanity check before committing to a full run

### Phase 2: Tune Hand Approach Poses for Realistic Grasp Scenarios

- [ ] **2.1** Review all 10 hand approach poses in `hand_approaches.py:18-110`
  - Current poses all use identity quaternion `(0,0,0,1)` — hand approaches from -X direction
  - With the corrected downward-looking wrist camera, objects must be positioned within the camera's downward frustum
  - The hand pose must place the object below and slightly in front of the camera
  - Rationale: If the object isn't in the camera FOV, the occlusion test produces empty clouds

- [ ] **2.2** Add a dry-run / diagnostic mode to `run.py`
  - New `--dry-run` flag: generates all point clouds, prints coverage stats, does NOT call the planner
  - For each object, print: full cloud size, single-view size, multi-view size, coverage %
  - Target: single-view sees 25–65% of object; multi-view sees 45–85%
  - Rationale: Rapid iteration on camera geometry without expensive SMC computations

- [ ] **2.3** Run diagnostic and adjust poses for any objects with poor coverage
  - Flag objects where single-view sees < 20% or multi-view sees < 40%
  - Adjust hand position (px, py, pz) to place object within camera frustum
  - Re-run diagnostic until all objects have reasonable coverage

### Phase 3: Restyle Figures with Project Colors

- [ ] **3.1** Define project color palette as constants at top of `plot_results.py`
  - Primary blue: `#5099e9` — for single-view bars/lines
  - Multi-view complementary: derive from palette (e.g., `#2ecc71` or `#50C878` — a green that contrasts with blue)
  - Background: `#fffdf6` — warm cream for figure background
  - Axes background: `#f4fbf9` — very light mint
  - Threshold red: `#e74c3c` or similar
  - Rationale: Consistent visual identity with the rest of the report

- [ ] **3.2** Update all 8 figure functions to use the new palette
  - Replace `#4c72b0` (single-view) → `#5099e9`
  - Replace `#55a868` (multi-view) → chosen complementary color
  - Set `fig.patch.set_facecolor('#fffdf6')` and `ax.set_facecolor('#f4fbf9')`
  - Update threshold lines to use palette-consistent colors

- [ ] **3.3** Improve figure typography and layout
  - Font: "DejaVu Sans" (available everywhere) or system default
  - Title: 14pt, labels: 11pt, ticks: 9pt, annotations: 7-8pt
  - Consistent figure widths proportional to number of objects
  - Light gridlines (alpha=0.3) on appropriate axes

- [ ] **3.4** Create a combined summary figure (new Figure 0)
  - Single figure telling the whole Test 1 story:
    - Panel A: Latency bar chart with P95 line
    - Panel B: Intent precision Δ with IDE threshold
    - Panel C: Wrist error CDF (single vs multi)
  - This is the primary figure referenced in the report

### Phase 4: Run the Full Test Suite

- [ ] **4.1** Run the dry-run diagnostic (after Phase 2.2)
  - `python run.py --dry-run`
  - Verify all 10 objects have reasonable coverage
  - Fix any issues before proceeding

- [ ] **4.2** Run Tier A latency test
  - `python run.py --latency-only --repetitions 30`
  - Expected: all objects under 200 ms mean, P95 under 300 ms
  - Verify: `results/latency_results.csv` has ~300 rows

- [ ] **4.3** Run Tier A occlusion proxy test
  - `python run.py --occlusion-only --repetitions 100`
  - Most time-consuming step (~30-60 min with baselines)
  - Verify: `results/occlusion_results.csv` has ~2000 rows, `intent_precision_delta.csv` has 10 rows

- [ ] **4.4** Generate all figures
  - `python plot_results.py --format pdf --dpi 300`
  - Verify all 8+ PDFs are generated in `figures/`

- [ ] **4.5** (Optional) Run Tier B full ROS pipeline test
  - Requires running ROS mock system: `ros2 launch prosthesis_launch mock.launch.py`
  - `python run_tier_b.py --method emg --repetitions 5`
  - Verify: `results/tier_b_latency_results.csv` updated

### Phase 5: Results Analysis and Report Preparation

- [ ] **5.1** Analyze latency results
  - Overall P95 across all objects
  - Verify MAR (≤400 ms) and IDE (≤100 ms) pass/fail
  - Note outliers

- [ ] **5.2** Analyze intent precision results
  - Per-object and mean Δ for grasp type correctness
  - Identify which objects benefit most from multi-view
  - Verify MAR (>0%) and IDE (≥10%) pass/fail
  - If IDE still fails, document honestly — may be genuine result

- [ ] **5.3** Generate updated LaTeX results tables
  - `figures/test1_results_table.tex` with new data
  - Ensure format matches report style

- [ ] **5.4** Write Results section content
  - Summary: pass/fail for each requirement
  - Latency subsection: reference figures, per-object table
  - Occlusion subsection: reference figures, Δ table
  - Key findings and discussion

---

## Verification Criteria

- [ ] **V1: Object files exist** — All 10 `.npz` files present in `objects/parametric/` and `objects/ycb/`
- [ ] **V2: Camera geometry matches real system** — Wrist camera position/orientation derived from `camera_mounts.yaml` 8_cm mount (within 1 cm positional, 10° angular tolerance)
- [ ] **V3: Point cloud coverage is realistic** — Single-view sees 25–65% of full cloud; multi-view sees 45–85% for at least 8 of 10 objects
- [ ] **V4: All figures use project colors** — `#5099e9` for primary, `#fffdf6` backgrounds, consistent styling
- [ ] **V5: Test runs without errors** — `python run.py` completes end-to-end inside Docker with exit code 0
- [ ] **V6: Results are reproducible** — Same seed produces identical CSVs
- [ ] **V7: All 10 objects produce valid results** — No SKIP messages, all have single-view and multi-view data
- [ ] **V8: Figures are publication quality** — 300 DPI, readable labels, no overlapping text
- [ ] **V9: Figures exist on disk** — All PDF/PNG files present in `figures/` directory

---

## Potential Risks and Mitigations

1. **`.npz` generation fails inside container**
   The parametric generators are pure numpy (should work). YCB requires `trimesh` which may not be installed.
   **Mitigation**: Use `--fallbacks` flag for parametric approximations. Add `trimesh` to Dockerfile for future runs.

2. **Camera geometry derivation is complex**
   The 8_cm mount quaternion `(-0.5, -0.5, 0.5, -0.5)` requires careful transform chain computation.
   **Mitigation**: Use `_invert_transform()` from `publish_camera_mounts.py:78-88` as reference implementation. Validate with diagnostic output.

3. **Updated camera geometry may worsen results**
   Real downward-looking camera may produce more aggressive occlusion → lower success rates.
   **Mitigation**: This is desirable — more representative. If success rates drop too low, the test still provides valid data about system limitations.

4. **IDE threshold (Δ ≥ 10%) may still fail**
   Current mean Δ is +4.4%. Even with better geometry, may not reach 10%.
   **Mitigation**: Document honestly. MAR (>0%) is the minimum acceptable. IDE is aspirational.

5. **Figures not persisted from container**
   Previous figures were generated inside container but not visible on host.
   **Mitigation**: Ensure `tests/` directory is mounted as a volume in the Docker compose config, or copy results out after run.

---

## Alternative Approaches

1. **Keep existing camera geometry, document as "simplified"**: Less rigorous but avoids risk of introducing errors. Useful as a fallback if the real geometry derivation proves problematic.

2. **Load `camera_mounts.yaml` dynamically in `view_geometry.py`**: Instead of hardcoding, compute transforms from the YAML file. Ensures test always matches current mount. Trade-off: adds YAML dependency and complexity.

3. **Test with multiple mount configurations**: Run with 5_cm, 8_cm, 10_cm, 12_cm mounts to show camera placement effect. More compelling figures but 4× runtime.

---

## Key Files Reference

| File | Purpose | Key Lines |
|---|---|---|
| `tests/test1_software_verification/view_geometry.py` | Camera frustum definitions — **needs updating** | `view_geometry.py:22-43` |
| `tests/test1_software_verification/occlusion.py` | Frustum culling + depth buffer | `occlusion.py:97-188` |
| `tests/test1_software_verification/hand_approaches.py` | Per-object hand poses — **may need tuning** | `hand_approaches.py:18-110` |
| `tests/test1_software_verification/plot_results.py` | Figure generation — **needs restyling** | `plot_results.py:1-680` |
| `tests/test1_software_verification/run.py` | Main test orchestrator | `run.py:238-294` (latency), `run.py:301-469` (occlusion) |
| `tests/test1_software_verification/generate_objects.py` | Object generation — **must run before test** | `generate_objects.py:533-580` |
| `src/sensor_fusion_bringup/config/camera_mounts.yaml` | Real camera mount geometry | `camera_mounts.yaml:100-165` |
| `src/sensor_fusion_bringup/scripts/publish_camera_mounts.py` | Reference for transform computation | `publish_camera_mounts.py:78-88` (`_invert_transform`) |
| `config/grasp_preshaping.yaml` | Production grasp config | `grasp_preshaping.yaml:1-66` |
| `docker/Dockerfile` | Container build — **needs matplotlib/trimesh** | `Dockerfile:31-33` |
