# Test 1 Figure Overhaul Plan

## Objective

Restructure and fix the Test 1 `plot_results.py` figure suite: remove unwanted figures, fix layout/labeling issues on kept figures, and add superquadric overlays to the object gallery.

## File

Single file: `tests/test1_software_verification/plot_results.py` (1943 lines)

## Figure Status Matrix

| Fig | Function | Lines | Status |
|-----|----------|-------|--------|
| 1 | `plot_latency_boxplot` | 72-114 | REMOVE |
| 2 | `plot_latency_summary` | 121-164 | REMOVE |
| 3 | `plot_intent_precision` | 171-231 | REMOVE |
| 4 | `plot_intent_delta` | 238-273 | REMOVE |
| 5 | `plot_wrist_error_cdf` | 280-371 | MODIFY (keep CDF only, remove grasp bar panel) |
| 6 | `plot_pose_error_scatter` | 378-411 | REMOVE |
| 7 | `plot_cloud_coverage` | 418-471 | REMOVE |
| 7b | `plot_convexity_analysis` | 478-566 | KEEP |
| 7c | `plot_per_view_coverage` | 573-734 | MODIFY (remove arrows, add legend, consistent colors) |
| 7d-bench | `plot_benchmark_comparison` | 741-877 | MODIFY (violin → boxplot, fix x-label overlap) |
| 7d-score | `plot_score_distribution` | 883-954 | REMOVE |
| 7e | `plot_best_score_by_type` | 961-1057 | MODIFY (remove "unknown" grasp type category) |
| 7f | `plot_score_vs_wrist_angle` | 1064-1143 | KEEP |
| 8 | `plot_tier_ab_latency` | 1150-1242 | REMOVE |
| 9 | `plot_synthetic_setup` | 1354-1540 | KEEP |
| 10 | `plot_per_stage_latency` | 1546-1613 | REMOVE |
| 11 | `plot_per_stage_waterfall` | 1630-1691 | MODIFY (combine PM+PM cloud → ROS overhead, fix x-labels) |
| 12 | `plot_object_gallery` | 1698-1778 | MODIFY (add SQ mesh overlays) |
| 13 | `plot_score_vs_samples` | 1785-1876 | KEEP |

---

## Implementation Tasks

### Task 1: Remove unwanted figures from `main()`

**File:** `plot_results.py:1908-1937`

Remove the following call lines from `main()`:

- [ ] Remove `plot_latency_boxplot(latency_rows, args.format, args.dpi)` (line 1909)
- [ ] Remove `plot_latency_summary(latency_rows, args.format, args.dpi)` (line 1910)
- [ ] Remove `plot_intent_precision(summary_rows, delta_rows, args.format, args.dpi)` (line 1911)
- [ ] Remove `plot_intent_delta(delta_rows, args.format, args.dpi)` (line 1912)
- [ ] Remove `plot_pose_error_scatter(occlusion_rows, args.format, args.dpi)` (line 1914)
- [ ] Remove `plot_cloud_coverage(occlusion_rows, args.format, args.dpi)` (line 1915)
- [ ] Remove `plot_score_distribution(occlusion_rows, args.format, args.dpi)` (line 1921)
- [ ] Remove `plot_tier_ab_latency(latency_rows, tier_b_rows, args.format, args.dpi)` (line 1925)
- [ ] Remove `plot_per_stage_latency(args.format, args.dpi)` (line 1926)

After removals, the remaining `main()` calls should be:

```
plot_wrist_error_cdf(...)          # Fig 5 (modified)
plot_convexity_analysis(...)       # Fig 7b (kept)
plot_per_view_coverage(...)        # Fig 7c (modified)
plot_benchmark_comparison(...)     # Fig 7d-bench (modified)
plot_best_score_by_type(...)       # Fig 7e (modified)
plot_score_vs_wrist_angle(...)     # Fig 7f (kept)
plot_per_stage_waterfall(...)      # Fig 11 (modified)
generate_latex_table(...)          # LaTeX table (kept)
plot_synthetic_setup(...)          # Fig 9 (kept)
plot_object_gallery(...)           # Fig 12 (modified)
plot_score_vs_samples(...)         # Fig 13 (kept)
```

Optionally delete the 9 unused function bodies (~400 lines) to keep the file clean.

---

### Task 2: Modify Figure 5 — Keep only the CDF panel

**File:** `plot_results.py:280-371` (`plot_wrist_error_cdf`)

Current layout: 2-panel figure — left = CDF, right = grasp correctness bar chart.

- [ ] Change subplot creation from `plt.subplots(1, 2, ...)` (line 294) to `plt.subplots(1, 1, figsize=(8, 5))`
- [ ] Remove the `gridspec_kw={"width_ratios": [1, 1.2]}` parameter
- [ ] Rename `ax_cdf` to just `ax` (the single axis)
- [ ] Delete the entire right-panel block (lines 325-367): the `if summary_rows:` block containing the grouped bar chart
- [ ] Update `fig.suptitle` (line 368) from `"Wrist Error Distribution & Grasp Correctness"` to `"Wrist Rotation Error CDF"`
- [ ] Update `_save_fig` name from `"fig5_wrist_cdf_grasp_correct"` to `"fig5_wrist_error_cdf"`
- [ ] Remove the now-unused `summary_rows` parameter from the function signature

---

### Task 3: Modify Figure 7c — Remove view arrows, add legend, consistent colors

**File:** `plot_results.py:573-734` (`plot_per_view_coverage`)

**3a. Remove camera position markers and quiver arrows from subplot 1:**

- [ ] Delete lines 673-678 (the `for label, cf in [("Head", ...), ("Wrist", ...)]:` loop that draws camera triangle markers and quiver arrows)
- [ ] Delete `ax1.legend(fontsize=7, loc="upper right")` (line 683)

**3b. Add a shared color legend to the figure:**

- [ ] After all 4 subplots are drawn (after line 729), add a figure-level legend using `matplotlib.patches.Patch`:
  ```python
  from matplotlib.patches import Patch
  legend_elements = [
      Patch(facecolor='#5099e9', label='Head camera only'),
      Patch(facecolor='#ff9e4a', label='Wrist camera only'),
      Patch(facecolor='#55a868', label='Both cameras'),
      Patch(facecolor='#b0b0b0', alpha=0.3, label='Unseen'),
  ]
  fig.legend(handles=legend_elements, loc='lower center', ncol=4, fontsize=9,
             framealpha=0.9, edgecolor='#cccccc')
  ```
- [ ] Adjust `fig.tight_layout(rect=[0, 0.06, 1, 0.95])` to leave room for the bottom legend

**3c. Make colors consistent across subplots:**

The color map `cmap` (lines 633-639) already uses consistent indices. The issue is that subplots 2/3 remap categories (e.g., subplot 2 shows wrist-only as gray). This is semantically correct (wrist-only points are "not visible from head"), but the head-only and both colors must match subplot 1.

- [ ] Verify that in each subplot, the colors for "head-only" (index 1), "wrist-only" (index 2), "both" (index 3), and "neither" (index 4) always use `cmap[1]`, `cmap[2]`, `cmap[3]`, `cmap[4]` respectively. The current remapping in subplots 2/3 changes the semantic meaning per panel but the physical colors remain consistent with cmap. No change needed here — just verify visually after running.

---

### Task 4: Modify Figure 7d-bench — Violin → Boxplot, fix x-label overlap

**File:** `plot_results.py:741-877` (`plot_benchmark_comparison`)

**4a. Replace violin plot with boxplot:**

- [ ] Replace the violin data preparation and plotting (lines 798-838) with boxplot logic:
  - Group data per object: for each object, collect `[baseline_scores, sv_scores, mv_scores]`
  - Use `ax.boxplot(data_groups, positions=positions, widths=2.8, patch_artist=True)`
  - Style each box with the same colors: `#8ecae6` (baseline), `COLORS["primary"]` (SV), `#55a868` (MV)
  - The boxplot natively shows median, quartiles, and whiskers — remove the separate median/quartile drawing code

**4b. Fix x-axis label overlap with convexity indicators:**

- [ ] Increase padding between x-labels and convexity dots:
  - Move convexity dots further down: change y-position from `-0.02` (line 844) to `-0.08` in axes transform
  - OR add `ax.tick_params(axis='x', pad=15)` to push labels down
- [ ] Add a small gap between the label text and the convexity dot

---

### Task 5: Modify Figure 7e — Remove "unknown" grasp type

**File:** `plot_results.py:961-1057` (`plot_best_score_by_type`)

- [ ] In the data collection loop (lines 977-982), add a filter after reading `gtype`:
  ```python
  gtype = row["grasp_type_name"]
  if gtype in ("unknown", "Unknown", ""):
      continue
  ```
  This goes right after line 981 (`gtype = row["grasp_type_name"]`).
- [ ] The `all_types` collection (lines 996-1000) will automatically exclude "unknown" since no rows with that type will be added to `data`.

---

### Task 6: Modify Figure 11 — Combine PM + PM Cloud → ROS overhead, fix x-labels

**File:** `plot_results.py:1630-1691` (`plot_per_stage_waterfall`)

**6a. Combine "Pipeline Manager" and "PM Cloud Handling" into "ROS Overhead":**

- [ ] In `stage_fields` (lines 1647-1653), replace the two PM entries with a single combined entry:
  ```python
  stage_fields = [
      ("ros_overhead_ms", "ROS Overhead", "#3498db"),
      ("twist_propagation_ms", "Twist Propagation", "#2ecc71"),
      ("segmentation_ms", "Segmentation", "#e67e22"),
      ("preshaping_ms", "Grasp Preshaping", "#e74c3c"),
  ]
  ```
- [ ] Before computing `stage_means` (line 1656), pre-compute the combined value by modifying `ok_rows`:
  ```python
  for r in ok_rows:
      pm = float(r.get("pipeline_manager_ms", 0) or 0)
      pc = float(r.get("pm_cloud_handling_ms", 0) or 0)
      r["ros_overhead_ms"] = str(pm + pc)
  ```
  Note: values are stored as strings from CSV, so store the combined value as a string too (or handle in the float conversion).

**6b. Fix x-axis label overlap:**

- [ ] Add rotation and alignment to the x-axis labels:
  ```python
  ax.tick_params(axis='x', rotation=25, labelsize=9)
  ```
  Or use:
  ```python
  for label in ax.get_xticklabels():
      label.set_ha('right')
      label.set_rotation(25)
  ```

---

### Task 7: Modify Figure 12 — Add superquadric estimation overlays

**File:** `plot_results.py:1698-1778` (`plot_object_gallery`)

This is the largest change. The SQ parameters are NOT available through the FFI response (`ffi_bridge.py:301-328` — no SQ fields). They are only available in debug dump NPZ files. The approach: run a single grasp computation with debug visualization enabled to generate dumps, then load the SQ params from those dumps.

**Better approach:** Since the debug dumps may not exist for all objects, we should call the Rust library directly within `plot_object_gallery` to compute SQ fits. But the SQ params are internal to the Rust pipeline and not exposed through the FFI.

**Practical approach:** Use the existing debug dump NPZ files if they exist. If not, generate them by running the pipeline once per object with debug enabled. Alternatively, we can reconstruct the SQ from the point cloud using the same template-fitting approach.

**Simplest viable approach:** For the gallery figure, we can approximate the SQ overlay by fitting simple bounding primitives (sphere, box, cylinder) to each point cloud, matching the three templates used by the Rust code. This avoids the FFI dependency entirely.

**However**, the most correct approach is to load from debug dumps. Let me outline both:

#### Option A: Load from debug dumps (preferred if dumps exist)

- [ ] Check if `results/debug_dumps/` contains NPZ files for each object
- [ ] For each object in the gallery, try to load `{object_name}_*.npz` from the debug dumps directory
- [ ] Parse `sq_params` and `sq_meta` using the same pattern as `report/generate_figures.py:70-88`:
  ```python
  sq_params = None
  if "sq_params" in data:
      raw = data["sq_params"]
      if len(raw) == 14:
          sq_params = {
              "epsilon1": float(raw[0]), "epsilon2": float(raw[1]),
              "a": float(raw[2]), "b": float(raw[3]), "c": float(raw[4]),
              "translation": raw[5:8],
              "rotation": raw[8:14].reshape(2, 3),
          }
  ```
- [ ] Generate SQ surface mesh using parametric sampling (see helper below)
- [ ] Plot as `ax.plot_wireframe(x*100, y*100, z*100, ...)` overlay

#### Option B: Call FFI to generate SQ fits (if dumps don't exist)

- [ ] Import `ffi_bridge` and run one compute call per object with debug enabled
- [ ] The debug dump NPZ will be written to `results/debug_dumps/`
- [ ] Then load the SQ params from the dump as in Option A

#### SQ mesh generation helper (needed for both options):

- [ ] Add a helper function to generate SQ surface points from parameters:
  ```python
  def _generate_sq_surface(sq_params, n_eta=30, n_omega=30):
      """Generate superquadric surface mesh from fitted parameters."""
      e1 = sq_params["epsilon1"]
      e2 = sq_params["epsilon2"]
      a, b, c = sq_params["a"], sq_params["b"], sq_params["c"]
      
      eta = np.linspace(-np.pi/2, np.pi/2, n_eta)
      omega = np.linspace(-np.pi, np.pi, n_omega)
      E, O = np.meshgrid(eta, omega)
      
      # Parametric superquadric equations
      x = a * np.sign(np.cos(E)) * np.abs(np.cos(E))**e1 * np.sign(np.cos(O)) * np.abs(np.cos(O))**e2
      y = b * np.sign(np.cos(E)) * np.abs(np.cos(E))**e1 * np.sign(np.sin(O)) * np.abs(np.sin(O))**e2
      z = c * np.sign(np.sin(E)) * np.abs(np.sin(E))**e1
      
      # Apply rotation
      rot_flat = sq_params["rotation"]
      rot = np.eye(3)
      rot[0, :] = rot_flat[0]
      rot[1, :] = rot_flat[1]
      rot[2, :] = np.cross(rot[0, :], rot[1, :])
      
      t = sq_params["translation"]
      shape = x.shape
      pts = np.stack([x.ravel(), y.ravel(), z.ravel()], axis=0)  # (3, N)
      pts = rot @ pts + t.reshape(3, 1)
      
      return pts[0].reshape(shape), pts[1].reshape(shape), pts[2].reshape(shape)
  ```

- [ ] In the gallery loop, after plotting the point cloud, add:
  ```python
  sq = _try_load_sq_params(obj_name)  # helper to find & load dump
  if sq is not None:
      xs, ys, zs = _generate_sq_surface(sq)
      ax.plot_wireframe(xs*100, ys*100, zs*100, color='red', alpha=0.4,
                        linewidth=0.5, rstride=2, cstride=2)
  ```

- [ ] Add fallback: if no SQ data is available for an object, just show the point cloud (current behavior) with a printed warning.

**Reference files for SQ loading:**
- `report/generate_figures.py:70-88` — SQ param parsing from NPZ
- `src/grasp_preshaping/scripts/visualize_grasp_debug.py:530-600` — SQ primitive mesh building (PyVista, but logic is reusable)
- `src/grasp_preshaping/src/superquadric.rs:16-36` — Rust `SuperquadricParams` struct definition
- `src/grasp_preshaping/src/debug_export.rs:42-55` — DebugDump struct with `sq_params` field

---

### Task 8: Clean up `main()` — remove unused data loading

After removing the figure calls, some CSV loading may become unnecessary:

- [ ] `latency_rows` is still needed by `generate_latex_table()` — keep it
- [ ] `tier_b_rows` is no longer used (Fig 8 removed) — can remove the load line (line 1898)
- [ ] `per_stage_rows` is no longer used by `main()` directly (Fig 10 removed) — but `plot_per_stage_waterfall` loads its own data internally — can remove the load line (line 1899) and the print line (line 1906)

---

### Task 9 (optional): Delete unused function bodies

After confirming everything works, delete these function definitions to reduce file size by ~400 lines:

- [ ] `plot_latency_boxplot` (lines 72-114)
- [ ] `plot_latency_summary` (lines 121-164)
- [ ] `plot_intent_precision` (lines 171-231)
- [ ] `plot_intent_delta` (lines 238-273)
- [ ] `plot_pose_error_scatter` (lines 378-411)
- [ ] `plot_cloud_coverage` (lines 418-471)
- [ ] `plot_score_distribution` (lines 883-954)
- [ ] `plot_tier_ab_latency` (lines 1150-1242)
- [ ] `plot_per_stage_latency` (lines 1546-1613)

---

## Verification Criteria

- [ ] `python plot_results.py --format png --dpi 300` runs without errors
- [ ] Only the kept figures are generated in `figures/`
- [ ] Fig 5 is a single CDF panel (no bar chart)
- [ ] Fig 7c has no camera arrows, has a shared 4-color legend at the bottom
- [ ] Fig 7d-bench uses boxplots (not violins), x-labels don't overlap convexity dots
- [ ] Fig 7e has no "unknown" grasp type bars
- [ ] Fig 11 has "ROS Overhead" (combined PM + PM Cloud), x-labels are readable
- [ ] Fig 12 shows SQ wireframe overlays (red) on point clouds where data is available
- [ ] Fig 9 (synthetic setup) and Fig 13 (score vs samples) are unchanged

## Risks

1. **SQ data may not exist in debug dumps**: If `run.py` was never executed with `--debug`, no dumps exist. Mitigation: the gallery gracefully falls back to point-cloud-only display with a printed warning.
2. **SQ mesh generation with extreme epsilon values**: Epsilon near 0 (box template) can cause numerical issues with `abs(cos)**epsilon`. Mitigation: clamp epsilon to a minimum of 0.05.
3. **Boxplot positioning in 7d-bench**: The current violin uses custom positions with 4-unit spacing. Boxplot needs the same grouping. Mitigation: use `ax.boxplot()` with `positions` parameter matching the current spacing.
4. **Color consistency in 7c**: The remapping logic is subtle. Mitigation: test visually after running with a known object.
