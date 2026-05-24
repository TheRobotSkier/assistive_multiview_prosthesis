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

## Implementation Tasks

### Task 1: Remove unwanted figures from `main()`

**File:** `plot_results.py:1908-1937`

Remove the following call lines from `main()`:

- [ ] Remove `plot_latency_boxplot(...)` (line 1909)
- [ ] Remove `plot_latency_summary(...)` (line 1910)
- [ ] Remove `plot_intent_precision(...)` (line 1911)
- [ ] Remove `plot_intent_delta(...)` (line 1912)
- [ ] Remove `plot_pose_error_scatter(...)` (line 1914)
- [ ] Remove `plot_cloud_coverage(...)` (line 1915)
- [ ] Remove `plot_score_distribution(...)` (line 1921)
- [ ] Remove `plot_tier_ab_latency(...)` (line 1925)
- [ ] Remove `plot_per_stage_latency(...)` (line 1926)

Optionally delete the function bodies too (to keep the file clean), but the calls must go regardless.

### Task 2: Modify Figure 5 — Keep only the CDF panel

**File:** `plot_results.py:280-371` (`plot_wrist_error_cdf`)

Current: 2-panel figure — left = CDF, right = grasp correctness bar chart.

- [ ] Change from `plt.subplots(1, 2, ...)` to `plt.subplots(1, 1, ...)` (single panel)
- [ ] Remove the entire right-panel block (lines 325-367) — the `if summary_rows:` block with the grouped bar chart
- [ ] Remove `ax_bar` from the subplot creation (line 294)
- [ ] Adjust `gridspec_kw` removal (no longer needed)
- [ ] Update figure size from `(14, 5)` to something like `(8, 5)` for single panel
- [ ] Update `fig.suptitle` to reflect CDF-only content (line 368)
- [ ] Update `_save_fig` name if desired (currently `fig5_wrist_cdf_grasp_correct` → `fig5_wrist_error_cdf`)

### Task 3: Modify Figure 7c — Remove view arrows, add legend, consistent colors

**File:** `plot_results.py:573-734` (`plot_per_view_coverage`)

**3a. Remove camera position markers and quiver arrows from subplot 1:**

- [ ] Remove lines 673-678 (the `for label, cf in [("Head", ...), ("Wrist", ...)]:` loop that draws camera markers and quiver arrows in subplot 1)
- [ ] Remove the `ax1.legend(...)` call on line 683

**3b. Add a shared color legend:**

- [ ] Add a `fig.legend()` (or `fig.text()`) below the figure explaining the 4 colors:
  - Blue = Head camera only
  - Orange = Wrist camera only
  - Green = Both cameras
  - Gray = Unseen
- [ ] Use `matplotlib.patches.Patch` for the legend entries
- [ ] Place it at the bottom of the figure using `fig.legend(loc='lower center', ncol=4, ...)`
- [ ] Adjust `fig.tight_layout(rect=[0, 0.05, 1, 0.95])` to leave room

**3c. Make colors consistent across subplots:**

The color map `cmap` (lines 633-639) already uses the same 4 colors. The issue is that subplots 2/3/4 remap categories (e.g., subplot 2 maps wrist-only → gray). The fix:

- [ ] In each subplot, use the SAME color for each category as in subplot 1. Subplot 2 ("Head Camera Visible") currently remaps wrist-only to gray — that's correct semantically but the head-only and both colors must match subplot 1. Verify `cmap` indices are used consistently.
- [ ] Specifically: ensure `head_labels` (line 689-692) and `wrist_labels` (line 704-706) and `mv_labels` (line 719-721) all use the same cmap indices for the same semantic meaning. Currently they do, but verify.

### Task 4: Modify Figure 7d-bench — Violin → Boxplot, fix x-label overlap

**File:** `plot_results.py:741-877` (`plot_benchmark_comparison`)

**4a. Replace violin plot with boxplot:**

- [ ] Replace `ax.violinplot(...)` (line 815) with `ax.boxplot(...)`:
  - Group data by object: for each object, create a list of 3 datasets `[baseline, sv, mv]`
  - Use `positions` array for x-positioning (same spacing as current)
  - Use `patch_artist=True` for colored fills
  - Set box face colors to match current scheme: `#8ecae6` (baseline), `COLORS["primary"]` (SV), `#55a868` (MV)
- [ ] Remove the violin body styling loop (lines 819-825)
- [ ] Remove the separate median/quartile drawing (lines 828-838) — boxplot handles this natively

**4b. Fix x-axis label overlap with convexity indicators:**

- [ ] Move x-axis tick labels down by adding padding: `ax.tick_params(axis='x', pad=10)` or similar
- [ ] Alternatively, move the convexity dots further down: change the y-position from `-0.02` (line 844) to something like `-0.06` in axes transform coordinates
- [ ] Or use a separate annotation row below the labels for convexity dots

### Task 5: Modify Figure 7e — Remove "unknown" grasp type

**File:** `plot_results.py:961-1057` (`plot_best_score_by_type`)

- [ ] After line 982 (where `gtype` is read from the row), add a filter to skip rows where `gtype` is `"unknown"` or unrecognizable:
  ```python
  if gtype in ("unknown", "Unknown", ""):
      continue
  ```
- [ ] This filters out any trial where the grasp type wasn't identified, preventing "unknown" bars from appearing
- [ ] Verify the `all_types` collection (lines 996-1000) also excludes "unknown"

### Task 6: Modify Figure 11 — Combine PM + PM Cloud → ROS overhead, fix x-labels

**File:** `plot_results.py:1630-1691` (`plot_per_stage_waterfall`)

**6a. Combine "Pipeline Manager" and "PM Cloud Handling" into "ROS Overhead":**

- [ ] In `stage_fields` (lines 1647-1653), replace:
  ```python
  ("pipeline_manager_ms", "Pipeline Manager", "#3498db"),
  ...
  ("pm_cloud_handling_ms", "PM Cloud Handling", "#9b59b6"),
  ```
  with a single combined entry:
  ```python
  ("ros_overhead_ms", "ROS Overhead", "#3498db"),
  ```
- [ ] Before computing `stage_means`, pre-compute the combined value:
  ```python
  for r in ok_rows:
      pm = float(r.get("pipeline_manager_ms", 0) or 0)
      pc = float(r.get("pm_cloud_handling_ms", 0) or 0)
      r["ros_overhead_ms"] = pm + pc
  ```
- [ ] Keep the other stages unchanged (Twist Propagation, Segmentation, Grasp Preshaping)

**6b. Fix x-axis label overlap:**

- [ ] Add rotation and padding to the x-axis labels: `ax.tick_params(axis='x', rotation=30, labelsize=9)`
- [ ] Or use `ha='right'` alignment with rotation

### Task 7: Modify Figure 12 — Add superquadric estimation overlays

**File:** `plot_results.py:1698-1778` (`plot_object_gallery`)

This is the largest change. The goal is to overlay the fitted superquadric (SQ) mesh on each object's point cloud.

**7a. Determine SQ data source:**

The SQ estimation is done by the Rust grasp preshaping library. We need to either:
- Load pre-computed SQ parameters from debug dumps, OR
- Run SQ fitting inline (calling the Rust library via `ffi_bridge`)

The most practical approach: load SQ params from the baseline debug dumps if available, or call the library directly.

- [ ] Add import of `ffi_bridge.GraspLibrary`, `make_pose`, `make_twist`, `make_request` at the top of the function
- [ ] For each object in the gallery:
  1. Load the object point cloud (already done)
  2. Get the approach pose (`hand_approaches.get_approach`)
  3. Call `lib.compute(req)` to get the response, which includes SQ parameters (superquadric axes and shape parameters)
  4. Extract SQ parameters from the response
  5. Generate an SQ mesh surface (parametric sampling)
  6. Plot as a semi-transparent wireframe or mesh overlay on the point cloud

**7b. SQ mesh generation helper:**

- [ ] Add a helper function `_generate_sq_mesh(a, b, c, e1, e2, center, n_points=2000)` that samples the superquadric surface:
  ```python
  def _generate_sq_mesh(a, b, c, e1, e2, n=2000):
      eta = np.linspace(-np.pi/2, np.pi/2, int(np.sqrt(n)))
      omega = np.linspace(-np.pi, np.pi, int(np.sqrt(n)))
      E, O = np.meshgrid(eta, omega)
      x = a * np.sign(np.cos(E)) * np.abs(np.cos(E))**e1 * np.sign(np.cos(O)) * np.abs(np.cos(O))**e2
      y = b * np.sign(np.cos(E)) * np.abs(np.cos(E))**e1 * np.sign(np.sin(O)) * np.abs(np.sin(O))**e2
      z = c * np.sign(np.sin(E)) * np.abs(np.sin(E))**e1
      return x, y, z
  ```
- [ ] Plot using `ax.plot_wireframe(x*100, y*100, z*100, color='red', alpha=0.3, linewidth=0.3)`

**7c. Check what SQ fields are available in the FFI response:**

- [ ] Verify which fields in `response_to_dict()` contain SQ parameters. Search `ffi_bridge.py` for superquadric-related fields (likely `sq_a`, `sq_b`, `sq_c`, `sq_e1`, `sq_e2` or similar).
- [ ] If SQ params aren't in the response, check the debug dump files in `results/debug_dumps/` for SQ data.

**7d. Fallback if SQ data unavailable:**

- [ ] If SQ fitting fails for an object, just show the point cloud without overlay (current behavior)
- [ ] Print a warning: `"  WARNING: No SQ params for {obj_name}, showing point cloud only"`

### Task 8: Clean up `main()` call order

**File:** `plot_results.py:1908-1937`

After all removals and modifications, the final `main()` should call (in order):

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

- [ ] Remove all deleted figure calls
- [ ] Ensure variable dependencies still work (e.g., `latency_rows` is only needed by `generate_latex_table` now)

### Task 9 (optional): Delete unused function bodies

After confirming everything works:

- [ ] Delete function bodies for: `plot_latency_boxplot`, `plot_latency_summary`, `plot_intent_precision`, `plot_intent_delta`, `plot_pose_error_scatter`, `plot_cloud_coverage`, `plot_score_distribution`, `plot_tier_ab_latency`, `plot_per_stage_latency`
- [ ] This reduces file size by ~400 lines

## Verification Criteria

- [ ] Running `python plot_results.py --format png --dpi 300` produces exactly the kept figures with no errors
- [ ] Fig 5 is a single-panel CDF (no bar chart)
- [ ] Fig 7c has no camera arrows, has a shared color legend, and colors are consistent across all 4 subplots
- [ ] Fig 7d-bench uses boxplots (not violins), x-labels don't overlap convexity dots
- [ ] Fig 7e has no "unknown" grasp type bars
- [ ] Fig 11 has "ROS Overhead" instead of separate PM + PM Cloud, x-labels are readable
- [ ] Fig 12 shows SQ wireframe overlays on point clouds (or gracefully falls back)
- [ ] No removed figure functions are called from `main()`

## Risks

1. **SQ params not in FFI response**: The Rust library may not expose SQ parameters through the FFI bridge. Mitigation: check `ffi_bridge.py` and the response struct first; if unavailable, read from debug dumps or skip the overlay with a warning.
2. **SQ mesh generation performance**: Generating meshes for 12+ objects could be slow. Mitigation: use low-resolution meshes (500-1000 points) for the gallery.
3. **Color consistency in 7c**: The current remapping logic is subtle. Mitigation: test with a known object and verify colors visually.
