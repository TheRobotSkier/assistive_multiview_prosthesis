# Test 1 Figure Cleanup Plan

## Objective

Clean up the figures in `tests/test1_software_verification/plot_results.py` to address visual issues and remove three objects (`thin_plate`, `notched_box`, `l_block`) from the test suite entirely. This affects figure generation code, object configuration, and test pipeline code. After code changes, the tests need to be re-run to regenerate CSV data.

## Current Figure Numbering

The current numbering has gaps from removed figures:

| Current # | Name | Status |
|-----------|------|--------|
| 5 | `fig5_wrist_error_cdf` | Active |
| 7b | `fig7b_convexity_analysis` | Active |
| 7c | `fig7c_per_view_coverage` | Active |
| 7d | `fig7d_benchmark_comparison` | Active |
| 7e | `fig7e_best_score_by_type` | Active |
| 7f | `fig7f_score_vs_wrist_angle` | Active |
| 9 | `fig9_synthetic_setup_3d` | Active |
| 11 | `fig11_latency_waterfall` | Active |
| 12 | `fig12_object_gallery` | Active |
| 13 | `fig13_score_vs_samples` | Active |

## Proposed Renumbering

Renumber sequentially 1–10:

| New # | Old # | New Filename |
|-------|-------|-------------|
| 1 | 5 | `fig1_wrist_error_cdf` |
| 2 | 7b | `fig2_convexity_analysis` |
| 3 | 7c | `fig3_per_view_coverage` |
| 4 | 7d | `fig4_benchmark_comparison` |
| 5 | 7e | `fig5_best_score_by_type` |
| 6 | 7f | `fig6_score_vs_wrist_angle` |
| 7 | 9 | `fig7_synthetic_setup_3d` |
| 8 | 11 | `fig8_latency_waterfall` |
| 9 | 12 | `fig9_object_gallery` |
| 10 | 13 | `fig10_score_vs_samples` |

## Implementation Plan

### Phase 1: Remove Objects from Test Suite

- [ ] **1.1** Remove `thin_plate`, `notched_box`, and `l_block` from `generate_objects.py:686-698` (`PARAMETRIC_GENERATORS` list). Remove the generator functions `gen_l_block` (lines 195-246), `gen_thin_plate` (lines 270-305), and `gen_notched_box` (lines 359-415) — or simply remove them from the `PARAMETRIC_GENERATORS` list and optionally leave the functions as dead code for reference.

- [ ] **1.2** Remove the three objects from `object_registry.py:26-42` (`CONVEXITY` dict). Remove entries for `"l_block"`, `"notched_box"`, `"thin_plate"` at lines 34, 35, 37.

- [ ] **1.3** Remove the three objects from `hand_approaches.py:44-98` (`APPROACHES` dict). Remove entries for `"l_block"` (line 62-65), `"thin_plate"` (line 70-73), `"notched_box"` (line 76-79).

- [ ] **1.4** Remove `l_block` and `thin_plate` from `run_tier_b.py:487-493` (`default_objects` list). The remaining default objects will be: `cylinder_upright`, `tapered_bottle`, `small_cube`.

- [ ] **1.5** Delete the `.npz` files for the three objects from `tests/test1_software_verification/objects/parametric/`: `l_block.npz`, `thin_plate.npz`, `notched_box.npz`.

- [ ] **1.6** Note: `run.py` uses `list_objects()` from the registry (auto-discovers `.npz` files), so no changes needed there — removing the files is sufficient.

### Phase 2: Figure-Specific Fixes

#### Fig7c → New Fig3: Per-view coverage — single view, transparent colors

- [ ] **2.1** Modify `plot_per_view_coverage()` (`plot_results.py:215-378`) to show only **one subplot** instead of four (2x2 grid). The single view should show the object with three categories: **head-only** (blue), **wrist-only** (orange), and **unseen** (gray). Remove the "both" category from the color map or merge it into head/wrist visibility. The key requirement is that overlapping regions are visible, so colors should be semi-transparent (alpha ~0.5-0.6).

- [ ] **2.2** Reduce the figure from a 2x2 grid to a single subplot (`figsize` ~ `(8, 7)`). Keep the 3D projection, the legend, and the axis labels. Choose a good viewing angle (e.g., the current "combined" view at `elev=25, azim=-55`).

- [ ] **2.3** Adjust the color map: head-only blue at alpha ~0.55, wrist-only orange at alpha ~0.55, unseen gray at alpha ~0.15. This makes overlap regions clearly visible as blended colors.

#### Fig7d → New Fig4: Narrower boxplot boxes, fix convexity/x-axis overlap

- [ ] **2.4** In `plot_benchmark_comparison()` (`plot_results.py:384-509`), reduce the `widths` parameter of `ax.boxplot()` from `2.4` to approximately `1.8` (or calculate dynamically based on the number of objects). The current spacing uses `pos = i * 4 + j` with `widths=2.4`, which causes boxes within each group to overlap.

- [ ] **2.5** Move the convexity dot markers further down below the x-axis labels. Currently at `y=-0.08` using `ax.get_xaxis_transform()` (line 475). Increase the offset to approximately `y=-0.15` or lower. Alternatively, increase `ax.tick_params(axis='x', pad=...)` from `pad=15` to `pad=25` to push labels down, giving more room for the convexity markers between the labels and the dots.

#### Fig7e → New Fig5: Move legend down, remove plus signs

- [ ] **2.6** In `plot_best_score_by_type()` (`plot_results.py:515-610`), change the legend position for the first subplot from `loc="upper left"` (line 603) to `loc="lower left"` or `loc="upper right"` — whichever avoids covering the data. The user wants the legend moved so it does not cover the top of the bars.

- [ ] **2.7** Remove the "+" annotations that highlight where MV beats SV. Delete the annotation block at lines 587-593 (the `if mv_best > sv_best:` block with `ax.annotate("+", ...)`).

#### Fig9 → New Fig7: Remove view-based coloring, single color

- [ ] **2.8** In `plot_synthetic_setup()` (`plot_results.py:803-987`), change the point cloud rendering to use a single color instead of separate head-visible (blue) and wrist-only (orange) colors. In `_draw_scene()` (lines 903-950), replace the separate `head_cloud` scatter (blue, line 910) and `wrist_only` scatter (orange, line 916) with a single scatter of all visible points in one neutral color (e.g., a medium gray or the project blue `COLORS["primary"]`). Keep the full cloud as a faint background (`#cccccc`).

#### Fig12 → New Fig9: 3x3 grid, remove three objects, SQ instead of OBB

- [ ] **2.9** In `plot_object_gallery()` (`plot_results.py:1266-1383`), change the grid from `ncols=5` to `ncols=3` (which gives `nrows = ceil(9/3) = 3` for a 3x3 grid). The three removed objects (`thin_plate`, `notched_box`, `l_block`) will automatically be excluded since their `.npz` files are deleted and `list_objects()` won't find them.

- [ ] **2.10** Replace the OBB fallback with SQ-only rendering. Currently, when no SQ debug dump matches, `_draw_obb_wireframe()` is called (lines 1324-1327). Change this so that if no SQ match is found, no wireframe overlay is drawn at all (just show the point cloud). The SQ overlay (`_generate_sq_surface`) remains when debug dumps are available.

- [ ] **2.11** Update the legend to remove the "OBB (PCA)" entry. Only show "SQ fit" when SQ data is available. Remove the unconditional OBB legend entry at lines 1377-1378.

- [ ] **2.12** Update the summary print at lines 1360-1364 to reflect the new behavior (no OBB fallback mention).

### Phase 3: Renumber All Figures

- [ ] **3.1** Renumber `fig5_wrist_error_cdf` → `fig1_wrist_error_cdf` in `_save_fig()` call at `plot_results.py:115` and the function comment at line 69.

- [ ] **3.2** Renumber `fig7b_convexity_analysis` → `fig2_convexity_analysis` in `_save_fig()` call at `plot_results.py:209`, skip message at line 135, and section comment at line 120.

- [ ] **3.3** Renumber `fig7c_per_view_coverage` → `fig3_per_view_coverage` in `_save_fig()` call at `plot_results.py:378` and section comment at line 212.

- [ ] **3.4** Renumber `fig7d_benchmark_comparison` → `fig4_benchmark_comparison` in save path at `plot_results.py:506`, skip message at line 433, and section comment at line 381.

- [ ] **3.5** Renumber `fig7e_best_score_by_type` → `fig5_best_score_by_type` in `_save_fig()` call at `plot_results.py:610` and section comment at line 512.

- [ ] **3.6** Renumber `fig7f_score_vs_wrist_angle` → `fig6_score_vs_wrist_angle` in `_save_fig()` call at `plot_results.py:692` and section comment at line 614.

- [ ] **3.7** Renumber `fig9_synthetic_setup_3d` → `fig7_synthetic_setup_3d` in `_save_fig()` call at `plot_results.py:987` and section comment at line 800.

- [ ] **3.8** Renumber `fig11_latency_waterfall` → `fig8_latency_waterfall` in `_save_fig()` call at `plot_results.py:1065` and section comment at line 991.

- [ ] **3.9** Renumber `fig12_object_gallery` → `fig9_object_gallery` in `_save_fig()` call at `plot_results.py:1383` and section comment at line 1068.

- [ ] **3.10** Renumber `fig13_score_vs_samples` → `fig10_score_vs_samples` in `_save_fig()` call at `plot_results.py:1478`, skip messages at lines 1399/1407, and section comment at line 1386.

- [ ] **3.11** Update the `main()` function comments at lines 1505-1525 to reflect new figure numbers.

### Phase 4: Re-run Tests and Verify

- [ ] **4.1** Regenerate objects: `python tests/test1_software_verification/generate_objects.py --all`

- [ ] **4.2** Re-run Tier A tests: `python tests/test1_software_verification/run.py` (this will regenerate all CSV files without the removed objects)

- [ ] **4.3** Re-generate all figures: `python tests/test1_software_verification/plot_results.py`

- [ ] **4.4** Visually inspect all 10 figures to confirm changes are correct

## Verification Criteria

- [ ] Fig3 (per-view coverage) shows a single 3D view with head-only, wrist-only, and unseen categories using transparent colors where overlap is visible
- [ ] Fig4 (benchmark) has narrower box widths with no box overlap within groups; convexity markers are clearly separated from x-axis labels
- [ ] Fig5 (best score by type) has legend positioned to not cover data; no "+" annotations present
- [ ] Fig7 (synthetic setup) uses a single color for all visible points (no view-based coloring)
- [ ] Fig9 (object gallery) is a 3x3 grid with only 9 objects (no thin_plate, notched_box, l_block); shows SQ overlays where available, no OBB fallback
- [ ] All figures are numbered sequentially 1–10 with no gaps
- [ ] All figure filenames match the new numbering scheme
- [ ] The three removed objects do not appear in any figure or test result

## Potential Risks and Mitigations

1. **Stale CSV data**: After removing objects, the existing CSV files in `results/` still contain rows for `thin_plate`, `notched_box`, `l_block`. The plotting functions iterate over whatever is in the CSV, so they would try to plot removed objects.
   - Mitigation: Re-running `run.py` regenerates all CSVs. Alternatively, add a filter in `plot_results.py` to skip removed objects.

2. **SQ debug dumps may reference removed objects**: The `_load_sq_cache()` function scans for `.npz` debug dumps and matches by centroid. If old debug dumps for removed objects exist, they would simply not match any gallery object and be ignored — no issue.

3. **`mug_with_handle` not in PARAMETRIC_GENERATORS**: The `plot_synthetic_setup()` function tries to load `mug_with_handle` first (line 835), falling back to `cylinder_upright`. Since `mug_with_handle` was removed from generators (line 695 comment), it may not exist as an `.npz` file. This is a pre-existing issue.
   - Mitigation: No change needed — the fallback to `cylinder_upright` handles this. Could optionally update the preference to `mug` (YCB) or `power_drill` instead.

4. **Figure numbering references in report**: If the report text (e.g., `report.md`, LaTeX documents) references old figure numbers like "Figure 7c" or "Figure 12", those references would become stale.
   - Mitigation: The user should update report references after renumbering. Flag this in the plan.

5. **Tier B default objects reduced to 3**: After removing `l_block` and `thin_plate`, the Tier B default set has only 3 objects (`cylinder_upright`, `tapered_bottle`, `small_cube`). This is a smaller but still representative set covering convex objects.
   - Mitigation: Consider adding `cross_shape` or a YCB object to maintain diversity.

## Alternative Approaches

1. **Keep objects, just exclude from figures**: Instead of removing objects from the test suite entirely, only filter them out in `plot_results.py`. This avoids needing to re-run tests.
   - Trade-off: Less thorough cleanup; the objects still exist in the codebase and could be accidentally used.

2. **Add an object exclusion list**: Create a centralized `EXCLUDED_OBJECTS` list in `object_registry.py` that all consumers reference, rather than deleting entries from each file.
   - Trade-off: More maintainable long-term, but adds indirection. The user's request is to remove them entirely.

3. **Keep OBB as fallback but make it optional**: Instead of removing OBB entirely, add a flag `--sq-only` to `plot_results.py`.
   - Trade-off: More flexible, but adds complexity. The user's intent seems to be SQ-only.

## Remaining Object Inventory (After Removal)

After removing `thin_plate`, `notched_box`, `l_block`, the test suite will have 9 objects:

| Object | Type | Convexity | Expected Grasp |
|--------|------|-----------|---------------|
| cylinder_upright | Parametric | convex | cylindrical |
| cylinder_tilted | Parametric | convex | cylindrical |
| ellipsoid | Parametric | convex | cylindrical |
| tapered_bottle | Parametric | convex | cylindrical |
| small_cube | Parametric | convex | pinch |
| cross_shape | Parametric | non-convex | cylindrical |
| banana | YCB | non-convex | cylindrical |
| mug | YCB | non-convex | cylindrical |
| power_drill | YCB | non-convex | cylindrical |

This gives 5 convex + 4 non-convex objects, which is a reasonable distribution for demonstrating multi-view advantages.
