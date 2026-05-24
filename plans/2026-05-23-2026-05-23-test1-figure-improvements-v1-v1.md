# Test 1: Figure Improvements & Analysis Enhancements

## Objective

Improve 5 specific aspects of the test 1 visualization suite based on user feedback, and add 2 new analytical figures (score-vs-samples sweep, object gallery). All changes maintain the project color scheme (`#5099e9`, `#fffdf6`, `#f4fbf9`).

---

## Implementation Plan

### Phase 1: Fix per-view coverage figure (fig7c)

- [ ] **1.1** Fix axis scaling in `plot_per_view_coverage()` — add `ax.set_box_aspect([1,1,1])` to all 3 subplots so X/Y/Z have equal unit scales. The current figure looks "squashed" because matplotlib auto-scales each axis independently. Set equal data ranges via `set_xlim/set_ylim/set_zlim` derived from the cloud bounds.
- [ ] **1.2** Improve overlap visibility — reduce point size to `s=0.3`, increase alpha for the "visible" categories (head-only, wrist-only, both) to `0.9`, and decrease alpha for "neither" to `0.15`. This makes overlapping regions easier to distinguish.
- [ ] **1.3** Add a 4th subplot showing the **combined multi-view** (union of head+wrist visible points in green, unseen in gray) to complete the story: "what each camera sees → what they see together".

### Phase 2: Fix 3D scene viewpoints and labels (fig9)

- [ ] **2.1** Rename viewpoints: current "Side View (from +Y)" → **"Side View (from +Y)"** stays, current "Rear View (from -X)" → rename to **"Side View (from -X)"**. The current "Side" label should become **"Rear View (from +Y)"** since that shows the back of the hand/object. The key insight from user: the wrist camera is not angled at the object in the rear view, so rename to match physical meaning.
- [ ] **2.2** Fix axis scaling — add `ax.set_box_aspect([ub_x, ub_y, ub_z])` to all 4 panels using the actual data bounds so the scene is not distorted. Compute unified bounds from the union of all point clouds + camera positions + frustum extents.

### Phase 3: Violin plot benchmark comparison (fig7d upgrade)

- [ ] **3.1** Increase baseline repetitions from 5 → **20** in `compute_baseline()` (`run.py:174`). The current baseline is a single consensus from 5 runs, which gives only 1 data point per object. With 20 runs, we get a distribution to compare against.
- [ ] **3.2** Modify `compute_baseline()` to return ALL successful results (not just the consensus) and save them to `results/baseline_all_results.csv` with one row per repetition. This provides the distribution data needed for the violin plot.
- [ ] **3.3** Rewrite `plot_benchmark_comparison()` to use `matplotlib.violinplot` or `seaborn.violinplot` showing three distributions per object: baseline (green), single-view (blue), multi-view (orange). Each violin shows the full score distribution from 20/100/100 repetitions respectively.
- [ ] **3.4** Add median markers and quartile boxes inside each violin for readability. Use project colors with transparency.

### Phase 4: Object gallery figure (new fig12)

- [ ] **4.1** Create `plot_object_gallery()` — a grid figure (4×4 or 3×5) showing a 3D rendering of each of the 13 objects as a point cloud. Each subplot shows one object from a consistent viewpoint (elev=25, azim=-45).
- [ ] **4.2** Color each object by its convexity: convex objects in blue (`#5099e9`), non-convex in orange (`#e67e22`). Label each subplot with the object name and point count.
- [ ] **4.3** Use equal axis scaling (`set_box_aspect`) so objects are not distorted. Set consistent axis limits based on the largest object dimensions.

### Phase 5: Score-vs-samples sweep figure (new fig13)

- [ ] **5.1** Create `run_score_sweep.py` (or add `--score-sweep` mode to `run.py`) that runs the grasp planner for a single representative object (e.g., `power_drill`) at multiple sample counts: `[1000, 2000, 5000, 10000, 20000, 50000, 100000]`. At each sample count, run 30 repetitions and record the best (max) score and mean score.
- [ ] **5.2** Accumulate results across ALL 13 objects by running the sweep for each object and pooling the scores. Save to `results/score_sweep_results.csv` with columns: `object, n_samples, repetition, combined_score, grasp_type_name, pipeline_time_ms`.
- [ ] **5.3** Create `plot_score_sweep()` figure showing two panels: (a) mean score vs n_samples with error bars, accumulated across all objects; (b) P95 latency vs n_samples. Overlay the baseline score as a horizontal dashed line. This answers: "how many samples are needed to approach baseline quality?"
- [ ] **5.4** Add a secondary y-axis or annotation showing the latency budget (MAR=400ms, IDE=100ms) to show the sample-latency tradeoff.

### Phase 6: Re-run and regenerate

- [ ] **6.1** Re-run the baseline computation with 20 repetitions: `python run.py --occlusion-only --baseline-timeout 600` (the increased baseline reps will take longer).
- [ ] **6.2** Run the score sweep: `python run_score_sweep.py` (estimated ~10-15 minutes for all objects × 7 sample counts × 30 reps).
- [ ] **6.3** Regenerate all figures: `python plot_results.py --format png --dpi 200`.
- [ ] **6.4** Verify all figures visually — check axis scaling, labels, colors, and data integrity.

---

## Verification Criteria

- [ ] `fig7c_per_view_coverage.png`: All 4 subplots have equal axis scaling (circles look circular, not elliptical). Overlapping regions are clearly distinguishable.
- [ ] `fig9_synthetic_setup_3d.png`: Viewpoints are correctly labeled (Side/Rear match physical direction). Axes are not squashed.
- [ ] `fig7d_benchmark_comparison.png`: Shows violin plots with 3 distributions per object. Baseline violins are visible (20 data points). Median markers are present.
- [ ] `fig12_object_gallery.png`: All 13 objects shown in a grid with equal axis scaling. Convex/non-convex color coding is correct.
- [ ] `fig13_score_sweep.png`: Shows score convergence curve with latency overlay. Baseline reference line is visible. The "knee" of the curve is identifiable.
- [ ] All figures use project colors (`#5099e9`, `#fffdf6`, `#f4fbf9`).

## Potential Risks and Mitigations

1. **Score sweep runtime**: 13 objects × 7 sample counts × 30 reps = 2,730 grasp planner calls. At ~60ms each ≈ 2.7 minutes. But 100K samples takes ~300ms → ~14 minutes total.
   - Mitigation: Run the sweep in parallel per object, or reduce to 5 representative objects.

2. **Baseline with 20 reps**: Each baseline call runs 20 repetitions at 100K samples in a subprocess. At ~300ms per rep × 20 reps = 6 seconds per object × 13 objects ≈ 78 seconds. Well within timeout.
   - Mitigation: Increase `--baseline-timeout` to 600s.

3. **Violin plot with few baseline points**: 20 baseline repetitions may produce a thin violin. Seaborn's `violinplot` can handle this but it may look sparse.
   - Mitigation: Use `inner="box"` to show quartile boxes, and overlay individual points with `sns.stripplot` for n<30.

4. **3D axis scaling**: `set_box_aspect` requires matplotlib ≥ 3.3. The current environment has matplotlib installed but version unknown.
   - Mitigation: Check matplotlib version first; fall back to manual axis limit equalization if `set_box_aspect` is unavailable.

## Alternative Approaches

1. **Score sweep**: Instead of a separate script, add a `--score-sweep` flag to `run.py` that overrides `prediction_samples` via a custom config. This keeps everything in one file but makes `run.py` more complex.
2. **Violin plots**: Use `seaborn.violinplot` (requires seaborn) vs manual `matplotlib.violinplot`. Seaborn produces better-looking plots but adds a dependency. Recommendation: use seaborn if available, fall back to matplotlib.
3. **Object gallery**: Could be a separate script (`plot_object_gallery.py`) instead of adding to `plot_results.py`. This keeps the figure pipeline modular.
