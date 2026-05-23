# Test 1 Score Improvement Plan

## Objective

Fix the root cause of low grasp scores in Test 1 by extending the proximity/distance signal beyond the 2cm TSDF truncation band, using the already-fitted superquadric as a global distance proxy. Also fix Tier 3 scoring to provide directional gradient, and create the benchmark comparison figure.

## Background

### Current Architecture

1. **TSDF** (`pointcloud_helper.rs`): 5mm resolution, 4-cell truncation = **2cm band** around the surface. Beyond 2cm, `tsdf.get_distance()` returns `f32::MAX` (no information).

2. **Superquadric** (`superquadric.rs`): Fitted to the point cloud via PCA + Gauss-Newton (`c_api.rs:399-404`). Has `taubin_distance()` which provides a signed distance approximation **everywhere** — not limited to 2cm. Currently only used for TSDF backside sign correction and distance blending.

3. **Scoring** (`planner.rs`):
   - **Tier 4** (no collision, far from object): `contact_score = 0.0–0.05` (proximity via TSDF only, zero beyond 2cm)
   - **Tier 3** (hand inside object): `contact_score = 0.1` (flat, no gradient)
   - **Tier 2** (collision but missing fingers): `contact_score = 0.25–0.5`
   - **Tier 1** (valid grasp): `contact_score = 0.8–1.0`

4. **The problem**: From 25cm approach distance, most SMC particles land in Tier 4 with `contact_score ≈ 0.0` because the TSDF has no gradient beyond 2cm. The optimizer is essentially blind — it can only find the object by random chance.

### The Key Insight

The superquadric is **already fitted** but **not passed to the scorer**. It provides:
- `evaluate(world_pt)` → F value (0 = surface, <0 = inside, >0 = outside)
- `taubin_distance(world_pt)` → signed distance approximation (works globally)
- `is_inside(world_pt)` → boolean inside check

We need to thread `sq_params` through to `score_grasp` and use it as a fallback when the TSDF returns `f32::MAX`.

---

## Implementation Plan

### Phase 1: Thread superquadric through the scoring pipeline

- [ ] **1.1** Update `score_all_particles` signature in `c_api.rs:209-214` to accept `sq_params: Option<&SuperquadricParams>`
- [ ] **1.2** Pass `sq_params` from `compute_from_request` to `score_all_particles` at the two call sites (`c_api.rs:449` and `c_api.rs:493`)
- [ ] **1.3** Thread `sq_params` through to `score_grasp` — update the scorer function signatures (`score_cylindrical`, `score_pinch`, `score_lateral`) and `score_grasp` to accept `sq_params: Option<&SuperquadricParams>`
- [ ] **1.4** Verify compilation with `cargo build --release --lib`

### Phase 2: Extend Tier 4 proximity with superquadric fallback

- [ ] **2.1** In `planner.rs:sparse_proximity_distance`, add `sq_params: Option<&SuperquadricParams>` parameter
- [ ] **2.2** When `tsdf.get_distance()` returns `f32::MAX` for all 3 key points, fall back to `sq_params.taubin_distance()` for each point and take the minimum
- [ ] **2.3** In the Tier 4 branch of `score_grasp` (`planner.rs:300-326`), use the combined TSDF+SQ distance for the proximity score. The scoring should smoothly transition:
  - Within TSDF band (0–2cm): use TSDF distance (high accuracy)
  - Beyond TSDF but SQ available (2cm–∞): use `taubin_distance` (approximate but directional)
  - No SQ available: fall back to current behavior (0.0)
- [ ] **2.4** Scale the SQ-based proximity score to range [0.0, 0.04] (slightly below the TSDF-based max of 0.05, to preserve the "TSDF is more accurate" hierarchy)

### Phase 3: Fix Tier 3 scoring with superquadric gradient

- [ ] **3.1** In the Tier 3 branch (`planner.rs:329-337`), use `sq_params` to compute how deep inside the object the palm center is
- [ ] **3.2** Score Tier 3 on a gradient: `contact_score = 0.04 + 0.06 * (1 - normalized_depth)` where:
  - Palm barely inside surface → `contact_score ≈ 0.10` (nearly as good as touching)
  - Palm deep inside → `contact_score ≈ 0.04` (worse than Tier 4 proximity at 0.05)
  - This ensures Tier 3 always scores between Tier 4 far (0.0) and Tier 4 near (0.05)
- [ ] **3.3** When no SQ available, keep current flat 0.1 (backward compatible)

### Phase 4: Create benchmark comparison figure

- [ ] **4.1** Add `plot_benchmark_comparison` function to `plot_results.py` — grouped bar chart showing baseline, single-view, and multi-view mean scores per object
- [ ] **4.2** Include error bars (std of combined_score across 100 trials)
- [ ] **4.3** Color-code: baseline=#55a868 (green), single-view=#5099e9 (blue), multi-view=#e68a00 (orange)
- [ ] **4.4** Add horizontal line for the "Tier 1 threshold" (contact_score ≥ 0.8 → combined ≈ 0.57)

### Phase 5: Tune approach distance

- [ ] **5.1** Reduce approach distance in `hand_approaches.py` from 25cm to **15cm** for all objects
- [ ] **5.2** Verify wrist camera visibility at 15cm (should be similar — the camera is still forward-looking from 16cm above)

### Phase 6: Rebuild, re-run, regenerate

- [ ] **6.1** `cargo build --release --lib` and copy `.so` to `lib/`
- [ ] **6.2** Re-run test suite: `python3 run.py --repetitions 100 --baseline-timeout 300`
- [ ] **6.3** Regenerate all figures: `python3 plot_results.py --format png --dpi 200`
- [ ] **6.4** Verify Tier 1 fraction ≥ 30% (up from ~15%)

---

## Verification Criteria

1. **Tier 4 proximity extends beyond 2cm**: A particle 10cm from the object should get a non-zero proximity score (currently 0.0)
2. **Tier 3 has gradient**: Two Tier 3 particles at different depths should get different scores
3. **Mean contact_score improves**: From ~0.10 to ≥ 0.20 (more particles finding the object)
4. **Tier 1 fraction improves**: From ~15% to ≥ 30% of trials
5. **Latency stays within budget**: P95 ≤ 100ms (SQ distance evaluation adds minimal cost — just a few `powf` calls per particle)
6. **Benchmark figure clearly shows the score ceiling**: baseline >> multi-view ≥ single-view

## Potential Risks and Mitigations

1. **SQ fitting fails for some objects**: The superquadric fit can return `None` if the point cloud is too small or the fit error exceeds the threshold. Mitigation: always fall back to TSDF-only scoring when `sq_params` is `None`.

2. **Taubin distance accuracy degrades far from surface**: The Taubin approximation `D ≈ F/||∇F||` is less accurate far from the surface. Mitigation: clamp the SQ proximity score to [0.0, 0.04] (below the TSDF max of 0.05) so it only provides coarse directional information.

3. **Latency increase from SQ evaluation**: `taubin_distance` involves ~10 `powf` calls plus a Newton refinement step. For 20K particles × 3 key points, this is ~60K evaluations. Mitigation: only evaluate SQ when TSDF returns `f32::MAX` (most particles near the surface use TSDF). Expected overhead: <5ms.

4. **Tier 3 gradient could create local minima**: If the SQ says a particle is "slightly inside" (high Tier 3 score), the optimizer might converge to a position just inside the surface rather than approaching from outside. Mitigation: cap Tier 3 score below the Tier 4 TSDF-based max (0.05), so the optimizer always prefers being near-but-outside over being inside.

5. **Approach distance change affects coverage figures**: Moving from 25cm to 15cm changes the camera geometry and occlusion patterns. Mitigation: the 3D setup figure will be regenerated to reflect the new geometry.

## Alternative Approaches

1. **Increase TSDF truncation instead of using SQ**: Set `truncation_cells` from 4 to 8 (2cm → 4cm band). Rejected per user request — "I do not want to increase the TSDF truncation level." Also, this would double the TSDF grid volume (8x more voxels), significantly increasing memory and computation.

2. **Use point cloud centroid distance**: Compute distance from hand to point cloud centroid as a cheap proximity proxy. Rejected — the centroid doesn't capture object shape, and the SQ provides a much better surface distance approximation at similar cost.

3. **Multi-resolution TSDF**: Use a coarse TSDF (2cm resolution, 10cm band) for proximity and a fine TSDF (5mm, 2cm band) for contact scoring. Rejected — significantly more complex to implement and maintain, with double the memory footprint.
