# Grasp Preshaping: Sampling Strategy & TSDF Sign Analysis

## Objective

Investigate two issues in the grasp preshaping pipeline:
1. **Sampling efficiency**: Whether using different sample counts for ROI prediction vs. SMC iterations would improve performance.
2. **TSDF sign calculation bugs**: Why obstructed voxels sometimes don't get negative signs, and why grasps not touching the object still get high contact scores after the sign fix.

---

## Task 1: Sampling Strategy Analysis

### Current Architecture (verified from code)

The pipeline in `c_api.rs:349-479` works as follows:

1. **ROI Prediction** (`c_api.rs:362-368`): Calls `predict_roi_with_samples()` which generates `PREDICTION_SAMPLES` (5000) poses from the motion model, projects fingertip positions, and computes the AABB that covers the search space. **These samples are discarded** — `_samples_for_roi` is unused.

2. **SMC Iteration 0** (`c_api.rs:402-409`): Calls `sample_initial_particles()` which generates a **new** set of `n_samples` (also `PREDICTION_SAMPLES` = 5000) particles from the same motion model. These are the particles actually scored.

3. **SMC Iterations 1-9** (`c_api.rs:425-479`): Resamples `n_samples` (5000) particles around elites each iteration, with decaying proposal variance.

**Key observation**: The ROI samples and SMC iteration-0 samples are drawn independently from the same distribution. They are NOT the same samples — they are two separate draws of 5000 each. The ROI samples are thrown away after AABB construction.

### Analysis: Different Sample Counts

| Phase | Current | Proposed Lower | Proposed Higher |
|-------|---------|---------------|----------------|
| ROI Prediction | 5000 | ~500-1000 | N/A |
| SMC Iter 0 | 5000 | N/A | ~8000-10000 |
| SMC Iter 1+ | 5000 | 5000 (same) | 5000 (same) |

**ROI Prediction**:
- The ROI only needs to cover where the hand *could* reach. With 5000 samples, the AABB is well-covered. 
- Since the AABB is inflated by `HAND_RADIUS_M` (0.05m) and enforced to `MIN_TSDF_DIM_M` (0.1m), the ROI is already conservative.
- **500-1000 samples would likely produce a nearly identical AABB** because the inflation and minimum-dimension enforcement dominate the boundary.
- Risk: With very few samples, the AABB might miss an extreme outlier position, but the inflation padding mitigates this.
- **Recommendation**: Reducing ROI samples to ~1000 is safe and saves ~0.5ms per pipeline invocation.

**SMC Iteration 0 (broad sampling)**:
- More initial samples = better coverage of the high-dimensional search space (6D pose + grasp type + wrist rotation).
- Currently 5000 samples × 10 iterations = 50,000 total evaluations. 
- If we do 8000 for iter 0 and 4000 for iter 1+, we get 8000 + 9×4000 = 44,000 total — similar cost but better initial coverage.
- **Recommendation**: This trade-off is marginal. The SMC resampling already focuses the search. Increasing initial samples helps more when the object is hard to find, but the proximity score (Tier 4, 0.0-0.05) already provides gradient.

**SMC Subsequent Iterations**:
- The elite selection + resampling mechanism means later iterations need fewer samples to refine.
- Could potentially reduce to 3000-4000 for iterations 2+ without quality loss.
- **Recommendation**: Keep the same count for simplicity. The current setup is clean and the performance difference is small.

### Verdict on Sampling

**My recommendation: Keep the same sample count across all phases for simplicity.** The reasons are:
1. The ROI computation cost is negligible compared to TSDF construction and scoring.
2. Having different sample counts adds config complexity without significant payoff.
3. The current SMC mechanism already handles the exploration-exploitation trade-off well through elite selection and decay.
4. If you want to optimize, the lowest-hanging fruit is reducing ROI samples to ~1000 (safe, simple, saves a small amount of time).

If you do want to differentiate, add these to `config.rs`:
- `ROI_SAMPLES: usize = 1000` (for AABB only)
- `SMC_INITIAL_SAMPLES: usize = 5000` (iteration 0)
- `SMC_RESAMPLE_COUNT: usize = 5000` (iterations 1+)

---

## Task 2: TSDF Sign Calculation Analysis

### Current Sign Logic (verified from `pointcloud_helper.rs:412-461`)

The sign determination at `pointcloud_helper.rs:413-460`:

```
For each voxel with non-zero, non-MAX distance:
  For each camera:
    If voxel is farther from camera than nearest surface point (d_cv > d_cp):
      → behind_count += 1
      → If voxel-to-surface vector aligns with camera-to-voxel ray (dot > 0.8):
        → inside_votes += 1
  
  If behind_count > n_cams/2 AND inside_votes > behind_count/2:
    → Flip sign to negative
```

### Identified Issues

#### Issue 1: The `n_cams/2` threshold is too strict for 2-camera setups

With 2 cameras, `n_cams/2 = 1`, so `behind_count > 1` means **both** cameras must agree the voxel is behind the surface. This is problematic because:

- **Camera on the opposite side** will see the voxel as *in front of* the surface (it's closer to that camera than the surface point is). So `d_cv < d_cp` and `behind_count` only increments for the camera on the near side.
- With 2 opposite cameras, a voxel truly inside the object will typically get `behind_count = 1` (only the near camera sees it as behind). 
- `1 > 1` is **false**, so the voxel stays positive!

**This is the primary bug.** For 2 cameras, the condition should be `behind_count > 0` or use a different voting scheme.

#### Issue 2: The alignment check can fail for voxels deep inside the object

The `RAY_ALIGNMENT_THRESHOLD` (0.8) checks if the direction from the nearest surface point to the voxel aligns with the camera-to-voxel ray. For voxels deep inside the object, the nearest surface point might be on a different face, causing the alignment to fail. This is a secondary issue but contributes to missed negatives.

#### Issue 3: "Object shadow" problem (fundamental TSDF limitation)

When approaching from the backside where there's no camera information:
- The TSDF has no data (f32::MAX) in the shadow region behind the object relative to all cameras.
- When a finger enters this shadow, `get_distance()` returns f32::MAX (trilinear interpolation short-circuits if any corner is MAX).
- In `sweep_for_collision()` (`planner.rs:418`), `f32::MAX < collision_tol` is false, so no collision is detected.
- **This is actually correct behavior** — the system correctly ignores regions with no information.

However, the problem occurs at the **boundary** between observed and shadow regions. A voxel just inside the object might have:
- One corner with positive distance (observed from one camera)
- Another corner with f32::MAX (shadow from the other camera)
- The interpolation returns f32::MAX, treating it as "no data" rather than "inside"

This means some truly obstructed voxels near the shadow boundary get treated as free space.

#### Issue 4: Why grasps not touching the object get high contact scores

After the sign fix, some voxels that should be negative are now negative. The collision detection in `sweep_for_collision()` (`planner.rs:403-443`) checks:

```rust
if tsdf.get_distance(p.x, p.y, p.z) < collision_tol {
    return Some(sample);  // collision found
}
```

A **negative** distance (inside the object) is `< collision_tol` (0.005), so it triggers as a collision. This means:

1. A finger that enters a region with negative TSDF values triggers a "collision" even if it's not near the actual surface.
2. The `find_active_contacts()` function (`planner.rs:509-550`) then checks `dist < threshold` (threshold = 2×collision_tol = 0.01) at the collision point.
3. If the finger is in a negative region (e.g., dist = -3.0 cells = -0.015m), then `-0.015 < 0.01` is true, so it's counted as an "active contact."
4. The surface normal is computed from the TSDF gradient, which points outward from the surface. But the force direction (finger movement direction) may not align well with this normal.
5. Despite poor alignment, the `contact_count_score` can still be high if multiple contacts are found.
6. The `contact_score` for Tier 1 is `0.8 + 0.2 * contact_count_score`, which can be 0.8-1.0 even for grasps that don't actually touch the surface.

**This is the root cause of the "high scores without touching" bug.** The negative voxels (inside the object) are being treated as collision surfaces, and the scoring system rewards these collisions equally.

### Proposed Solutions

#### Fix 1: Correct the sign voting threshold (Critical — fixes the missing negatives)

Change the condition at `pointcloud_helper.rs:457`:
```rust
// BEFORE: behind_count > n_cams / 2
// AFTER:  behind_count > 0  (any camera sees it as behind → it's inside)
```

Or better, use a more nuanced approach:
```rust
// A voxel is inside if ANY camera sees it as behind AND aligned.
// This handles multi-camera setups correctly.
if inside_votes > 0 {
    *dist = -*dist;
}
```

Rationale: If even one camera can confirm the voxel is behind the surface (with good alignment), it should be negative. The alignment check already provides robustness against false positives.

#### Fix 2: Distinguish negative-distance collisions from surface contacts (Critical — fixes the false high scores)

In `find_active_contacts()` (`planner.rs:509-550`), add a check that the contact is actually near the surface (distance close to zero), not deep inside:

```rust
// Only count as active contact if near the surface (|dist| < threshold)
// Negative distances far from zero mean the finger is inside the object,
// not at the surface.
let dist = tsdf.get_distance(p_hi.x, p_hi.y, p_hi.z);
if dist >= threshold || dist < -threshold {
    continue;  // Too far inside or outside
}
```

This ensures only contacts near the zero-crossing (actual surface) are scored. Contacts deep inside the object are filtered out.

#### Fix 3: Handle negative-distance collisions differently in scoring

In `sweep_for_collision()` (`planner.rs:403-443`), when a collision is found at a strongly negative distance, it should be treated differently:

Option A: Treat strongly negative distances as Tier 3 (start collision, score = 0.1) rather than a valid contact.
Option B: Add a separate tier for "finger inside object" with a very low score.

This would prevent the optimizer from rewarding grasps that pass through the object.

#### Fix 4 (Optional): Ray-casting occlusion check for more robust sign determination

Instead of the current heuristic (distance comparison + alignment), trace a ray from each camera to the voxel and check if any occupied voxel blocks the path. This is more accurate but computationally expensive. The BFS-based distance field already provides the nearest surface point, so we could check if the line from camera to voxel passes through any zero-distance (surface) voxel.

This is a larger change and probably not necessary if Fixes 1-3 resolve the issues.

### Priority Ranking

1. **Fix 1** (sign voting threshold) — Critical. This is the direct cause of missing negatives.
2. **Fix 2** (surface proximity filter) — Critical. This prevents false high scores from negative voxels.
3. **Fix 3** (negative distance tier) — Important. Provides better gradient for the optimizer.
4. **Fix 4** (ray casting) — Low priority. Nice-to-have for edge cases.

---

## Implementation Plan

### Phase 1: Sign Calculation Fix

- [ ] **Task 1.1**: Modify sign voting condition in `pointcloud_helper.rs:457`. Change `behind_count > n_cams / 2` to `inside_votes > 0` (any camera with good alignment confirms inside).
- [ ] **Task 1.2**: Add a unit test in `pointcloud_helper.rs` for 2 opposite cameras verifying that voxels inside the object get negative signs.
- [ ] **Task 1.3**: Add a unit test for voxels in the shadow region (no camera information) verifying they remain f32::MAX.

### Phase 2: Contact Scoring Fix

- [ ] **Task 2.1**: Modify `find_active_contacts()` in `planner.rs:509-550` to filter out contacts where the distance is strongly negative (e.g., `dist < -threshold`). Only count contacts near the zero-crossing.
- [ ] **Task 2.2**: Modify `sweep_for_collision()` in `planner.rs:403-443` to return additional metadata about whether the collision is at the surface or deep inside. Consider returning a struct instead of `Option<usize>`.
- [ ] **Task 2.3**: Add a new Tier between Tier 3 and Tier 2 for "collision with strongly negative distance" (finger inside object but not at start position). Score: ~0.05 (between Tier 4's proximity and Tier 3's start collision).
- [ ] **Task 2.4**: Add unit tests in `planner.rs` verifying that grasps with fingers inside the object (negative TSDF) get low contact scores.

### Phase 3: Sampling (Optional)

- [ ] **Task 3.1**: Add `ROI_SAMPLES` constant to `config.rs` (default: 1000).
- [ ] **Task 3.2**: Modify `c_api.rs:362` to use `ROI_SAMPLES` instead of `PREDICTION_SAMPLES` for the ROI prediction call.
- [ ] **Task 3.3**: Verify AABB coverage is equivalent with reduced samples.

## Verification Criteria

- [ ] Sign calculation: Voxels inside a convex object viewed by 2 opposite cameras are consistently negative.
- [ ] Sign calculation: Voxels outside the object remain positive.
- [ ] Sign calculation: Shadow regions (no camera data) remain f32::MAX.
- [ ] Contact scoring: Grasps with fingers inside the object (negative TSDF) get contact_score ≤ 0.1.
- [ ] Contact scoring: Grasps touching the actual surface get contact_score ≥ 0.8.
- [ ] Pipeline: End-to-end test with 2 opposite cameras produces valid grasps that touch the surface.
- [ ] Sampling: ROI AABB with 1000 samples covers ≥95% of the volume compared to 5000 samples.

## Potential Risks and Mitigations

1. **Sign fix too aggressive**: Changing to `inside_votes > 0` might flip some voxels incorrectly (e.g., near edges where alignment is ambiguous). Mitigation: The alignment threshold (0.8) already provides robustness; can increase to 0.9 if needed.

2. **Contact filter too strict**: Filtering `dist < -threshold` might miss valid contacts at the surface boundary where interpolation produces small negative values. Mitigation: Use a smaller threshold for the negative side (e.g., `dist < -threshold * 2`) to only filter clearly interior voxels.

3. **Breaking existing behavior**: The sign fix changes the TSDF output, which affects all downstream scoring. Mitigation: Run the debug visualizer before and after to compare results visually.

4. **Performance impact**: The sign calculation changes are in the parallel loop (`par_iter_mut`) and should have negligible performance impact.

## Alternative Approaches

1. **Weighted voting instead of threshold**: Instead of a binary inside/outside decision, use a continuous sign weight based on the fraction of cameras confirming inside. This provides a softer transition but adds complexity.

2. **Ray-marching for sign determination**: Trace rays from cameras through the TSDF to check for occlusions. More accurate but O(n_cameras × n_voxels × max_steps) complexity. Not recommended given the current BFS-based approach works well with the voting fix.

3. **Separate inside/outside TSDF bands**: Maintain two separate truncation bands — one for positive (outside) and one for negative (inside) distances. This allows different collision tolerances for each side. Adds memory but provides more control.
