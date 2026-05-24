# Grasp Preshaping Pipeline — Strategic Analysis & Implementation Plan

## Objective

Analyze and create actionable implementation plans for seven identified issues in the grasp preshaping pipeline, covering grasp scoring, wrist orientation, TSDF clipping, start-position collisions, self-collisions, TSDF sign calculation, and parallelism.

---

## Codebase Summary

The grasp preshaping system is a Rust library (`grasp_preshaping`) with a C FFI interface consumed by a C++ ROS 2 service bridge node. The pipeline flow is:

1. **Predictor** (`predictor.rs`) — samples future hand poses from current pose + twist
2. **Point cloud** (`pointcloud_helper.rs`) — prunes cloud to ROI, builds Morton-sorted TSDF via BFS
3. **Planner** (`planner.rs`) — sweeps finger contact points (from LUT) through the TSDF, binary searches for collision, then scores alignment and force closure
4. **C API** (`c_api.rs`) — orchestrates the pipeline, selects the best grasp across all samples × grasp types
5. **Service bridge** (`preshaping_service_bridge_node.cpp`) — ROS 2 node that calls the Rust library via FFI and publishes joint commands

---

## Issue 1: Grasp Scoring — Single-Finger Collision Accepted as Valid Grasp

### Analysis

**Root cause**: In `planner.rs:259-301`, `sweep_for_collision` returns the first sample where **any** `Flex::Coupled` point enters the TSDF below `collision_tol`. The sweep stops at the first collision — it does not check whether multiple fingers are in contact. The subsequent `find_active_contacts` at `planner.rs:375-416` collects contacts that are near the surface at the collision closure amount, but the alignment and force closure scores do not penalize having too few contacts.

For example, in a cylindrical grasp spec (`planner.rs:98-160`), there are 17 score_contacts. If only the thumb tip collides at sample 5, the sweep stops, binary search refines to closure=0.23, and `find_active_contacts` may find only 1 active contact. The alignment score averages over that single contact (can be high), and force closure with 1 contact gives `(1 - |centroid|)` which can also be moderate. The combined score can be non-trivially positive, causing this bad grasp to be selected.

**Why this is wrong**: A single finger touching an object is not a grasp. The planner should require a minimum number of contacting fingers proportional to the grasp type.

### Recommended Approach: Fix Grasp Scoring (Option 1 — Primary)

This is the recommended approach. It is faster to sample, simpler to implement, and the quality tradeoff can be mitigated by adjusting the contact count threshold.

**Rationale for Option 1 over Option 2 (keep stepping fingers)**:
- Option 2 (continue stepping until all fingers collide) introduces complex state management: different fingers may never collide (e.g., the object is too small for all fingers), and normalizing to an "appropriate closure amount" is ambiguous
- Option 2 also increases per-sample compute cost since each sample requires more TSDF lookups
- Option 1 cleanly separates the collision detection from the quality assessment

### Implementation Plan

- [ ] **1.1** Add a `min_contacts` field to `GraspSpec` (e.g., cylindrical=3, pinch=2, lateral=2) in `planner.rs:60-63`. This defines the minimum number of active contacts required for a valid grasp of that type.
- [ ] **1.2** Modify `score_grasp` at `planner.rs:252-301` to check the number of active contacts against `min_contacts`. If fewer contacts are found, return a zero-score result with `found_collision: true` but `alignment_score: 0.0` and `force_closure_score: 0.0`.
- [ ] **1.3** Add a `contact_count_score` component to `GraspScoreResult` — a normalized metric like `min(active.len(), spec.min_contacts) / spec.min_contacts` that rewards grasps with more contacts. Integrate this into `combined_score` with a new weight `w_contact_count`.
- [ ] **1.4** Update `GraspWeights` at `planner.rs:27-42` to include `w_contact_count: f64`.
- [ ] **1.5** Add a config constant for the contact count weight (e.g., `DEFAULT_W_CONTACT_COUNT: f64 = 1.5`) in `config.rs`.
- [ ] **1.6** Ensure `score_all_samples` in `c_api.rs:171-198` still works correctly — it already filters on `found_collision`, and the new zero-score grasps with `found_collision: true` will naturally rank below grasps with actual scores.

### Verification Criteria
- A grasp where only 1 finger contacts the object should score significantly lower than one where 3+ fingers contact
- The planner should never select a single-finger grasp as the best result when multi-finger alternatives exist
- Existing test cases for cylindrical, pinch, and lateral grasps should still pass

### Potential Risks
1. **Over-rejection of valid grasps on small objects**: If the object is very small (e.g., a pen), fewer fingers naturally contact it. Mitigation: set `min_contacts` conservatively (2 for cylindrical rather than 3), and allow the contact_count_score to be a soft penalty rather than a hard rejection.
2. **Tuning required**: The new weight needs empirical tuning. Mitigation: expose it as a config constant and test with the debug visualization.

---

## Issue 2: Wrist Orientation Handling

### Analysis

Currently, the predictor (`predictor.rs:150-176`) samples future poses by applying a twist-based displacement to the current pose. The twist has 6 DOF (3 angular, 3 linear), so orientation **is** sampled — but only as perturbations from the current orientation via the twist noise. The base orientation comes entirely from the input pose quaternion.

The issue is that the **search space is limited to perturbations around the current wrist orientation**. If the current wrist orientation is suboptimal for grasping, no amount of twist perturbation will discover a better one. The real hardware has a wrist joint that can rotate, and the planner should explore that space.

The `GraspComputeResponseFFI` at `ffi_types.hpp:69-78` does not return orientation — only closure amounts and grasp type. The ROS bridge at `preshaping_service_bridge_node.cpp:168-177` only publishes joint commands (thumb/index/mrl closure), not wrist orientation.

### Implementation Plan

- [ ] **2.1** Add explicit wrist orientation sampling to `sample_future_poses` in `predictor.rs`. Generate random small-angle rotations (e.g., uniform in [-30°, +30°] around each Euler axis) and compose them with the twist-based displacement. This samples across all dimensions (position + orientation) simultaneously as requested.
- [ ] **2.2** Add a new config constant for the wrist orientation sampling range (e.g., `WRIST_ORIENT_RANGE_RAD: f64 = 0.52` ≈ ±30°) in `config.rs`.
- [ ] **2.3** Add orientation output fields to `GraspComputeResponseFFI` in both `c_api.rs:78-88` and `ffi_types.hpp:69-78`: `qx: f64, qy: f64, qz: f64, qw: f64` for the best grasp's wrist orientation.
- [ ] **2.4** Update `ComputeOutput` at `c_api.rs:135-142` to include the orientation quaternion of the best grasp.
- [ ] **2.5** In `compute_from_request` at `c_api.rs:312-440`, extract the orientation from the best scored grasp's `SampledPose` and populate the new response fields.
- [ ] **2.6** In `preshaping_service_bridge_node.cpp`, read the new orientation fields from the FFI response and publish them on a new topic (e.g., `/grasp_preshaping/wrist_orientation` as `geometry_msgs/PoseStamped`).
- [ ] **2.7** Update the debug export to include the wrist orientation in the scored grasps output.

### Verification Criteria
- The planner explores orientations beyond the current wrist orientation
- The response includes a valid quaternion for the recommended wrist orientation
- The ROS node publishes the wrist orientation
- The debug visualization shows orientation spread

### Potential Risks
1. **Increased sampling dimensionality**: Adding 3 orientation DOF increases the search space. Mitigation: the existing 1000 samples (`PREDICTION_SAMPLES`) should be sufficient since we sample all dimensions simultaneously, but this may need tuning.
2. **FFI struct layout change**: Adding fields to `GraspComputeResponseFFI` requires rebuilding both Rust and C++ sides simultaneously. Mitigation: bump the API version returned by `grasp_preshaping_api_version`.

---

## Issue 3: Asymmetric TSDF Clipping

### Analysis

The TSDF grid dimensions are determined by the point cloud data extent in `get_tsdf` at `pointcloud_helper.rs:326-471`:

```
width  = max_gx + 1 + 2 * trunc
height = max_gy + 1 + 2 * trunc
depth  = max_gz + 1 + 2 * trunc
```

The grid is sized to fit the actual data plus truncation padding. The ROI (from `predict_roi_with_samples`) determines which points are kept via `prune`, but the TSDF grid itself is sized by the data extent, not the ROI.

**Current behavior (asymmetric, data-driven)**:
- The TSDF only occupies the space where there is actual point cloud data
- If the object is small and offset within the ROI, the TSDF is small and offset too
- This is memory-efficient and avoids wasted computation on empty space

**Alternative (symmetric, ROI-driven)**:
- Size the TSDF to fill the entire ROI regardless of data extent
- More uniform coverage, but potentially wasteful if data is sparse

### Assessment

The current asymmetric approach is **correct and preferable** for the following reasons:

1. **Memory efficiency**: A 0.3m ROI at 5mm resolution is 60³ = 216K voxels. If data only covers a fraction, the asymmetric approach uses proportionally less memory.
2. **No functional issue**: The planner queries the TSDF at finger positions. If a finger is in free space (no data), `get_distance` returns `f32::MAX`, which means "far from surface" — the sweep correctly treats this as no collision. This is the expected behavior.
3. **The ROI already constrains the search**: The `prune` function at `pointcloud_helper.rs:234-247` filters points to the ROI. The TSDF is built only from these points. The planner only evaluates samples within the ROI. So the effective search space is already bounded.

**One potential concern**: If the point cloud is very sparse in one dimension (e.g., a flat surface seen from one camera), the TSDF will be very thin in that dimension. Finger positions that are slightly outside the thin TSDF will get `f32::MAX` distances, which is correct (they are in unknown space, not in collision). This is fine.

### Recommendation

**Keep the current asymmetric clipping.** No changes needed. The data-driven sizing is appropriate and there are no correctness issues.

---

## Issue 4: Collisions in Start Position

### Analysis

The sweep in `sweep_for_collision` at `planner.rs:303-333` checks `Flex::Locked` points at sample 0 (the open hand position) first:

```rust
for sp in sweep_points {
    if let Flex::Locked(locked_s) = sp.flex {
        let p = pos_at_sample(lut, sp.contact, locked_s, base);
        if tsdf.get_distance(p.x, p.y, p.z) < collision_tol {
            return Some(0);
        }
    }
}
```

If a locked point (palm contacts, locked finger positions) collides at the open position, the function returns `Some(0)`. Then in `score_grasp` at `planner.rs:266-271`:

```rust
Some(0) => GraspScoreResult {
    closure_amount: 0.0,
    alignment_score: 0.0,
    force_closure_score: 0.0,
    found_collision: true,
},
```

This returns `found_collision: true` with all zero scores. In `score_all_samples` at `c_api.rs:184-185`, since `found_collision` is true, it computes a combined score which will be `0.0 * probability = 0.0` (since alignment and force_closure are both 0). This is low but not `-Infinity`.

**The real problem**: For `Flex::Coupled` points, the sweep starts at sample 0 and checks each sample. If a finger is already inside the object at sample 0 (the open hand position), the sweep returns `Some(0)` immediately. But this sample 0 collision is treated the same as a locked-point collision — zero scores but `found_collision: true`.

Additionally, there is **no check for coupled points at sample 0** before the sweep loop. The sweep loop at `planner.rs:321-330` starts at `sample = 0`, so if a coupled point is already colliding at the open position, it returns `Some(0)`.

### Implementation Plan

- [ ] **4.1** Add a pre-sweep check in `sweep_for_collision` at `planner.rs:303-333` that checks **all** sweep points (both locked and coupled) at the open position (sample 0). If any coupled point collides at the open position, return a special sentinel value (e.g., `None` or a new `SweepResult::StartCollision` variant) to indicate the start position is invalid.
- [ ] **4.2** Alternatively (simpler): check coupled points at sample 0 as part of the existing locked-point check block. If any point collides at the open position, return `None` (no valid grasp found for this pose), which maps to `found_collision: false` and gets `-Infinity` score. This effectively invalidates the starting position.
- [ ] **4.3** Add a config constant `INVALIDATE_START_COLLISIONS: bool = true` to allow toggling this behavior.
- [ ] **4.4** Ensure the debug export shows which samples were invalidated due to start-position collisions.

### Verification Criteria
- Poses where the hand is already inside the object at the open position are rejected (score = -Infinity)
- The planner continues searching other samples instead of accepting a zero-quality grasp
- The best grasp is never one where the start position was in collision

### Potential Risks
1. **Too many rejections**: If the object is very close to the hand, many samples may have start-position collisions. Mitigation: this is correct behavior — if the hand is already penetrating the object, it's not a valid grasp starting point.

---

## Issue 5: Self Collisions

### Analysis

Currently, the planner only checks finger-to-object collisions (via the TSDF). There is no check for finger-to-finger or finger-to-palm self-collisions. This means the planner can recommend a grasp where fingers interpenetrate each other, which is physically impossible.

The LUT (`lut_helper.rs`) stores finger contact positions as a function of a single closure parameter (0.0 = open, 1.0 = closed). All coupled fingers move together. The LUT was presumably generated from the actual hand kinematics, so self-collisions are implicitly encoded in the LUT data — but only if the LUT was generated with collision checking.

### Recommended Approach: Pre-Sampled Finger Range Limits

The user's suggestion of pre-sampling how far each finger can go in a given grasp type is sound. This avoids runtime self-collision checking (which would require a full hand mesh model) and instead uses offline-computed limits.

### Implementation Plan

- [ ] **5.1** Create a self-collision analysis script/tool (Python or Rust) that loads the hand model (MuJoCo XML) and, for each grasp type, simulates closing the hand and records the closure amount at which self-collision first occurs for each finger configuration. Output: a mapping from grasp type → maximum valid closure amount.
- [ ] **5.2** Add a `max_closure` field to `GraspSpec` in `planner.rs:60-63` that specifies the maximum valid closure for each grasp type (e.g., cylindrical=0.85, pinch=0.9, lateral=0.7).
- [ ] **5.3** Modify `sweep_for_collision` to clamp the sweep range to `[0, max_closure_sample]` instead of sweeping the full `[0, resolution)` range. This prevents the planner from exploring closure amounts where self-collisions occur.
- [ ] **5.4** Alternatively (more granular): store per-finger max closure limits in the LUT data as additional arrays (e.g., `max_closure_index`, `max_closure_thumb`, etc.) and clamp each finger's sweep independently.
- [ ] **5.5** Update the binary search in `refine_binary` at `planner.rs:335-353` to respect the max closure limit as the upper bound.

### Verification Criteria
- No recommended grasp has fingers in self-collision
- The max closure limits are consistent with the physical hand model
- Grasp quality does not degrade significantly (the limits should only exclude physically impossible configurations)

### Potential Risks
1. **LUT generation dependency**: This approach requires access to the hand model for pre-sampling. Mitigation: use the existing MuJoCo scene files.
2. **Conservative limits**: If the pre-sampled limits are too conservative, valid grasps may be excluded. Mitigation: add a safety margin parameter and validate empirically.

---

## Issue 6: TSDF Sign Calculation with Opposing Cameras

### Analysis

The current sign calculation in `get_tsdf` at `pointcloud_helper.rs:412-461` works as follows:

1. Build an unsigned distance field via BFS from surface voxels
2. For each non-surface voxel, check all cameras: if the voxel is farther from the camera than its nearest surface point, and the voxel-to-surface vector aligns with the camera-to-voxel ray, it's considered "behind" the surface (inside the object)
3. If a majority of cameras agree the voxel is behind, and a majority of those also agree on the alignment, the distance is negated

**The problem with opposing cameras**: Consider a thin object viewed from both sides. Camera A is at -Z, Camera B is at +Z. A voxel on the far side of the object (from Camera A's perspective) is "behind" the surface w.r.t. Camera A. But from Camera B's perspective, the same voxel is "in front of" the surface. The majority vote (`behind_count > n_cams / 2`) fails because only half the cameras see it as behind.

The user's proposed approach is: for each camera, check if the ray from the camera to the voxel passes through any occupied voxels. If any camera has a clear line of sight (no occlusions), the voxel is outside. If all cameras are occluded, the voxel is inside.

### Assessment of the Proposed Approach

The proposed approach is **correct in principle** and aligns with standard TSDF sign calculation. However, there are nuances:

1. **The current TDF is unsigned**: The BFS builds an unsigned distance field. The sign is added afterward. The proposed approach would replace the current heuristic with a ray-marching sign determination.

2. **Ray occlusion check**: For each voxel, march along the ray from each camera to the voxel. If any occupied voxel (distance = 0) is encountered along the ray, the camera cannot see this voxel → the voxel is potentially inside. If at least one camera has a clear ray, the voxel is outside.

3. **Performance concern**: Ray marching for every voxel × every camera is O(voxels × cameras × ray_length). With the current grid sizes (~100³ voxels), this is feasible but needs optimization.

4. **Integration with BFS**: The BFS already computes nearest-surface-point information (`nearest[]` array). This can be leveraged: for each voxel, the nearest surface point is known. The ray from camera to voxel passes through the nearest surface point if and only if the voxel is behind the surface from that camera's perspective. This is essentially what the current code does, but the majority-vote logic is wrong for opposing cameras.

### Recommended Approach: Any-Camera Visibility

Instead of majority vote, use **any-camera visibility**: a voxel is outside if **any** camera has a clear line of sight to it (no occupied voxels along the ray). A voxel is inside only if **all** cameras agree it is occluded.

### Implementation Plan

- [ ] **6.1** Replace the sign calculation logic in `get_tsdf` at `pointcloud_helper.rs:412-461` with an any-camera visibility check. For each non-surface voxel: (a) for each camera, check if the ray from camera to voxel intersects any surface voxel; (b) if any camera has a clear ray, the voxel is outside (positive distance); (c) if all cameras are occluded, the voxel is inside (negative distance).
- [ ] **6.2** Optimize the ray-surface intersection check using the existing BFS data. The `nearest[]` array already maps each voxel to its closest surface voxel. For a given voxel V with nearest surface point S: if S is between the camera C and V (i.e., `|C-S| < |C-V|`), then the ray from C to V passes through (or near) S. This is essentially the current approach but with corrected logic.
- [ ] **6.3** Fix the majority-vote condition: change `behind_count > n_cams / 2` to `behind_count == n_cams` (all cameras agree the voxel is behind). This ensures that with opposing cameras, a voxel is only marked as inside if it is occluded from **all** viewpoints.
- [ ] **6.4** Add a more robust ray-occlusion check: instead of just checking distance comparison, actually trace the ray through the TSDF grid and check if any voxel along the ray has distance 0 (surface). This handles cases where the nearest surface point is not on the direct ray.
- [ ] **6.5** Update the existing test `tsdf_sign_negative_inside_with_cameras` at `pointcloud_helper.rs:559-583` to also test the opposing-cameras case.
- [ ] **6.6** Add a new test case with two cameras on opposite sides of a thin object and verify correct sign assignment.

### Verification Criteria
- With two opposing cameras, voxels inside the object are correctly marked as negative
- Voxels outside the object (visible from at least one camera) are correctly marked as positive
- The single-camera case still works correctly
- Performance does not degrade significantly (the optimization in 6.2 should help)

### Potential Risks
1. **Performance**: Full ray marching is expensive. Mitigation: use the `nearest[]` array optimization (6.2) which avoids explicit ray marching in most cases.
2. **Edge cases at object boundaries**: Voxels very close to the surface may have ambiguous visibility. Mitigation: the existing `collision_tol` parameter handles this uncertainty in the planner.

---

## Issue 7: Rayon Parallelism

### Analysis

Rayon is already a dependency (`Cargo.toml:12`) and is used in:
- `pointcloud_helper.rs:239` — parallel point cloud pruning (`par_iter`)
- `pointcloud_helper.rs:289` — parallel Morton code computation (`par_iter`)
- `pointcloud_helper.rs:313` — parallel sort (`par_sort_by_key`)
- `pointcloud_helper.rs:415` — parallel TSDF sign computation (`par_iter_mut`)

The main opportunities for additional parallelism are:

1. **`score_all_samples` in `c_api.rs:171-198`**: This is a triple-nested loop (samples × grasp types × sweep points). The outer loop over samples is embarrassingly parallel — each sample is independent.
2. **`sweep_for_collision` in `planner.rs:303-333`**: The inner loop over sweep points for each sample could be parallelized, but the early-return-on-collision semantics make this tricky.
3. **`find_active_contacts` in `planner.rs:375-416`**: The loop over score_contacts is parallelizable.

### Implementation Plan

- [ ] **7.1** Parallelize `score_all_samples` in `c_api.rs:171-198` using `par_iter()`. Collect `(sample_index, scorer_index)` pairs and iterate them in parallel. Each iteration produces a `ScoredGrasp`. Collect results into a `Vec<ScoredGrasp>`.
- [ ] **7.2** The TSDF and LUT are read-only during scoring, so they can be safely shared across threads via references (Rayon handles this).
- [ ] **7.3** Benchmark the parallelized version against the sequential version using the existing `bench_full_pipeline` benchmark.
- [ ] **7.4** (Optional) Parallelize the BFS in `get_tsdf` using a parallel BFS approach (e.g., wavefront parallelism). This is more complex and may not be worth the effort for typical grid sizes.

### Verification Criteria
- Correctness: parallelized scoring produces identical results to sequential scoring
- Performance: measurable speedup on multi-core systems (target: 2-4x on 4+ cores)
- No data races or deadlocks

### Potential Risks
1. **Diminishing returns**: If the TSDF construction dominates runtime, parallelizing scoring won't help much. Mitigation: profile first to identify the actual bottleneck.
2. **Overhead for small sample counts**: With 1000 samples × 3 grasp types = 3000 evaluations, Rayon's overhead should be negligible, but verify.

---

## Priority Ranking

| Priority | Issue | Impact | Effort |
|----------|-------|--------|--------|
| **P0** | Issue 1: Grasp scoring fix | High — directly causes bad grasps | Low |
| **P0** | Issue 4: Start position collisions | High — accepts invalid grasps | Low |
| **P1** | Issue 6: TSDF sign calculation | Medium — incorrect inside/outside with opposing cameras | Medium |
| **P1** | Issue 5: Self collisions | Medium — can produce physically impossible grasps | Medium-High |
| **P2** | Issue 2: Wrist orientation | Medium — limits grasp diversity | Medium |
| **P3** | Issue 7: Rayon parallelism | Low — performance optimization | Low |
| **N/A** | Issue 3: Asymmetric clipping | None — current behavior is correct | None |

### Rationale for Priority Order

1. **Issues 1 and 4 are P0** because they directly cause the planner to accept invalid or poor-quality grasps. These are correctness bugs that affect every planning result.
2. **Issue 6 is P1** because incorrect TSDF signs affect collision detection accuracy, which is fundamental to the planner. With multiple cameras (which is the target deployment scenario), this will cause incorrect inside/outside classification.
3. **Issue 5 is P1** because self-collisions produce physically impossible grasps, but the impact depends on how often self-collisions actually occur in practice (the LUT may already avoid them for most closure amounts).
4. **Issue 2 is P2** because it limits the diversity of explored grasps but doesn't cause incorrect results — it just misses potentially better grasps.
5. **Issue 7 is P3** because it's a performance optimization with no correctness impact.
6. **Issue 3 requires no action** — the current behavior is correct and efficient.
