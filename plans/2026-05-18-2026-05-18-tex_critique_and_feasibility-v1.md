# Plan: Critique and Feasibility Analysis of `rapport/current_grasp_section.tex`

## Objective

Provide a detailed comparison of the tex skeleton against the actual codebase and the typst version, answer the open questions raised in the tex file, assess feasibility of proposed changes, and suggest improvements.

---

## Part A: Overall Comparison — Tex vs Code vs Typst

### Structural Alignment

The tex file follows a sensible section order that is closer to the actual pipeline flow than the typst version:
- Tex moves TSDF construction AFTER superquadric backside estimation → **Correct**. The code at `c_api.rs:398-414` confirms: SQ fitting happens before `get_tsdf()`, and the SQ params are passed into TSDF construction.
- Typst has TSDF before SQ → **Incorrect ordering** relative to the final implementation.

The tex file is more honest about uncertainties ("I am not sure", "maybe", "not sure if valid") which is better for a working document than the typst's confident but sometimes inaccurate claims.

---

## Part B: Section-by-Section Analysis and Answers to Open Questions

### Section: Related Work

**Tex says**: "Not sure if these are the right ones to compare to... maybe something on primitive estimation and principal component for axis."

**Assessment**: The three chosen (Dex-Net, Contact-GraspNet, GraspNet-1Billion) are appropriate for positioning. The key differentiators to emphasize:

1. **Latency**: Dex-Net uses GQ-CNN (~10-50ms per grasp on GPU), Contact-GraspNet is a deep network (~100ms+). Our pipeline runs CPU-only in Rust with a ~50ms budget. This is the strongest argument.
2. **No training data**: We have a specific hand prosthesis with coupled finger mechanics. No grasp dataset exists for this hardware. Learning-based approaches require retraining.
3. **Partial observability during motion**: Most grasp planners assume a static scene with complete point clouds. Our system must plan while the hand is approaching, with only the visible portion of the object.

**Recommendation on simpler comparison**: The tex author's instinct about "primitive estimation + PCA for axis" is actually a good idea for a simpler baseline. This is essentially what the SQ backside estimation does (PCA → primitive fitting), but without the full TSDF + SMC optimization. A valid comparison would be: "Given PCA axis, choose cylindrical grasp aligned with longest axis, close until collision." This would work for simple objects but fail on asymmetric or complex shapes. Worth mentioning as motivation, not as a full comparison.

**On "why not ML"**: The strongest argument is the hardware constraint — the prosthesis has coupled finger joints, so standard grasp datasets (which assume independent finger control) don't transfer. Add this.

### Section: Architecture

**Tex says**: "We should likely talk about the fact that we use Rust, and how we do some parallelizations."

**Assessment**: Yes, this is worth mentioning briefly. The codebase uses `rayon` for parallelization in:
- `pointcloud_helper.rs:243` — parallel pruning
- `pointcloud_helper.rs:316` — parallel Morton sorting
- `pointcloud_helper.rs:431` — parallel sign determination
- `pointcloud_helper.rs:583` — parallel SQ distance blending
- `superquadric.rs:629` — parallel template matching (3 templates via `par_iter`)
- `c_api.rs:218` — parallel particle scoring

This is not a main contribution but worth a sentence about choosing Rust for zero-cost abstractions + fear-free concurrency via rayon, enabling the ~50ms budget on CPU.

**Tex says**: Pipeline is `PointCloud → Prune(ROI) → Morton Sort → Superquadric Backside Estimation → TSDF Construction → (SMC Loop → Score Particles) → Select Best Grasp → Output`

**Verification against code** (`c_api.rs:361-501`):
1. Parse point cloud from FFI ✓
2. `predict_roi_with_samples()` → ROI AABB ✓
3. `prune(&cloud, Some(roi))` ✓
4. `morton(&pruned, ...)` ✓
5. `fit_best_superquadric(&pruned.points)` ✓
6. `get_tsdf(..., sq_params.as_ref())` ✓
7. SMC loop: `sample_initial_particles` → `score_all_particles` → `select_elite_indices` → `resample_around_elites` ✓
8. `select_best_grasps(&scored)` ✓

**The tex pipeline order is correct.** The typst version had TSDF before SQ, which is wrong.

### Section: Region of Interest

**Tex says**: "The goal was to sample twists using covariance and then propagate them, but I think we propagate without noise, and then add it later. Maybe there is no functional difference."

**Verification against code** (`predictor.rs:104-133`):
The `sample_twist()` function:
1. Computes mean twist scaled by time: `omega_mean = twist.omega * t`, `v_mean = twist.v * t`
2. Samples noise from Normal(0, σ√t) for each component
3. Returns `SampledTwist { omega: omega_mean + noise_omega, v: v_mean + noise_v }`

So the noise is added to the scaled twist BEFORE propagation through `twist_to_se3()`. The tex author's intuition is correct — there is no functional difference between "propagate then add noise" and "add noise then propagate" for small perturbations. The current approach (noise in twist space, then propagate) is actually more principled because it produces physically consistent screw motions.

**Answer**: The noise IS added before propagation (in twist space), not after. But the distinction doesn't matter functionally for small covariances. Worth a brief mention.

**Tex says**: "We do ROI pointcloud pruning if earlier cleaning fails to isolate the object exactly."

**Verification**: The pruning at `c_api.rs:382` uses the ROI AABB to filter the full point cloud. This serves two purposes:
1. Removes distant points that aren't relevant
2. Reduces the point count for Morton sorting and TSDF construction

The tex is correct that this is a safety net for imperfect segmentation upstream.

### Section: Pruning and Morton Ordering

**Tex says**: "I believe we first prune based on the ROI, and then we might compute the actual AABB (but I am not sure about this)."

**Verification**: Looking at `c_api.rs:382-396`:
1. `prune(&cloud, Some(roi))` — filter by ROI AABB
2. `morton(&pruned, ...)` — this function at `pointcloud_helper.rs:275-328` recomputes the AABB from the pruned points internally (lines 279-288), then uses it for Morton code computation

So yes, after pruning, the Morton function computes a tighter AABB from the remaining points. The tex author's memory is correct.

**Tex says**: "Maybe something about how Morton codes work briefly."

**Recommendation**: Include a brief explanation: Morton codes interleave x/y/z bits so spatially close points get close codes. Sorting by Morton code produces a Z-order curve, which gives cache-friendly TSDF traversal. The implementation uses `split_by_3()` to interleave 16-bit coordinates into 48-bit codes.

### Section: Initial TSDF Construction via BFS

**Tex says**: "For sign assignment, we check that a voxel is behind the surface (maybe using points, not sure), and then check the alignment using dot product with a threshold."

**Verification** (`pointcloud_helper.rs:417-519`):
The sign logic is more nuanced than described:
1. For each voxel, find the nearest surface point (from the BFS `nearest[]` array)
2. For each camera, compute the camera-to-voxel ray direction
3. Project the camera-to-surface vector onto this ray
4. If `voxel_proj > surf_proj` (voxel is farther along ray than surface) → inside vote
5. Also check alignment: direction from surface to voxel must align with camera-to-voxel ray (dot > 0.9)
6. If `inside_votes > 0 AND outside_votes == 0` → negative sign
7. If cameras disagree (both inside and outside votes) → use SQ as tiebreaker

The tex should describe this two-stage process: ray-based voting first, SQ tiebreaker for opposing cameras.

**Tex says**: "Since we only check in the truncation band, there is no need to further check which camera saw what voxels."

**Assessment**: Partially correct. The BFS naturally limits to the truncation band, but the sign determination still needs per-camera checks because different cameras see different faces. The current code handles this correctly through the voting mechanism.

### Section: Design Decisions (TSDF)

**Tex says**: "We reason that we would have two TSDFs and would then have to merge them, which is an issue we could likely have dealt with. I am not sure if there is a super valid reason for it."

**Assessment**: This is a valid self-critique. The actual reasons for BFS-based TSDF over projective TSDF are:
1. **Simplicity**: No need for camera intrinsics, depth maps, or multi-TSDF fusion
2. **Multi-camera handling**: The BFS approach naturally handles multiple cameras through the voting mechanism, while projective TSDF requires explicit fusion (weighted average, etc.)
3. **Works with arbitrary point clouds**: The input is already a segmented point cloud, not raw depth images. Projective TSDF assumes you have depth maps.
4. **Speed**: BFS from surface voxels is O(n_voxels_in_band) which is very fast for small truncation bands.

The "two TSDFs to merge" concern is real but not the primary reason. The primary reason is that the input is already a segmented point cloud, not raw sensor data.

**Tex asks**: "Why voxel representation? Why truncation? Why signed?"

**Answers**:
1. **Voxel/distance field**: Enables O(1) distance queries for collision detection. No need for nearest-neighbor searches against the point cloud for every finger position.
2. **Truncation**: Limits memory and computation. Only the band near the surface matters for collision detection. Beyond the truncation distance, we don't care about exact distances.
3. **Signed**: Needed for surface normals (gradient of SDF), which are used for alignment scoring and force direction computation. Also distinguishes "inside" from "outside" for proper collision semantics.

### Section: Superquadric Backside Estimation

**Tex says**: "Our initial thought was to do a cube, a cylinder and a sphere. But since they differ in equations... we wanted a unified representation."

**Verification**: The code at `superquadric.rs:47-63` defines exactly three templates:
- Sphere: ε₁=1.0, ε₂=1.0
- Box: ε₁=0.1, ε₂=0.1
- Cylinder: ε₁=0.1, ε₂=1.0

This confirms the tex narrative. The superquadric formulation unifies these three primitives under one equation with different shape exponents.

**Tex says**: "It is the Taubin distance with one Newton refinement step. I think Taubin is like Newton, but with some extra tricks."

**Verification** (`superquadric.rs:247-280`):
The Taubin distance is `D = F(x) / ||∇F(x)||`, which is a first-order approximation of the true distance to the implicit surface. The code then does one Newton refinement step: project along the gradient to the estimated surface, evaluate the residual, and correct. This is not "extra tricks" — it's simply a second-order correction that significantly improves accuracy for points far from the surface. The Taubin distance is accurate to ~15% without refinement and ~5% with one refinement step (as verified by the unit tests at `superquadric.rs:1083-1123`).

**Tex says**: "Then I think we optimise something like positions, rotations and types, using either Levenberg-Marquardt or maybe something simpler like Newton."

**Verification** (`superquadric.rs:467-597`):
The fitting uses **Gauss-Newton with Levenberg-Marquardt damping** — it builds JᵀJ (the approximate Hessian) and adds λ·diag(JᵀJ) as damping. This is a hybrid approach. The rotation is **locked** from PCA — only 6 parameters are optimized: 3 scale (a, b, c) and 3 translation (tx, ty, tz). The rotation comes from PCA and is NOT optimized.

**Tex says**: "I am not sure if we still have a blending domain or not."

**Verification** (`pointcloud_helper.rs:521-665`):
Yes, the blending domain still exists. The SQ integration has two regimes:
1. **Signs agree** (camera and SQ both say inside or both say outside): blend distances in the outer band (near truncation boundary) using smoothstep interpolation
2. **Signs disagree** (camera says outside, SQ says inside — the backside case): use SQ sign, blend distances with a transition zone. Close to visible surface → trust camera distance but flip sign. Far from surface → trust SQ distance entirely.

The blending uses smoothstep: `w = 1 - t²(3 - 2t)` for smooth transitions. The blend zone is controlled by `SQ_BLEND_DELTA_CELLS` (default: 3 cells) for agreeing signs and `SQ_MIN_SIGN_OVERRIDE_CELLS` (default: 2 cells) for disagreeing signs.

### Section: Finger Contact Lookup Table

**Tex says**: "This is done through Pinocchio and the official URDF (though made flat and patched a bit)."

**Assessment**: This is correct context. The LUT is pre-computed externally using Pinocchio (a rigid body dynamics library) to step through the hand's closure trajectory. The URDF is modified to flatten the kinematic chain (removing closed-loop constraints into open-chain approximations).

**Tex says**: "Store them as dual quaternions (For cheaper computations, I think, and also simpler interpolations)."

**Verification**: Dual quaternions provide:
1. **Efficient interpolation**: `DualQuaternion::lerp()` at `lut_helper.rs:52-81` — linear blend with sign continuity handling
2. **Compact representation**: 8 doubles (64 bytes) vs 4×4 matrix (128 bytes) per transform
3. **Singularity-free rotation interpolation**: Unlike Euler angles or axis-angle

The primary benefit is indeed the interpolation quality — linearly blending dual quaternions produces approximately screw-motion interpolation, which is physically meaningful for finger trajectories.

### Section: Grasp Scoring

**Tex asks**: "Should we modify our current approach in the Rust code? I think we do some relatively complex approximations to avoid estimating the centre of mass, but since we now do backside estimations, we should maybe not do these."

**This is the most important open question. Analysis:**

The current `compute_force_closure()` at `planner.rs:636-654` computes:
```
force_closure = 1 - ||mean_surface_normal|| / n_contacts
```
This is NOT a true wrench-based force closure metric. It's a proxy that measures how "balanced" the contact normals are. If normals point in opposing directions, their average is small → high score. This is a reasonable approximation that doesn't require knowing the center of mass.

**With backside estimation**, the argument for keeping the proxy:
1. The SQ provides shape completion, not mass properties. We still don't know the object's density distribution.
2. The SQ's centroid ≠ center of mass. Assuming uniform density for the SQ would be an approximation itself.
3. The proxy works well in practice — it penalizes one-sided grasps.

**The argument for implementing true wrench analysis:**
1. With SQ backside, we CAN estimate a centroid (the SQ center) as a proxy for COM
2. A true wrench metric would better distinguish stable from unstable grasps
3. The current proxy can give high scores to grasps with balanced but weak contacts

**My recommendation**: Keep the proxy for the report. The reasoning is sound even with SQ: we don't know mass distribution, and the proxy captures the key property (balanced opposition). If time permits, implement a wrench metric as an improvement and compare in the evaluation section. But don't frame the current approach as a "bug" — frame it as a deliberate simplification.

**Feasibility of implementing true wrench metric**: Moderate. Would need:
- Estimate COM from SQ centroid (or centroid of observed points)
- For each contact, compute torque = r × F where r = contact_pos - COM, F = surface_normal
- Compute the grasp wrench space and check if it contains the origin
- This is well-studied; ~50-100 lines of code
- **Time estimate**: 2-4 hours for implementation + testing

**Tex says**: "There is something about something having a higher weight, and I believe there is a very good reason for that."

**Verification** (`runtime_config.rs:64-67`):
```
GRASP_WEIGHT_PROBABILITY: 1.0
GRASP_WEIGHT_ALIGNMENT: 1.0
GRASP_WEIGHT_FORCE_CLOSURE: 1.0
GRASP_WEIGHT_CONTACT_SCORE: 3.0
```

The `contact_score` has 3× weight. The reason: `contact_score` is the tiered metric that provides the primary optimization gradient. It encodes the full collision hierarchy (Tier 1-4), while the other scores only differentiate within Tier 1 (valid grasps). Without the 3× weight, the optimizer would not strongly prefer making contact over merely being near the object.

**The lower weight**: All others are equal at 1.0. The `sample_probability` being 1.0 (not higher) is correct — it should not dominate over actual grasp quality.

### Section: Tier-Based Scoring System

**Tex says**: "This needs to be detailed, like what tiers we have and why."

**Verification from `planner.rs:296-450`**:

| Tier | Condition | contact_score | Purpose |
|------|-----------|---------------|---------|
| 4 | No collision (swept fully closed) | 0.0–0.05 (proximity) | Provides gradient toward object when hand misses |
| 3 | Collision at sample 0 (start position) | 0.1 (fixed) | Penalizes hand starting inside object |
| 2 | Collision found but < min_fingers or missing thumb/index | 0.0–0.5 | Partial credit: hand touches object but grip is invalid |
| 1 | Collision found, ≥ min_fingers, has thumb+index | 0.8–1.0 | Valid grasp: full scoring with alignment + force closure |

**Why this matters for the report**: The tiered system is one of the key design decisions. Without it, the optimizer would have a binary landscape (collision/no collision) with no gradient. The tiers create a smooth landscape that guides the SMC from "approaching" → "touching" → "gripping."

### Section: Collision Detection

**Tex says**: "We also take the grasp type into account, so one finger is not moved further than is physically possible on the hardware couplings. Also, we bake hardware limits into the LUT."

**Verification**: The `max_closure_index` in each grasp spec (`planner.rs:78`) maps to a pre-computed maximum closure value from the LUT. This prevents:
- Cylindrical: all fingers close together
- Pinch: middle/ring/little locked, only thumb+index close
- Lateral: thumb in adducted mode, index side contacts

The `Flex::Locked(0)` entries in the specs mean those contacts are fixed at sample 0 (open position) — they don't move during the sweep. This correctly models the coupled finger mechanics.

### Section: SMC Optimization

**Tex asks**: "Maybe something about having x samples per dimension or something."

**Assessment**: The search space is effectively 8-dimensional:
- 6D pose (3 translation + 3 rotation from twist propagation)
- 1D grasp type (categorical: 0, 1, 2)
- 1D wrist rotation (continuous: [-π/2, π/2])

With 20,000 samples in iteration 0, that's ~3 samples per dimension if distributed uniformly, which is very sparse. The SMC works because:
1. The motion model provides a strong prior (poses cluster around likely future positions)
2. The tiered scoring provides gradient even in sparse regions
3. Resampling focuses the population quickly

**Tex says**: "We should write something about elite preservation, and what other modifications we made."

**Verification from code** (`predictor.rs:288-423`, `c_api.rs:477-500`):

Five SMC enhancements, each solving a specific problem:

1. **Elite injection** (`predictor.rs:326-328`): Top 1% of elites are preserved unchanged. Prevents losing the best solution during resampling.

2. **Score-weighted parent selection** (`predictor.rs:333-348`): New particles are generated around parents selected with probability proportional to their score. Better than uniform selection.

3. **Grasp type mutation** (`predictor.rs:391-397`): 10% chance of mutating to a different grasp type, sampled from weighted distribution based on population performance. Prevents premature convergence to one type.

4. **Decaying proposal variance** (`c_api.rs:487-490`): `std = initial_std × 0.7^iteration`. Starts broad, narrows over iterations. Standard SMC practice.

5. **Convergence-based early stopping** (`c_api.rs:462-469`): Stops if best score changes by < 0.01 between iterations (after minimum 3 iterations). Saves computation.

### Section: Evaluation

**Tex says**: "Maybe use the Rust benchmark as a foundation here."

**Assessment**: The pipeline already records `pipeline_time_ms` in the response (`c_api.rs:599`). The debug dump also captures per-iteration data. A simple evaluation script could:
1. Run the pipeline on N test objects with known ground truth
2. Record timing breakdown (ROI, Morton, SQ, TSDF, SMC per iteration)
3. Report grasp type distribution and score distributions

**Tex asks**: "Maybe see how the variability changes without the backside?"

**Assessment**: This is feasible and valuable. The `SQ_ENABLE_BACKSIDE` config flag (`runtime_config.rs:70`) already allows disabling SQ. An ablation study comparing:
- With SQ backside vs. without
- Score distributions, grasp success rates, timing

This would be the strongest evaluation result. **Recommendation**: Do this — it directly validates the main contribution of the SQ backside estimation.

**Feasibility**: High. Set `sq_enable_backside: false` in the YAML config, run the same test cases, compare results.

### Section: Integration

**Tex says**: "Not sure what this should contain."

**Assessment**: Keep it brief. The key points:
1. Rust library compiled to `.so`, loaded by C++ bridge node via `dlopen`
2. FFI boundary uses flat C structs (`GraspComputeRequestFFI` / `GraspComputeResponseFFI`)
3. API versioning (`grasp_preshaping_api_version()` returns 5) to detect ABI mismatches
4. Bridge node subscribes to `/hand_pose`, `/hand_twist`, `/segmentation/object_cloud`
5. Exposes a ROS2 service `/grasp_preshaping/compute_grasp` (Trigger type)
6. Publishes finger commands, wrist rotation, target pose, grasp type

This should be 1-2 paragraphs plus a node graph figure. The typst version had too much detail here.

---

## Part C: Factual Corrections Needed in Tex

1. **Pipeline order**: Tex is correct (SQ before TSDF). Typst was wrong.

2. **Optimizer**: The tex says "Levenberg-Marquardt or maybe something simpler like Newton." It's Gauss-Newton with LM damping — a specific hybrid. The rotation is locked from PCA, not optimized.

3. **Taubin distance**: Not "Newton with tricks." It's `F/||∇F||` plus one Newton refinement step. Simple and well-defined.

4. **Sign determination**: More nuanced than described. Uses per-camera ray-based voting with alignment threshold, plus SQ tiebreaker for opposing cameras.

5. **Blending domain**: Still exists. Two regimes (agree/disagree) with smoothstep transitions.

6. **Noise in prediction**: Noise is added in twist space before propagation, not after. Functionally equivalent for small perturbations.

---

## Part D: Recommendations

### High Priority (affects report quality)

1. **Add the tiered scoring table** — this is the clearest way to explain the scoring system. Include the 4 tiers with conditions and score ranges.

2. **Explain the force closure proxy honestly** — frame it as a deliberate design choice, not a limitation. The proxy measures normal balance, which is the key property for grasp stability without requiring mass information.

3. **Do the SQ ablation study** — this is the most valuable evaluation result and directly validates the main contribution.

4. **Fix the TSDF design decisions** — the primary reason for BFS-based TSDF is that the input is already a segmented point cloud (not raw depth maps), making projective TSDF impractical.

5. **Include the 5 SMC enhancements** with the specific problem each one solves.

### Medium Priority (improves clarity)

6. **Add the Morton code explanation** — brief but important for understanding the spatial data structure.

7. **Clarify the SQ fitting pipeline**: PCA → OBB → parallel template matching (3 templates via rayon) → Gauss-Newton (6-DOF: scale + translation, rotation locked).

8. **Mention the Rust + rayon parallelization** briefly in the architecture section.

9. **Include the combined score formula** with the 3× weight on contact_score and explain why.

### Low Priority (nice to have)

10. **Implement true wrench metric** — only if time permits. The current proxy is defensible.

11. **Add convergence plots** from debug dumps showing how the best score evolves over SMC iterations.

12. **Grasp variability analysis** — run on multiple objects and report type distributions.

---

## Part E: Feasibility Assessment of Proposed Changes

| Proposed Change | Feasibility | Effort | Impact | Recommendation |
|----------------|-------------|--------|--------|----------------|
| True wrench metric | Moderate | 2-4h | Medium | Do if time permits |
| SQ ablation study | High | 1-2h | High | **Do this** |
| Timing benchmarks | High | 1h | High | **Do this** |
| Grasp variability script | High | 2-3h | Medium | Worth doing |
| Simpler baseline comparison | Low (requires implementation) | 4-8h | Medium | Skip for report, mention conceptually |
| Convergence analysis | High (data exists in debug dumps) | 1h | Medium | **Do this** |
