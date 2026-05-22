# Grasp Preshaping Pipeline — Focused Implementation Plan (v2)

## Objective

Implement five targeted changes to the grasp preshaping pipeline, addressing grasp quality, self-collisions, start-position validation, unified sampling with wrist orientation, and parallelism.

---

## Codebase Context

Key files and their roles:
- `scripts/model.py` — Generates the finger contact LUT from the Mia hand URDF using Pinocchio FK
- `src/lut_helper.rs` — Loads the LUT (dual-quaternion tables for index, MRL, thumb add/abd, palm)
- `src/predictor.rs` — Samples future hand poses from current pose + twist + noise
- `src/planner.rs` — Sweeps finger contacts through TSDF, binary searches for collision, scores grasps
- `src/c_api.rs` — Orchestrates pipeline: predict ROI → build TSDF → score all samples → select best
- `src/pointcloud_helper.rs` — Point cloud pruning, Morton sorting, TSDF construction
- `include/grasp_preshaping/ffi_types.hpp` — C++ FFI struct definitions (must stay in sync with Rust)
- `nodes/preshaping_service_bridge_node.cpp` — ROS 2 node that calls Rust via FFI, publishes joint commands

---

## Change 1: Grasp Scoring — Reject Single-Finger Grasps

### Problem

In `planner.rs:259-301`, `sweep_for_collision` returns on the **first** collision from any finger. The subsequent scoring (`find_active_contacts` at `planner.rs:375-416`) does not penalize having too few contacts. A single finger touching the object can produce a non-trivial combined score because:

- `alignment_score` averages over the 1 contact (can be high)
- `force_closure_score` with 1 contact gives `(1 - |centroid|)` which can be moderate
- `combined_score` = weighted sum of probability + alignment + force_closure

This causes the planner to accept bad grasps.

### Approach

Add a minimum contact count requirement per grasp type. If fewer contacts are found, the grasp gets a zero quality score. This is the simpler and faster approach (vs. stepping fingers until they all collide), and the quality tradeoff is manageable.

### Implementation

- [ ] **1.1** Add `min_contacts: usize` field to `GraspSpec` in `planner.rs:60-63`. Set values per grasp type:
  - `cylindrical_spec()`: `min_contacts = 3` (thumb + at least 2 fingers)
  - `pinch_spec()`: `min_contacts = 2` (thumb + index)
  - `lateral_spec()`: `min_contacts = 2` (thumb side + index side)

  **Rationale**: These thresholds match the physical requirements of each grasp type. A cylindrical grasp needs at least 3 fingers wrapping around the object. Pinch and lateral need exactly 2 opposing contacts.

- [ ] **1.2** Modify `score_grasp` at `planner.rs:252-301` to check active contact count after `find_active_contacts`. If `active.len() < spec.min_contacts`, return a result with `alignment_score: 0.0` and `force_closure_score: 0.0` (keeping `found_collision: true` so it doesn't get `-Infinity` from `score_all_samples`, but the zero scores ensure it ranks below any valid grasp).

  Wait — actually, re-examining `score_all_samples` at `c_api.rs:184-185`:
  ```rust
  let combined = if result.found_collision {
      result.combined_score(&weights, sp.sample_probability)
  } else {
      f64::NEG_INFINITY
  };
  ```
  If we set `alignment_score = 0` and `force_closure_score = 0`, the combined score becomes `w_probability * probability / denom`, which is still positive. Better to set `found_collision: false` for insufficient contacts, which gives `-Infinity`. But that conflates "no collision found" with "collision found but insufficient contacts."

  **Better approach**: Add a `contact_count_score` that multiplies the combined score. If `active.len() < min_contacts`, `contact_count_score = 0.0`, making the combined score zero. This cleanly separates the concerns.

- [ ] **1.3** Add a `contact_count_score` field to `GraspScoreResult` at `planner.rs:7-12`. Compute it as `min(active.len(), spec.min_contacts) as f64 / spec.min_contacts as f64`, clamped to [0, 1]. This gives a soft penalty for fewer contacts and a bonus for more.

- [ ] **1.4** Integrate `contact_count_score` into `combined_score` at `planner.rs:15-24`. Add a new weight `w_contact_count` to `GraspWeights` (default 1.5 — higher than other weights to strongly penalize low contact counts). The formula becomes:
  ```
  (w_probability * probability + w_alignment * alignment + w_force_closure * force_closure + w_contact_count * contact_count) / denom
  ```

- [ ] **1.5** Add config constant `DEFAULT_W_CONTACT_COUNT: f64 = 1.5` in `config.rs`.

### Verification
- A 1-finger grasp should score significantly lower than a 3+ finger grasp
- The planner should never select a single-finger grasp when multi-finger alternatives exist
- Existing grasp type specs remain unchanged in their contact definitions

### Risks
1. **Small objects**: Fewer fingers naturally contact small objects. Mitigation: `min_contacts = 2` for pinch/lateral handles this; cylindrical with `min_contacts = 3` may need lowering to 2 if small objects are common.

---

## Change 2: Self-Collisions via LUT Max Closure

### Problem

The planner has no self-collision checking. It can recommend closure amounts where fingers interpenetrate each other or the palm, producing physically impossible grasps.

### Approach

Pre-compute maximum valid closure amounts per grasp type using Pinocchio's collision checking in `model.py`, store them in the LUT, and clamp the sweep range in the Rust planner.

**Why this approach**: Runtime self-collision checking would require a full hand mesh model in the TSDF or a separate collision checker. Pre-sampling is simpler, faster at runtime, and stays true to the physical hand model since the limits come from the actual kinematics.

### Implementation

- [ ] **2.1** In `model.py`, add a function `compute_max_closure_for_grasp_type(thumb_opp_mode, finger_coupling, resolution=100)` that:
  1. Iterates closure from 0.0 to 1.0 in `resolution` steps
  2. At each step, builds `q_full` using the appropriate coupling (same as the planner's grasp spec)
  3. Calls `pin.computeCollisions(model, data, geom_model, geom_data, q_full)` to check for self-collisions
  4. Returns the closure value just before the first self-collision

  **Rationale for using Pinocchio collision checking**: The `geom_model` is already loaded at `model.py:131` with `pin.GeometryType.COLLISION`. Pinocchio's collision checking uses the same geometry primitives that define the hand's collision volumes. This is consistent with how the LUT is generated.

- [ ] **2.2** Define the three grasp type configurations for self-collision checking:
  - **Cylindrical**: thumb_opp_mode=1, q_active = `[s, s, s]` (all fingers coupled)
  - **Pinch**: thumb_opp_mode=1, q_active = `[s, s, 0.0]` (thumb + index coupled, MRL at 0)
  - **Lateral**: thumb_opp_mode=0, q_active = `[s, s, 0.0]` (thumb + index coupled, MRL at 0)

  Note: In the planner's `cylindrical_spec`, all coupled fingers share the same control value. The q_active mapping in model.py line 361-395 shows: `[thumb_flex, tisit_motor, mrl_flex]`. For cylindrical where all move together, `q_active = [s, s, s]` means thumb flex = s, TISIT motor = s (which drives index flex = s), MRL flex = s. This matches the planner's coupled behavior.

- [ ] **2.3** Add the max closure values to the LUT npz file in `generate_contact_lut()`:
  ```python
  max_closure_cylindrical = compute_max_closure_for_grasp_type(thumb_opp_mode=1, ...)
  max_closure_pinch = compute_max_closure_for_grasp_type(thumb_opp_mode=1, ...)
  max_closure_lateral = compute_max_closure_for_grasp_type(thumb_opp_mode=0, ...)
  
  np.savez(...,
      max_closure_cylindrical=np.array([max_closure_cylindrical], dtype=np.float32),
      max_closure_pinch=np.array([max_closure_pinch], dtype=np.float32),
      max_closure_lateral=np.array([max_closure_lateral], dtype=np.float32),
  )
  ```

- [ ] **2.4** In `lut_helper.rs`, add fields to `FingerLUT` for the max closure values:
  ```rust
  pub struct FingerLUT {
      // existing fields...
      max_closure_cylindrical: f64,
      max_closure_pinch: f64,
      max_closure_lateral: f64,
  }
  ```
  Load them from the npz in `FingerLUT::load()`. Provide a public getter:
  ```rust
  pub fn max_closure(&self, grasp_type_index: usize) -> f64
  ```

- [ ] **2.5** In `planner.rs`, add `max_closure: f64` to `GraspSpec`. Set it when constructing specs in `c_api.rs` (read from the LUT).

- [ ] **2.6** Modify `sweep_for_collision` at `planner.rs:303-333` to clamp the sweep range. Convert `max_closure` to a sample index: `max_sample = (max_closure * (resolution - 1) as f64).floor() as usize`. The sweep loop becomes `for sample in 0..=max_sample`.

- [ ] **2.7** Clamp `refine_binary`'s upper bound to `max_closure` as well.

- [ ] **2.8** Update `verify_lut.py` to check the new fields exist and are in valid range (0, 1].

### Verification
- No recommended grasp exceeds the pre-computed max closure for its type
- Max closure values are consistent with the physical hand model
- Grasp quality does not degrade (limits only exclude impossible configurations)

### Risks
1. **Conservative limits**: Pre-computed limits may be too conservative if the collision geometry is approximate. Mitigation: add a safety margin parameter (e.g., multiply max_closure by 0.95).
2. **Object-dependent collisions**: Self-collision limits are object-independent (they depend only on hand geometry). This is correct — self-collisions happen regardless of what's being grasped.

---

## Change 3: Invalidate Start-Position Collisions

### Problem

At `planner.rs:303-333`, `sweep_for_collision` checks locked points at sample 0 first. If they collide, it returns `Some(0)`. In `score_grasp` at `planner.rs:266-271`, this maps to `found_collision: true` with zero scores, giving a combined score of `0.0 * probability / denom = 0.0`. This is not `-Infinity`, so if no better grasp exists, this zero-score grasp can be selected as the "best."

Additionally, for coupled points, the sweep starts at sample 0. If a finger is already inside the object at the open position, the sweep returns `Some(0)` immediately — same problem.

### Approach

When any point (locked or coupled) collides at the open position (sample 0 / control 0.0), treat the entire pose as invalid by returning `None` from `sweep_for_collision`. This maps to `found_collision: false` → combined score `-Infinity` → never selected.

### Implementation

- [ ] **3.1** Modify `sweep_for_collision` at `planner.rs:303-333` to check **all** sweep points (both locked and coupled) at sample 0 before starting the sweep. Add a new pre-check block:

  ```
  // Check if the hand is already colliding at the open position
  for sp in sweep_points {
      let p = match sp.flex {
          Flex::Coupled => pos_at_sample(lut, sp.contact, 0, base),
          Flex::Locked(locked_s) => pos_at_sample(lut, sp.contact, locked_s, base),
      };
      if tsdf.get_distance(p.x, p.y, p.z) < collision_tol {
          return None;  // Start position is in collision — invalid pose
      }
  }
  ```

  This replaces the existing locked-point check (lines 312-319) and extends it to coupled points at sample 0.

- [ ] **3.2** The existing `match` in `score_grasp` already handles `None` correctly at `planner.rs:260-265`:
  ```rust
  None => GraspScoreResult {
      closure_amount: 0.0,
      alignment_score: 0.0,
      force_closure_score: 0.0,
      found_collision: false,  // ← this is key
  },
  ```
  And `score_all_samples` gives `-Infinity` for `found_collision: false`. No change needed here.

### Verification
- Poses where the hand penetrates the object at the open position get score `-Infinity`
- These poses are never selected as the best grasp
- The planner continues searching other samples

### Risks
1. **Too many rejections when object is very close**: If the hand is near the object, many samples may be rejected. This is correct behavior — you can't start a grasp from inside the object.

---

## Change 4: Unified Sampling with Wrist Orientation

### Problem

The current sampling approach:
1. `predictor.rs` generates N position samples from twist + noise
2. `c_api.rs` evaluates each sample against all 3 grasp types → N×3 evaluations
3. No explicit orientation sampling — only twist-induced orientation perturbation
4. The output doesn't include wrist orientation

The user wants:
- N samples where each sample specifies (position, orientation, grasp_type) — exactly N evaluations
- Better visualization (no duplicate positions from the ×3 multiplication)
- Random sampling across all dimensions simultaneously
- Wrist orientation returned as output and published via ROS

### Approach

Refactor sampling so each `SampledPose` includes a randomly assigned grasp type and an explicit orientation perturbation. The scoring loop evaluates each sample exactly once. Add orientation to the FFI response.

### Implementation

- [ ] **4.1** Modify `SampledPose` in `predictor.rs:8-12` to include the grasp type:
  ```rust
  pub struct SampledPose {
      pub pose: DualQuaternion,
      pub sample_probability: f64,
      pub grasp_type_index: usize,  // 0=cylindrical, 1=pinch, 2=lateral
  }
  ```

- [ ] **4.2** In `sample_future_poses` at `predictor.rs:150-176`, add two things per sample:
  1. **Random grasp type**: `let grasp_type_index: usize = rng.random_range(0..3);`
  2. **Random orientation perturbation**: Generate a small random rotation and compose it with the twist displacement. Use a random axis (uniform on unit sphere) and random angle (uniform in `[-WRIST_ORIENT_RANGE_RAD, +WRIST_ORIENT_RANGE_RAD]`).

  The pose computation becomes:
  ```
  displacement_dq = DualQuaternion::from_se3(&twist_to_se3(&omega, &v))
  orient_dq = DualQuaternion::from_se3(&random_small_rotation(rng))
  future_pose = current_pose.multiply(&displacement_dq).multiply(&orient_dq)
  ```

  **Rationale**: Composing the orientation perturbation after the displacement keeps the position sampling unchanged (physically motivated by the twist) while adding independent orientation exploration. The `multiply` applies the rotation in the local frame of the displaced pose.

- [ ] **4.3** Add helper function `random_small_rotation(rng) -> Matrix4<f64>` in `predictor.rs` that:
  1. Samples a uniform random unit vector (axis) using the rejection method or Marsaglia
  2. Samples a uniform random angle in `[-config::WRIST_ORIENT_RANGE_RAD, +config::WRIST_ORIENT_RANGE_RAD]`
  3. Returns the SE(3) matrix for this rotation (no translation)

- [ ] **4.4** Add config constant `WRIST_ORIENT_RANGE_RAD: f64 = 0.52` (≈ ±30°) in `config.rs`.

- [ ] **4.5** Refactor `score_all_samples` in `c_api.rs:171-198` to use the grasp type from each sample:
  ```rust
  fn score_all_samples(lut, tsdf, samples, collision_tol) -> Vec<ScoredGrasp> {
      let weights = GraspWeights::default();
      samples.iter().map(|sp| {
          let base_transform = sp.pose.to_se3();
          let gs = &SCORERS[sp.grasp_type_index];
          let result = (gs.scorer)(lut, tsdf, &base_transform, collision_tol);
          let combined = if result.found_collision {
              result.combined_score(&weights, sp.sample_probability)
          } else {
              f64::NEG_INFINITY
          };
          ScoredGrasp { grasp_type: gs.grasp_type, result, combined }
      }).collect()
  }
  ```

  **Key change**: The inner loop over `SCORERS` is removed. Each sample is evaluated exactly once with its assigned grasp type. This is simpler, more parallelizable, and produces N unique position+orientation samples in the visualization.

- [ ] **4.6** Update the debug export in `compute_from_request` at `c_api.rs:357-386` to account for the new `grasp_type_index` field in `SampledPose`. The `sample_index` in the export is now just the array index (no more dividing by `SCORERS.len()`).

- [ ] **4.7** Update the visualization script `visualize_grasp_debug.py` at line 118 to handle the new data format. Since each row now represents a unique sample (not a sample×grasp-type pair), no changes to the parsing logic are needed — the `sample_index` column just directly indexes the sample.

- [ ] **4.8** Add orientation output to `GraspComputeResponseFFI` in both `c_api.rs:78-88` and `ffi_types.hpp:69-78`. Append four `f64` fields: `wrist_qx, wrist_qy, wrist_qz, wrist_qw`.

  **FFI layout change**: Appending fields at the end of the struct is safe for the C++ side as long as both sides are rebuilt together. Bump the API version from 1 to 2 in `grasp_preshaping_api_version()`.

- [ ] **4.9** Update `ComputeOutput` at `c_api.rs:135-142` to include `wrist_orientation: [f64; 4]` (quaternion xyzw).

- [ ] **4.10** In `compute_from_request`, extract the orientation from the best grasp's `SampledPose`. Convert the `DualQuaternion` to a quaternion and populate the response fields:
  ```rust
  let se3 = best_sample.pose.to_se3();
  let rot = se3.fixed_view::<3,3>(0,0);
  let uq = UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(rot.clone_owned()));
  output.wrist_orientation = [uq.quaternion().i, uq.quaternion().j, uq.quaternion().k, uq.quaternion().w];
  ```

- [ ] **4.11** In `preshaping_service_bridge_node.cpp`, read the new orientation fields from `ffi_response` and publish on a new topic `/grasp_preshaping/wrist_orientation` as `geometry_msgs::msg::PoseStamped`. Also include the orientation in the response message string.

- [ ] **4.12** Update the debug export to include wrist orientation in the scored grasps output (expand the per-row columns from 24 to 28 to include qx, qy, qz, qw).

### Verification
- N samples produce exactly N scored grasps (not N×3)
- Each sample has a unique position in the visualization
- The response includes a valid quaternion for the recommended wrist orientation
- The ROS node publishes wrist orientation
- The debug visualization shows orientation spread
- API version is bumped to 2

### Risks
1. **Reduced per-position coverage**: Previously each position was tested with all 3 grasp types. Now each position is tested with only 1 random type. Mitigation: with 1000 samples, each grasp type gets ~333 evaluations on average, which should be sufficient. If needed, increase `PREDICTION_SAMPLES`.
2. **FFI struct change**: Requires coordinated rebuild of Rust and C++. Mitigation: API version bump provides a runtime check.

---

## Change 5: Rayon Parallelism for Scoring (High-Impact Only)

### Problem

`score_all_samples` in `c_api.rs:171-198` is a sequential loop over all samples. With the unified sampling change (Change 4), each sample is fully independent — making this trivially parallelizable.

### Approach

Parallelize the scoring loop with Rayon. This is the single highest-impact parallelization opportunity because:
1. Scoring involves TSDF lookups + binary search per sample — computationally non-trivial
2. Each sample is fully independent (reads only shared immutable TSDF + LUT)
3. With 1000 samples, the work distribution is even

### Implementation

- [ ] **5.1** After Change 4 is implemented, refactor `score_all_samples` to use `par_iter`:
  ```rust
  fn score_all_samples(lut: &FingerLUT, tsdf: &Tsdf, samples: &[SampledPose], collision_tol: f32) -> Vec<ScoredGrasp> {
      let weights = GraspWeights::default();
      samples.par_iter().map(|sp| {
          let base_transform = sp.pose.to_se3();
          let gs = &SCORERS[sp.grasp_type_index];
          let result = (gs.scorer)(lut, tsdf, &base_transform, collision_tol);
          let combined = if result.found_collision {
              result.combined_score(&weights, sp.sample_probability)
          } else {
              f64::NEG_INFINITY
          };
          ScoredGrasp { grasp_type: gs.grasp_type, result, combined }
      }).collect()
  }
  ```

  **Safety**: `lut` and `tsdf` are immutable references — Rayon handles shared reads safely. `GraspWeights` is `Copy` and cheap. Each `ScoredGrasp` is independent.

- [ ] **5.2** Verify that `SCORERS` (a `const` slice of `GraspScorer`) is safely shared across threads. Since it's a static slice of structs containing only a `GraspType` (Copy) and a function pointer, this is fine.

- [ ] **5.3** Run the existing `bench_full_pipeline` benchmark to measure speedup. Target: 2-4x on 4+ cores.

### Verification
- Parallelized scoring produces identical results to sequential scoring
- Measurable speedup on multi-core systems
- No data races (enforced by Rust's borrow checker + Rayon's API)

### Risks
1. **Rayon overhead for small sample counts**: With 1000 samples, overhead is negligible. If samples are reduced to <100, may not be worth it. Mitigation: Rayon has a minimum work granularity that handles this automatically.

---

## Implementation Order

The changes should be implemented in this order due to dependencies:

1. **Change 1** (Grasp scoring) — Independent, highest correctness impact
2. **Change 3** (Start position collisions) — Independent, high correctness impact
3. **Change 2** (Self-collisions via LUT) — Requires model.py changes first, then Rust changes
4. **Change 4** (Unified sampling + wrist orientation) — Most invasive, touches predictor, planner, c_api, FFI, and C++ bridge
5. **Change 5** (Rayon) — Depends on Change 4's refactored `score_all_samples`

Changes 1 and 3 can be done in parallel. Change 2 requires the LUT regeneration pipeline to be working. Change 4 is the largest change and should be done carefully with the API version bump.
