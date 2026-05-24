# Grasp Preshaping Pipeline — Focused Implementation Plan v2

## Objective

Fix correctness bugs (single-finger grasps, start-position collisions), add self-collision limits via the LUT, unify the sampling loop with single-axis wrist rotation, and parallelize the scoring loop. This plan addresses 5 issues in priority order.

---

## Change 1: Contact Count Penalty for Grasp Scoring

### Problem

At `planner.rs:259-301`, `sweep_for_collision` returns on the **first** collision from any single finger. The subsequent scoring at `planner.rs:375-416` (`find_active_contacts` → `compute_alignment` → `compute_force_closure`) has no concept of minimum contact count. A single-finger touch can produce moderate alignment and force closure scores, making it appear as a valid grasp.

### Approach

Add a `contact_count_score` component to the combined score. This penalizes grasps with too few active contacts without changing the sweep logic.

### Implementation

- [ ] **1.1** Add `min_contacts` field to `GraspSpec` in `planner.rs:60-63`. Set per grasp type:
  - Cylindrical: `min_contacts = 3` (thumb + at least 2 fingers)
  - Pinch: `min_contacts = 2` (thumb + index)
  - Lateral: `min_contacts = 2` (thumb + index side)
  - Rationale: These thresholds reflect the minimum contacts needed for a stable grasp of each type.

- [ ] **1.2** Add `contact_count` field to `GraspScoreResult` in `planner.rs:7-12` and update `combined_score` at `planner.rs:15-24` to include a `w_contact_count` weight. The contact count score should be computed as: `min(1.0, active_contacts.len() as f64 / min_contacts as f64)`. If `active_contacts.is_empty()`, the score is 0.0. Rationale: This creates a smooth ramp — a grasp with 1 contact when 3 are needed scores 0.33, not 0.0, allowing partial progress to be visible while still heavily penalizing insufficient contacts.

- [ ] **1.3** Add `w_contact_count: f64` to `GraspWeights` at `planner.rs:27-32` with default value `1.5` (slightly higher than other weights to strongly prefer multi-contact grasps). Update the `combined_score` denominator to include this weight.

- [ ] **1.4** Update the three `GraspSpec` constructors (`cylindrical_spec`, `pinch_spec`, `lateral_spec`) at `planner.rs:98-250` to include the `min_contacts` field.

- [ ] **1.5** Update `score_grasp` at `planner.rs:252-301` to pass `min_contacts` through and compute `contact_count_score` from the active contacts vector length.

- [ ] **1.6** Update debug export at `c_api.rs:358-386` to include the contact count score in the debug dump. Add a new column to the `scored_grasps` row format (bumping from 24 to 25 columns). Update the comment at `debug_export.rs:189-199` accordingly.

- [ ] **1.7** Update `visualize_grasp_debug.py` to parse the new column if present (backward-compatible with old dumps).

### Verification Criteria

- A grasp with only 1 finger in contact should never have a higher combined score than one with 3+ contacts at the same position.
- Existing test dumps should still load (backward-compatible debug format).
- The cylindrical grasp type should require at least 3 contacts for a score > 0.66.

### Potential Risks

1. **Weight tuning** — The 1.5 default may be too aggressive or too lenient. Mitigation: The weight is in `GraspWeights::default()` and can be tuned without code changes to the scoring logic.

---

## Change 2: Invalidate Start-Position Collisions

### Problem

At `planner.rs:266-271`, when a locked point (palm) or coupled point collides at sample 0 (open hand position), `sweep_for_collision` returns `Some(0)`. This maps to `GraspScoreResult { closure_amount: 0.0, ..., found_collision: true }` with zero alignment/force_closure scores. The `combined_score` at `c_api.rs:184` computes `0.0 / denom = 0.0`, which is **not** `-Infinity`. If no better grasp is found, this zero-score grasp "wins" and gets published.

### Approach

Return `None` from `score_grasp` when any point collides at the open position. This makes the combined score `-Infinity`, ensuring these poses are never selected.

### Implementation

- [ ] **2.1** Modify `score_grasp` at `planner.rs:252-301` to return `Option<GraspScoreResult>` instead of `GraspScoreResult`. When `sweep_for_collision` returns `Some(0)`, return `None` (invalid pose). Rationale: If the hand is already inside the object at the open position, no valid grasp can be formed from this pose — the palm is penetrating the object.

- [ ] **2.2** Update the three public scorer functions (`score_cylindrical`, `score_pinch`, `score_lateral`) at `planner.rs:65-96` to return `Option<GraspScoreResult>`.

- [ ] **2.3** Update the `ScorerFn` type alias at `c_api.rs:123-128` to return `Option<GraspScoreResult>`.

- [ ] **2.4** Update `score_all_samples` at `c_api.rs:171-198` to handle `None` results — push `ScoredGrasp` with `combined = f64::NEG_INFINITY` and `found_collision = false` when the scorer returns `None`. This is consistent with the existing handling for no-collision results.

### Verification Criteria

- Any sample where the palm collides with the object at the open position should produce `combined_score = -Infinity`.
- The best grasp should never be a start-position collision.
- Existing functionality for valid grasps should be unchanged.

### Potential Risks

1. **Too aggressive filtering** — For very large objects that fill the ROI, many poses may have palm collisions. Mitigation: This is correct behavior — if the palm can't approach the object without collision, the grasp is physically impossible. The sampling should naturally find poses that approach from feasible angles.

---

## Change 3: Self-Collision Limits via LUT Max Closure

### Problem

No self-collision checking exists. At high closure amounts, fingers can interpenetrate each other (e.g., thumb crossing through index finger in cylindrical grasp). The current sweep goes from sample 0 to the full LUT resolution, unaware of physical finger limits.

### Approach

Pre-compute the maximum closure amount before self-collision for each grasp type using Pinocchio's collision checking in `model.py`. Store these limits in the LUT npz file. Load them in `lut_helper.rs` and clamp the sweep range in `planner.rs`.

### Implementation

- [ ] **3.1** Add a `compute_max_closure_per_grasp_type()` function to `model.py` that:
  - For each grasp type (cylindrical, pinch, lateral), iterates closure from 0.0 to 1.0 in fine steps (e.g., 100 steps).
  - At each step, calls `pin.computeCollisions(model, data, geom_model, geom_data, q_full, False)` to check for self-collisions between finger geometries.
  - Records the last closure value before any collision is detected.
  - Returns a dict: `{"cylindrical": 0.85, "pinch": 0.92, "lateral": 0.78}` (example values).
  - Rationale: This is an offline pre-computation that captures the physical limits of the hand model. It's done once during LUT generation and doesn't affect runtime performance.

- [ ] **3.2** Update `generate_contact_lut()` in `model.py` at line 498 to save the max closure values as a new array `grasp_max_closure` in the npz file: `np.array([cyl_max, pinch_max, lateral_max], dtype=np.float32)`.

- [ ] **3.3** Add `grasp_max_closure: [f64; 3]` field to `FingerLUT` in `lut_helper.rs:204-211`. Load it from the npz in `FingerLUT::load()` at `lut_helper.rs:219-254`. Add a public accessor `fn get_grasp_max_closure(&self, grasp_type_index: usize) -> f64`.

- [ ] **3.4** Add `max_closure: f64` parameter to `score_grasp` in `planner.rs:252-258`. In `sweep_for_collision`, clamp the sweep range: instead of sweeping `0..resolution`, compute the max sample as `lut.get_sample(max_closure)` and sweep `0..=max_sample`. Similarly clamp `refine_binary` to not exceed `max_closure`. Rationale: This prevents the planner from considering closure amounts where fingers would self-collide, without any runtime collision checking overhead.

- [ ] **3.5** Update the three public scorers and `score_all_samples` in `c_api.rs` to pass the appropriate max closure from the LUT based on grasp type.

### Verification Criteria

- The sweep should never exceed the pre-computed max closure for each grasp type.
- A cylindrical grasp at the Mia hand's physical limit should stop before thumb-index interpenetration.
- The LUT npz file should contain a `grasp_max_closure` array of length 3.

### Potential Risks

1. **Conservative limits** — The pre-computed limits are for the hand in free space. When grasping an object, the actual collision-free closure may be lower. Mitigation: This is acceptable — the max closure is an upper bound. The object collision check (TSDF sweep) will still find the actual contact point, which is always ≤ the max closure.

2. **Grasp-type-specific geometry** — Different grasp types activate different thumb modes (adduction vs. abduction), which changes the self-collision geometry. Mitigation: The function computes limits per grasp type, using the correct thumb opposition mode for each.

---

## Change 4: Unified Sampling with Single-Axis Wrist Rotation

### Problem

The current flow at `c_api.rs:171-198` generates N position samples × 3 grasp types = 3N evaluations. Each sample gets the same position but different grasp types, creating visual clusters. There is no wrist orientation sampling. The FFI response has no orientation output.

### Clarification on Wrist Axis

The Mia hand's wrist rotates around the **forearm axis** (supination/pronation). In the hand's local coordinate frame (as defined by the URDF at `mia_hand_flat.urdf`), this is the **Y axis** — the axis along the finger direction. The rotation range is ±90° (180° total).

This was confirmed by examining:
- The MuJoCo scene places `palm_r` at `quat="0.707388 0.706825 0 0"` (≈90° rotation around X), mapping local Y → world Z (up).
- The hand approaches objects from above, so wrist rotation around local Y (the approach/forearm axis) rotates the hand in the horizontal plane — exactly supination/pronation.

### Approach

Each `SampledPose` gets a random grasp type AND a random Y-axis rotation perturbation. The scoring loop evaluates each sample exactly once. Add the resulting orientation quaternion to the FFI response and ROS output.

### Implementation

- [ ] **4.1** Add `grasp_type: GraspType` and `wrist_angle: f64` fields to `SampledPose` in `predictor.rs:8-12`. Rationale: Each sample is now a complete specification of position + orientation + grasp type.

- [ ] **4.2** Modify `sample_future_poses` in `predictor.rs:150-176` to:
  - For each sample, randomly pick a grasp type (uniform over 3 types).
  - Randomly pick a wrist angle from `[-π/2, π/2]` (uniform).
  - Apply the wrist rotation as a Y-axis rotation to the sampled pose's dual quaternion: create a rotation DQ from the angle, and multiply `future_pose = wrist_rotation_dq * future_pose`.
  - Rationale: Y-axis rotation in the hand frame corresponds to supination/pronation. Uniform sampling over the full range ensures good coverage.

- [ ] **4.3** Remove the inner `for gs in SCORERS` loop from `score_all_samples` at `c_api.rs:171-198`. Instead, for each sample, look up `sp.grasp_type` and call only the corresponding scorer. This reduces evaluations from 3N to N. Rationale: N samples = N unique evaluations, better visualization spread, naturally parallelizable.

- [ ] **4.4** Update `ScoredGrasp` at `c_api.rs:117-121` to store the wrist angle from the sample.

- [ ] **4.5** Add orientation quaternion fields to `GraspComputeResponseFFI` in both `c_api.rs:79-88` and `ffi_types.hpp:69-78`:
  - Add `qx: f64, qy: f64, qz: f64, qw: f64` fields (4 doubles).
  - The orientation should be the **input pose orientation** composed with the **sampled wrist rotation** around Y. Compute it from the winning sample's dual quaternion real part.
  - Bump `grasp_preshaping_api_version()` from 1 to 2 at `c_api.rs:458-460`.
  - Rationale: The caller needs to know the wrist orientation to command the real hardware.

- [ ] **4.6** Add `wrist_angle: f64` to `ComputeOutput` at `c_api.rs:135-142` and propagate it from the best grasp through `compute_from_request` at `c_api.rs:312-440`.

- [ ] **4.7** Update `preshaping_service_bridge_node.cpp` to:
  - Read the new quaternion fields from `ffi_response`.
  - Publish the wrist orientation on a new topic (e.g., `/grasp_preshaping/wrist_orientation` as `geometry_msgs::msg::Quaternion`).
  - Include the orientation in the service response message.
  - Rationale: The ROS ecosystem needs the orientation for visualization and hardware control.

- [ ] **4.8** Update debug export at `c_api.rs:358-386` to include the wrist angle and grasp type per sample in the debug dump. The `sample_index` field no longer needs division by `SCORERS.len()` — each sample is one evaluation.

- [ ] **4.9** Update `visualize_grasp_debug.py` to:
  - Parse the wrist angle if present in new dump format.
  - Visualize the wrist orientation as a rotated coordinate frame at each grasp position.
  - Remove the `sample_index = total_index / 3` assumption.

### Verification Criteria

- `PREDICTION_SAMPLES = 1000` should produce exactly 1000 scored grasps (not 3000).
- Each grasp should have a unique position + orientation combination.
- The wrist angle should be uniformly distributed in [-π/2, π/2].
- The FFI response should contain a valid quaternion representing the composed orientation.
- The ROS node should publish the wrist orientation.
- Visualization should show grasps spread across different positions (no 3x clustering).

### Potential Risks

1. **API breakage** — The FFI struct layout changes. Mitigation: The API version bump from 1→2 allows the C++ side to detect incompatibility. The bridge node should check the version and warn if it doesn't match.

2. **Fewer samples per grasp type** — With N=1000 and 3 types, each type gets ~333 samples on average instead of 1000. Mitigation: If this is a concern, `PREDICTION_SAMPLES` can be increased. The unified sampling actually improves coverage per sample since positions aren't triplicated.

3. **Y-axis convention** — If the hand frame's Y axis is not exactly the forearm axis due to URDF frame transforms. Mitigation: Verified from the MuJoCo scene — the `palm_r` body's local Y axis points along the fingers, which is the forearm/supination axis.

---

## Change 5: Rayon Parallelism for Scoring Loop

### Problem

After Change 4, each sample in `score_all_samples` is fully independent (unique grasp type, unique pose, unique orientation). The loop is embarrassingly parallel but currently sequential.

### Approach

Use `rayon::par_iter()` to parallelize the scoring loop. The TSDF and LUT are read-only during scoring, making this safe.

### Implementation

- [ ] **5.1** Refactor `score_all_samples` at `c_api.rs:171-198` to collect samples into a `Vec<(usize, &SampledPose)>` (index + reference), then use `par_iter()` to score each sample in parallel. Collect results into a `Vec<ScoredGrasp>`. Rationale: After Change 4, each sample maps to exactly one scorer call — no shared mutable state.

- [ ] **5.2** Ensure `ScoredGrasp` is `Send` (it already contains only `Copy` types + `GraspType` which is `Copy`, so this should be automatic).

- [ ] **5.3** Add `rayon` to the dependencies in `Cargo.toml` if not already present (it's already used in `pointcloud_helper.rs` so it should be there).

### Verification Criteria

- Results should be identical to sequential execution (deterministic scoring).
- On a 4-core machine, the scoring phase should be approximately 3-4x faster.
- No data races or panics under concurrent execution.

### Potential Risks

1. **Diminishing returns** — If the TSDF is small, the per-sample scoring cost is low and parallelization overhead may dominate. Mitigation: Rayon's work-stealing scheduler handles this well — if the work is too fine-grained, it falls back to sequential execution automatically.

---

## Implementation Order

1. **Changes 1 + 2** (independent, highest correctness impact, can be done in parallel)
2. **Change 3** (requires `model.py` + LUT regeneration + Rust loader changes)
3. **Change 4** (most invasive — touches predictor, planner, c_api, FFI structs, C++ bridge)
4. **Change 5** (depends on Change 4's refactored scoring loop)

## Files Modified Summary

| File | Changes |
|------|---------|
| `planner.rs` | Contact count scoring (1), start-position invalidation (2), max closure clamping (3), grasp type per sample (4) |
| `predictor.rs` | Grasp type + wrist angle in SampledPose (4) |
| `c_api.rs` | Unified scoring loop (4), parallel scoring (5), FFI response orientation (4), debug export updates (1, 4) |
| `config.rs` | Wrist rotation range constant (4) |
| `lut_helper.rs` | Load grasp_max_closure from npz (3) |
| `ffi_types.hpp` | Orientation fields in response struct (4) |
| `preshaping_service_bridge_node.cpp` | Publish wrist orientation (4) |
| `debug_export.rs` | Updated row format (1, 4) |
| `model.py` | Self-collision max closure computation (3) |
| `visualize_grasp_debug.py` | Parse new debug format (1, 4) |
