# Simplify grasp_preshaping: Feature-Parity Refactor

## Objective

Reduce complexity in the `dev/grasp_preshaping` package while preserving all runtime behavior. The main thrusts are: (1) remove redundant/trivial tests, (2) consolidate scattered constants into a single config file, (3) switch from topic-based covariance to a fixed constant, (4) make code more concise, and (5) add comments only where logic is genuinely convoluted.

---

## File-by-File Analysis

### Current state summary

| File | Lines | Role |
|---|---|---|
| `src/lib.rs` | 5 | Module root |
| `src/c_api.rs` | 493 | FFI entry, pipeline orchestration, dead code |
| `src/predictor.rs` | 500 | ROI prediction, duplicate constants, unused wrappers |
| `src/planner.rs` | 497 | Grasp scoring, identical weight factories |
| `src/pointcloud_helper.rs` | 806 | TSDF, point cloud, many trivial tests |
| `src/lut_helper.rs` | 729 | DQ math, LUT loading |
| `benches/pipeline.rs` | 225 | Benchmarks, duplicate constants |
| `nodes/preshaping_service_bridge_node.cpp` | 369 | ROS2 bridge, subscribes to TwistWithCovarianceStamped |
| `include/grasp_preshaping/ffi_types.hpp` | 103 | C++ FFI types (includes 36-element covariance array) |
| `scripts/model.py` | 623 | LUT generator (no changes needed) |
| `scripts/verify_lut.py` | 577 | LUT verifier (no changes needed) |
| `scripts/hand_tip_visualizer.py` | 326 | Visualizer (no changes needed) |

---

## Implementation Plan

### Phase 1: Create centralized config file

- [ ] **1.1 Create `src/config.rs`** with all tunable/performance-relevant constants in one place. This is the single source of truth for all magic numbers. Contents:

  ```
  TSDF_RESOLUTION_M: f32 = 0.005
  TRUNCATION_CELLS: usize = 4
  COLLISION_TOL_M: f32 = 0.005
  PREDICTION_HORIZON_S: f64 = 5.0
  PREDICTION_SAMPLES: usize = 1000
  HAND_RADIUS_M: f64 = 0.05
  MIN_TSDF_DIM_M: f32 = 0.1
  MAX_TSDF_DIM_M: f32 = 0.3
  BINARY_SEARCH_TOL: f64 = 0.01
  RAY_ALIGNMENT_THRESHOLD: f32 = 0.8
  FIXED_TWIST_COV_OMEGA: [f64; 3] = [0.01, 0.01, 0.01]
  FIXED_TWIST_COV_V: [f64; 3] = [0.005, 0.005, 0.005]
  ```

  Rationale: All these values are currently scattered across `c_api.rs`, `predictor.rs`, `planner.rs`, `pointcloud_helper.rs`, and `benches/pipeline.rs`. Consolidating them makes tuning trivial and eliminates duplication.

- [ ] **1.2 Register `config` module in `src/lib.rs`** — add `pub mod config;`

### Phase 2: Switch to fixed covariance (biggest interface change)

- [ ] **2.1 Simplify `GraspTwistFFI` in `src/c_api.rs`** — remove the `covariance: [f64; 36]` field. The twist struct becomes just 6 floats (lx, ly, lz, ax, ay, az).

- [ ] **2.2 Simplify `GraspTwistFFI` in `include/grasp_preshaping/ffi_types.hpp`** — mirror the Rust change: remove `std::array<double, 36> covariance`.

- [ ] **2.3 Update `twist_to_runtime()` in `src/c_api.rs`** — instead of extracting diagonal from the passed-in covariance array, construct `TwistCovariance` from the fixed constants in `config.rs`.

- [ ] **2.4 Update `nodes/preshaping_service_bridge_node.cpp`** — remove the covariance copy loop (lines 222-224). The subscription type can remain `TwistWithCovarianceStamped` (to avoid breaking the publisher side) but the covariance is simply ignored. Add a brief comment explaining why.

  Rationale: The covariance was being passed through FFI from a ROS topic, but in practice the values are not dynamically meaningful. Hardcoding them removes an entire data path and simplifies the FFI boundary. The fixed values (`omega=[0.01,0.01,0.01]`, `v=[0.005,0.005,0.005]`) match what `TwistCovariance::dummy()` already uses.

### Phase 3: Remove dead code and make code more concise

#### `src/c_api.rs`

- [ ] **3.1 Remove dead variables** at lines 398-399 (`_best_loc`, `_sample_prob`) — they are computed but never used.

- [ ] **3.2 Remove duplicate constants** (lines 14-27) — replace all with imports from `config`.

- [ ] **3.3 Simplify `get_prediction_config()`** — reference config constants instead of re-declaring them.

- [ ] **3.4 Simplify `grasp_scorers()`** — the function creates a `Vec<GraspScorer>` on every call. Since the data is static, return a const array or use `OnceLock`.

#### `src/predictor.rs`

- [ ] **3.5 Remove duplicate constants** (`HAND_RADIUS`, `MIN_TSDF_DIM`, `MAX_TSDF_DIM` at lines 13-15) — use config imports instead.

- [ ] **3.6 Remove `predict_roi()` and `predict_roi_dummy()`** (lines 268-292) — thin wrappers that are not used in production (only in tests/benches). The bench can call `predict_roi_with_samples` directly.

- [ ] **3.7 Remove `Twist6::dummy()` and `TwistCovariance::dummy()`** — only used in tests/benches. The bench can construct these inline or use the config constants.

- [ ] **3.8 Remove `PredictionConfig::default()`** — the only real user is `c_api.rs` which already constructs it from config constants. Tests that use `Default` can construct explicitly.

- [ ] **3.9 Remove `TwistCovariance::zero()`** — only used in tests.

- [ ] **3.10 Remove `Twist6::zero()`** — only used in tests. Tests can construct `Twist6 { omega: Vector3::zeros(), v: Vector3::zeros() }` inline.

#### `src/planner.rs`

- [ ] **3.11 Collapse `GraspWeights` factories** — `cylindrical()`, `pinch()`, `lateral()` all return identical weights `(1.0, 1.0, 1.0)`. Replace with a single `const DEFAULT: GraspWeights` or just use `GraspWeights { w_probability: 1.0, w_alignment: 1.0, w_force_closure: 1.0 }` directly.

- [ ] **3.12 Import `BINARY_SEARCH_TOL` from config** instead of declaring locally.

#### `src/pointcloud_helper.rs`

- [ ] **3.13 Import `RAY_ALIGNMENT_THRESHOLD` from config** instead of declaring locally.

- [ ] **3.14 Remove `PointCloud::from_xyz_file()`** (lines 108-136) — not used in production, only in a test. File I/O for point clouds is not part of the core pipeline.

- [ ] **3.15 Remove `PointCloud::demo_sphere()`** (lines 138-151) — only used in benchmarks. The bench can define this locally or use a simpler fixture.

#### `benches/pipeline.rs`

- [ ] **3.16 Replace all locally-defined constants** with imports from `config` module.

### Phase 4: Remove unnecessary tests

Tests to **remove** (trivial, redundant, or testing stdlib behavior rather than invariants):

#### `src/predictor.rs` tests

- [ ] **4.1 Remove `twist_to_se3_identity`** — covered by `twist_to_se3_pure_translation` with zero vectors.
- [ ] **4.2 Remove `twist_to_se3_small_theta_uses_pure_translation`** — tests an epsilon branch; the branch is obvious from the code and unlikely to regress.
- [ ] **4.3 Remove `sample_future_poses_with_translation_spreads`** — stochastic test with weak assertions; flaky by nature.
- [ ] **4.4 Remove `predict_roi_with_nonzero_twist_produces_larger_aabb_than_zero`** — stochastic, tests an intuitive property rather than a code invariant.

#### `src/pointcloud_helper.rs` tests

- [ ] **4.5 Remove `prune_without_aabb_returns_clone`** — tests that `None` means no filter; obvious from the match arm.
- [ ] **4.6 Remove `aabb_from_points_computes_bounds`** — tests min/max reduction; trivially obvious.
- [ ] **4.7 Remove `aabb_inflate_expands_uniformly`** — tests addition/subtraction; trivially obvious.
- [ ] **4.8 Remove `prune_empty_yields_no_tsdf_points`** — tests empty filter result; trivially obvious.
- [ ] **4.9 Remove `from_xyz_file_loads_sphere`** — `from_xyz_file` is being removed (3.14).
- [ ] **4.10 Remove `from_xyz_file_missing_file_returns_error`** — tests file-not-found; trivially obvious, and `from_xyz_file` is being removed.
- [ ] **4.11 Remove `demo_sphere_generates_points`** — `demo_sphere` is being removed (3.15).

Tests to **keep** (testing real invariants and algorithms):
- All morton roundtrip tests
- All TSDF distance, truncation, sign, and normal tests
- AABB clip_max_dims tests (non-trivial anchoring logic)
- AABB enforce_min_dims test
- prune + tsdf integration test
- All lut_helper tests (DQ roundtrips, LUT lookups)
- predictor: `twist_to_se3_pure_translation`, `twist_to_se3_pure_rotation_z`, `twist_to_se3_screw_motion`, `sample_future_poses_zero_twist_stays_at_current`, `project_index_tips_includes_base_and_current_tip`, `predict_roi_dummy_returns_valid_aabb`

### Phase 5: Add comments only where convoluted

- [ ] **5.1 Add comment on morton code purpose** in `pointcloud_helper.rs` — explain *why* morton ordering is used (spatial locality for TSDF BFS traversal). The `split_by_3` / `decode_morton` functions are correct but the motivation is not obvious.

- [ ] **5.2 Add comment on TSDF sign-flip voting logic** in `pointcloud_helper.rs` around lines 434-479 — the behind_count/inside_votes threshold logic is the most convoluted section in the entire package. Explain the geometric reasoning: a voxel behind a surface point (relative to camera) with ray alignment is likely inside the object.

- [ ] **5.3 Add comment on DQ lerp sign continuity** in `lut_helper.rs` at line 57 — the dot-product sign flip before blending is a standard quaternion technique but non-obvious to readers unfamiliar with DQ interpolation.

- [ ] **5.4 Add comment on thumb opposition modes** in `lut_helper.rs` / `planner.rs` — explain that `ThumbAdd` = adduction (mode 0) and `ThumbAbd` = abduction (mode 1) correspond to two discrete thumb opposition states from the LUT.

- [ ] **5.5 Remove obvious/redundant comments** throughout — e.g., `// SAFETY:` comments on safe operations, `// Rotate offset by hand orientation (simplified quaternion rotation)` followed by 6 lines of inline rotation that could use a helper, etc.

### Phase 6: Update benchmarks

- [ ] **6.1 Update `benches/pipeline.rs`** to import all constants from `config` module, construct `Twist6`/`TwistCovariance` inline using config values, and define `demo_sphere` locally (since it was removed from the library).

### Phase 7: Update build files

- [ ] **7.1 No changes needed to `Cargo.toml`** — `config.rs` is just another module in the existing crate.
- [ ] **7.2 No changes needed to `CMakeLists.txt`** — Rust source glob already picks up new files.
- [ ] **7.3 No changes needed to `package.xml`** — dependencies unchanged.

---

## Verification Criteria

- [ ] `cargo build --release --lib` succeeds with no warnings
- [ ] `cargo test` passes with remaining tests
- [ ] `cargo bench` still compiles and runs (with LUT file present)
- [ ] `colcon build --packages-select grasp_preshaping` succeeds
- [ ] FFI struct layout is identical between `ffi_types.hpp` and `c_api.rs` (minus removed covariance)
- [ ] All runtime behavior preserved: same grasp types scored, same collision detection, same TSDF pipeline
- [ ] The config file is the single source of truth for all tunable constants

---

## Potential Risks and Mitigations

1. **FFI struct layout mismatch**
   Mitigation: The covariance removal changes `GraspTwistFFI` size. Both Rust and C++ sides must be rebuilt together. The CMake build already handles this (Rust lib built first, then C++ links against it). Verify with `std::mem::size_of` checks if desired.

2. **Removing `from_xyz_file` / `demo_sphere` breaks downstream scripts**
   Mitigation: These are only used in tests and benchmarks within this package. No external consumer was found via codebase search. If `demo_sphere` is needed later, it's trivial to re-add.

3. **Fixed covariance values may not match real system noise**
   Mitigation: The values `[0.01, 0.01, 0.01]` / `[0.005, 0.005, 0.005]` are exactly what `TwistCovariance::dummy()` used, which was already the effective default. They are now in `config.rs` and trivially adjustable.

4. **Removing tests could hide regressions**
   Mitigation: Only trivially obvious or stochastic tests are removed. All tests that verify algorithmic invariants (morton roundtrip, DQ roundtrip, TSDF distance monotonicity, etc.) are kept.

5. **Keeping `TwistWithCovarianceStamped` subscription while ignoring covariance**
   Mitigation: This avoids breaking the publisher. A brief comment in the C++ node explains the intentional ignore. If desired later, the subscription type can be changed to `Twist` in a follow-up.

---

## Alternative Approaches

1. **TOML/config file for constants instead of Rust module**: Could use a `.toml` file parsed at startup. Trade-off: adds a parser dependency and runtime file I/O for values that rarely change. A Rust module with clear constant definitions is simpler and zero-cost.

2. **Keep covariance in FFI but hardcode on C++ side**: Instead of removing from FFI, hardcode in the C++ bridge. Trade-off: keeps the 36-element array in the FFI struct for no reason, and the constant is defined in C++ rather than the centralized Rust config.

3. **Remove covariance subscription entirely (switch to `Twist`)**: Change C++ to subscribe to `geometry_msgs::msg::Twist`. Trade-off: breaks compatibility with whatever publishes `/hand_twist` as `TwistWithCovarianceStamped`. Safer to keep the subscription type and just ignore the covariance field.

---

## Self-Critique

**Factual correctness check:**
- I verified that `predict_roi()` and `predict_roi_dummy()` are NOT called from `c_api.rs` — only `predict_roi_with_samples` is used. Confirmed via `c_api.rs:354`.
- I verified that `TwistCovariance::dummy()` values `[0.01, 0.01, 0.01]` / `[0.005, 0.005, 0.005]` match the proposed fixed constants. Confirmed at `predictor.rs:67-72`.
- I verified the dead code at `c_api.rs:398-399` — `_best_loc` and `_sample_prob` are indeed unused after assignment.
- I verified `GraspWeights` factories are all identical `(1.0, 1.0, 1.0)` at `planner.rs:36-59`.
- I verified `PointCloud::from_xyz_file` and `demo_sphere` are not used in production code — only in tests and benches.

**Prompt adherence check:**
- "removing unnecessary tests" — addressed in Phase 4 with specific justification per test
- "making code more concise" — addressed in Phase 3 (dead code, duplicate constants, identical factories)
- "adding comments only where things are convoluted" — addressed in Phase 5 (4 specific locations, with rationale)
- "hardcoding a few values" — addressed in Phase 2 (fixed covariance)
- "simple file with these values" — addressed in Phase 1 (`src/config.rs`)
- "all constants relevant for performance" — all 11 tunable constants consolidated
- "switching to fixed covariance instead of topic-based" — addressed in Phase 2
- "go through all files" — all 13 source files analyzed

**One concern:** The `scripts/` directory (model.py, verify_lut.py, hand_tip_visualizer.py) was analyzed but no changes are proposed. These are development/visualization tools that don't affect the runtime package. The user's goal is runtime simplification, and these scripts are already fit for purpose.
