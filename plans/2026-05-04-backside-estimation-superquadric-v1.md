# Backside Estimation via Superquadric Shape Completion

## Objective

Integrate superquadric-based backside estimation into the existing TSDF pipeline so that:
1. **Unobserved backside surfaces** get mathematically estimated via superquadric fitting
2. **TSDF sign calculation** becomes more robust (the current ray-based approach in `pointcloud_helper.rs:416-531` is noted as potentially problematic with opposing cameras)
3. **The fusion** produces a watertight SDF where Domain A (camera), Domain B (superquadric), and Domain C (blending zone) seamlessly merge
4. **Performance** stays under the 50ms budget for the entire pipeline

---

## Current Pipeline Summary (Verified)

The pipeline flow in `c_api.rs:349-601` (`compute_from_request`):

```
Point Cloud → prune(ROI) → morton() → get_tsdf() → SMC Loop → score particles → best grasp
```

Key files and their roles:
- `src/config.rs` — All tunable constants (TSDF resolution, truncation, SMC params)
- `src/pointcloud_helper.rs` — PointCloud, Aabb, Camera, MortonPoint, Tsdf struct, `prune()`, `morton()`, `get_tsdf()`
- `src/planner.rs` — Grasp scoring (cylindrical/pinch/lateral), sweep-for-collision, binary refinement
- `src/predictor.rs` — Motion model sampling, SMC particles, resampling, twist→SE(3)
- `src/lut_helper.rs` — FingerLUT, DualQuaternion, Contact enum, finger group logic
- `src/c_api.rs` — FFI entry point, orchestrates the full pipeline, debug export
- `src/debug_export.rs` — NPZ debug dump writer

**Current TSDF construction** (`pointcloud_helper.rs:329-542`):
1. BFS flood-fill from surface voxels to compute unsigned distance (TDF)
2. Ray-based sign determination: for each voxel, check if any camera sees it as "behind" the surface
3. Uses `RAY_ALIGNMENT_THRESHOLD` (0.9) to filter out misleading occluders

**Current sign calculation weakness** (confirmed by README line 78):
> "I observed an issue where if 2 cameras are looking at the object from opposite sides, then the sign calculation can get messed up"

The current approach at `pointcloud_helper.rs:416-531` does per-voxel per-camera ray checks, but for voxels in the deep interior, opposing cameras vote against each other, leading to inconsistent signs.

---

## Implementation Plan

### Phase 0: New Module — Superquadric Module

- [ ] **0.1 Create `src/superquadric.rs`** — New module for all superquadric math. Register it in `src/lib.rs` with `pub mod superquadric;`.

- [ ] **0.2 Define `SuperquadricParams` struct** containing:
  - `epsilon1: f32, epsilon2: f32` — shape parameters (fixed per template)
  - `a: f32, b: f32, c: f32` — scale parameters (optimized)
  - `translation: Vector3<f32>` — center offset (optimized)
  - `rotation: UnitQuaternion<f32>` — locked from PCA eigenvectors

- [ ] **0.3 Define `SuperquadricTemplate` enum/const array** with the three priors:
  - Sphere: `ε₁=1.0, ε₂=1.0`
  - Box: `ε₁=0.1, ε₂=0.1`
  - Cylinder: `ε₁=0.1, ε₂=1.0`

- [ ] **0.4 Implement the implicit function `F(x,y,z)`** — The superquadric inside-outside function:
  ```
  F(x,y,z) = ((x/a)^2)^(ε₁) + ((y/b)^2)^(ε₁))^(ε₂/ε₁) + ((z/c)^2)^(ε₂) - 1
  ```
  This must operate in the **local frame** of the superquadric (translate→rotate→evaluate).

- [ ] **0.5 Implement the analytical gradient `∇F(x,y,z)`** — Hardcoded partial derivatives for the Taubin distance approximation. Each partial derivative is computed symbolically:
  - `∂F/∂x`, `∂F/∂y`, `∂F/∂z` with respect to the local-frame coordinates
  - Chain-rule through the rotation to get world-frame gradient

- [ ] **0.6 Implement `taubin_distance(point) -> f32`** — The Taubin Distance Approximation:
  ```
  D ≈ |F(x,y,z) - 1| / ||∇F(x,y,z)||
  ```
  Returns signed distance: negative inside (F<1), positive outside (F>1).

---

### Phase 1: Shape Estimation (PCA + Gauss-Newton)

- [ ] **1.1 Implement `compute_pca(points: &[Vector3<f32>]) -> (Vector3<f32>, Matrix3<f32>, Vector3<f32>)`** in `superquadric.rs`:
  - Compute centroid (mean of all points)
  - Compute 3×3 covariance matrix using `nalgebra`
  - Eigendecomposition → eigenvectors (rotation) + eigenvalues (scale hints)
  - Returns (centroid, rotation_matrix, eigenvalues)

- [ ] **1.2 Implement `compute_obb_initial_guess()`** — Project all points onto PCA axes to get min/max extents. This gives initial estimates for `a, b, c` (half-extents) and `tx, ty, tz` (center). Since we only see the front face, the center will be biased toward the camera — the solver will correct this.

- [ ] **1.3 Implement `gauss_newton_step()`** — A single Gauss-Newton iteration for 6 parameters (a, b, c, tx, ty, tz):
  - Compute residual for each point: `rᵢ = F(pᵢ) - 1`
  - Compute 6×N Jacobian matrix (analytical derivatives of F w.r.t. each parameter)
  - Solve the normal equations: `JᵀJ · δ = -Jᵀr` using `nalgebra` QR decomposition (stack-allocated for 6×6 system)
  - Apply parameter update with damping if needed
  - **Hard-cap at 3-4 iterations max**

- [ ] **1.4 Implement `fit_superquadric_template()`** — For a single template (fixed ε₁, ε₂):
  - Take PCA rotation as locked
  - Take OBB extents as initial (a,b,c) and (tx,ty,tz)
  - Run Gauss-Newton for up to 4 iterations
  - Return `(SuperquadricParams, final_error: f32)`

- [ ] **1.5 Implement `fit_best_superquadric()`** — The top-level parallel template matching:
  - Use `rayon::par_iter` over the 3 templates (Sphere, Box, Cylinder)
  - Each thread runs `fit_superquadric_template()`
  - Select the template with lowest final error
  - Return the winning `SuperquadricParams`
  - **Estimated cost: ~0.5-2ms** (3 templates × 4 iterations × N points, all parallel)

---

### Phase 2: TSDF Fusion (Modified `get_tsdf`)

- [ ] **2.1 Add superquadric parameter to `get_tsdf` signature** — Change `get_tsdf()` in `pointcloud_helper.rs:329` to accept an `Option<&SuperquadricParams>` parameter. When `None`, behavior is identical to current (backward compatible). When `Some`, activates the fusion pipeline.

- [ ] **2.2 Add blending zone constants to `config.rs`**:
  - `SQ_BLEND_DELTA_CELLS: usize = 2` — Width of the blending zone in voxel cells (transition from camera authority to superquadric authority)
  - `SQ_MIN_FIT_POINTS: usize = 20` — Minimum points required to attempt superquadric fitting
  - `SQ_FIT_ERROR_THRESHOLD: f32 = 0.1` — Maximum acceptable fitting error; if exceeded, skip backside estimation

- [ ] **2.3 Implement Domain A logic (Camera Authority)** — This is the current behavior, unchanged:
  - Voxels within the truncation band of observed surface points
  - Distance from BFS flood-fill
  - Sign from ray-based camera occlusion check
  - **No code changes needed** — existing logic at `pointcloud_helper.rs:366-413` (BFS) and `pointcloud_helper.rs:416-531` (sign)

- [ ] **2.4 Implement Domain B logic (Superquadric Authority)** — For voxels where `distance == f32::MAX` (unreachable by BFS / beyond truncation):
  - Transform voxel world coordinates to superquadric local frame
  - Compute `F(x,y,z)` to determine sign (F<1 → inside/negative, F>1 → outside/positive)
  - Compute Taubin distance approximation for the distance value
  - Clamp distance to `TRUNCATION_CELLS` to match TSDF conventions
  - This fills in the deep interior and the unobserved backside shell

- [ ] **2.5 Implement Domain C logic (Blending Zone)** — For voxels at the boundary between Domain A and Domain B:
  - Compute both `D_camera` (from BFS) and `D_sq` (from Taubin)
  - Compute blend weight `w` based on camera TDF distance to the truncation boundary:
    - If `D_camera` is small (well within camera view): `w = 1.0`
    - If `D_camera` approaches the truncation limit: `w` smoothly transitions to `0.0`
    - Use smoothstep or similar: `w = smoothstep(blend_start, blend_end, D_camera_normalized)`
  - Final: `TDF_final = w * TDF_camera + (1-w) * TDF_sq`
  - Sign follows the same blending logic

- [ ] **2.6 Modify the sign calculation for robustness** — The current sign calculation at `pointcloud_helper.rs:439-531` should be enhanced:
  - **For voxels with valid camera data (Domain A):** Keep the current ray-based sign, but add the superquadric sign as a **tiebreaker** when cameras disagree (inside_votes > 0 AND outside_votes > 0). The superquadric's F(x,y,z) < 1 test is authoritative for inside/outside determination.
  - **For voxels in Domain B:** Sign is purely from F(x,y,z).
  - **For voxels in Domain C:** Sign is blended alongside the distance.
  - This directly addresses the README concern about opposing cameras causing sign issues.

---

### Phase 3: Pipeline Integration (`c_api.rs`)

- [ ] **3.1 Add superquadric fitting step in `compute_from_request()`** — Insert between TSDF construction and SMC loop in `c_api.rs:349-601`:
  ```
  After: morton() + get_tsdf() with cameras (current sign logic)
  Add:   fit_best_superquadric(&pruned.points)
  Then:  Rebuild TSDF with get_tsdf(..., Some(&sq_params)) for fusion
  ```
  **Alternative (more efficient):** Build the TSDF once with the superquadric params passed in, so the fusion happens during construction rather than as a separate pass.

- [ ] **3.2 Choose integration strategy** — Two options:
  - **Option A (Recommended): Single-pass fusion** — Pass `Option<&SuperquadricParams>` into `get_tsdf()`. After the BFS fills Domain A and the current sign logic runs, a second parallel pass fills Domain B and blends Domain C. This avoids building the TSDF twice.
  - **Option B: Two-pass** — Build TSDF normally, then run superquadric fitting, then do a second pass to fill/blend. Simpler but ~2× TSDF construction cost.

- [ ] **3.3 Add fallback when fitting fails** — If the point cloud has too few points (`< SQ_MIN_FIT_POINTS`) or the fitting error exceeds `SQ_FIT_ERROR_THRESHOLD`, the pipeline proceeds with the current camera-only TSDF (no backside estimation). This ensures robustness.

- [ ] **3.4 Add timing instrumentation** — Add `Instant::now()` markers around the superquadric fitting step to log its contribution to `pipeline_time_ms`.

---

### Phase 4: Debug Export Updates

- [ ] **4.1 Extend `DebugDump` in `debug_export.rs`** — Add optional superquadric parameters to the debug dump:
  - `sq_params: Option<SuperquadricParams>` — the fitted parameters
  - `sq_fit_error: f32` — the residual error
  - `sq_template_name: String` — which template won (sphere/box/cylinder)

- [ ] **4.2 Extend `export_npz()`** — Add arrays:
  - `sq_params` (f32, 13): [ε₁, ε₂, a, b, c, tx, ty, tz, qw, qx, qy, qz, fit_error]
  - `sq_template` (u8, 1): 0=sphere, 1=box, 2=cylinder

- [ ] **4.3 Update the Python visualizer** (`scripts/visualize_grasp_debug.py`) — Render the superquadric wireframe alongside the TSDF cross-section for visual verification of the fit quality.

---

### Phase 5: Configuration

- [ ] **5.1 Add to `config.rs`**:
  ```rust
  // Superquadric backside estimation
  pub const SQ_MAX_GN_ITERATIONS: usize = 4;
  pub const SQ_GN_DAMPING: f32 = 0.1;
  pub const SQ_BLEND_DELTA_CELLS: usize = 2;
  pub const SQ_MIN_FIT_POINTS: usize = 20;
  pub const SQ_FIT_ERROR_THRESHOLD: f32 = 0.15;
  pub const SQ_ENABLE_BACKSIDE: bool = true;
  ```

---

### Phase 6: Testing

- [ ] **6.1 Unit tests in `superquadric.rs`**:
  - Test `F(x,y,z)` returns 0 for points on the surface of a known sphere
  - Test `F(x,y,z)` returns <1 for interior points, >1 for exterior
  - Test Taubin distance is approximately correct for known geometries
  - Test PCA on a known point distribution (e.g., points on a box face)
  - Test Gauss-Newton converges on a synthetic partial sphere

- [ ] **6.2 Integration tests in `pointcloud_helper.rs`**:
  - Test that `get_tsdf()` with `None` superquadric produces identical results to current
  - Test that a partial sphere (front half only) with superquadric produces a complete TSDF
  - Test that the blending zone has no discontinuities (gradient check)
  - Test opposing-camera sign resolution with superquadric tiebreaking

- [ ] **6.3 Benchmark in `benches/pipeline.rs`**:
  - Add `bench_superquadric_fitting` benchmark
  - Add `bench_tsdf_with_fusion` benchmark
  - Verify total pipeline stays under 50ms target

---

## Verification Criteria

- [ ] **VC1: Watertight TSDF** — The TSDF has no `f32::MAX` gaps in the object region. Every voxel within the object's bounding box has a finite distance value.
- [ ] **VC2: Correct sign in deep interior** — Voxels deep inside the object (not visible to any camera) have negative sign, confirmed by both the superquadric F(x,y,z) test and the blended sign.
- [ ] **VC3: Seamless surface seam** — The zero-crossing surface is continuous at the boundary between camera-observed and superquadric-estimated regions. No holes or double-surfaces.
- [ ] **VC4: Backward compatibility** — Passing `None` for superquadric params produces identical results to the current pipeline. All existing tests pass unchanged.
- [ ] **VC5: Performance** — Superquadric fitting + TSDF fusion adds ≤10ms to the pipeline. Total pipeline remains under 50ms.
- [ ] **VC6: Robustness** — Pipeline degrades gracefully when fitting fails (few points, complex shapes). Falls back to camera-only TSDF.

---

## Potential Risks and Mitigations

1. **Superquadric fitting divergence**
   - *Risk:* Gauss-Newton may diverge on unusual point clouds (thin structures, highly concave objects).
   - *Mitigation:* Hard-cap iterations at 4, add parameter bounds (a,b,c must be positive and within reasonable range), fall back to camera-only if error exceeds threshold.

2. **Blending zone artifacts**
   - *Risk:* The transition between camera TDF and superquadric TDF may produce visible ridges or gaps at the seam.
   - *Mitigation:* Use smoothstep blending over 2+ voxel cells. The blending zone width `SQ_BLEND_DELTA_CELLS` is tunable. Test with the debug visualizer.

3. **Performance regression**
   - *Risk:* Superquadric evaluation for every voxel in Domain B could be slow for large grids.
   - *Mitigation:* Only evaluate voxels that are `f32::MAX` after BFS (Domain B is typically smaller than Domain A). Use `rayon` for the Domain B fill pass. The Taubin distance is a closed-form expression — no iteration needed per voxel.

4. **Sign disagreement between camera and superquadric**
   - *Risk:* Near the surface, the camera ray-based sign might disagree with the superquadric F(x,y,z) sign.
   - *Mitigation:* In Domain A, camera authority wins. In Domain B, superquadric wins. In Domain C, the blend naturally resolves disagreements. The superquadric acts as a tiebreaker for the opposing-camera problem.

5. **Incorrect PCA orientation for symmetric objects**
   - *Risk:* For symmetric objects (e.g., a sphere), PCA eigenvectors are ambiguous, leading to a poorly oriented superquadric.
   - *Mitigation:* For symmetric objects, the orientation doesn't matter much (sphere fits regardless). For asymmetric objects, PCA gives a good initial frame. The Gauss-Newton translation optimization corrects the center position.

6. **TSDF grid size increase**
   - *Risk:* The superquadric may extend beyond the current grid bounds if the estimated backside is larger than expected.
   - *Mitigation:* The grid is already padded by `TRUNCATION_CELLS`. The superquadric center is pushed into the shadow, not beyond the object extents. If needed, the grid can be expanded by checking superquadric bounds during allocation.

7. **Gauss-Newton Ill-Conditioning**
   - *Risk:* The solver may fail or diverge if the initial PCA guess is slightly off or the Jacobian matrix becomes ill-conditioned.
   - *Mitigation:* Add a slight Levenberg-Marquardt identity penalty: `(JᵀJ + λI) · δ = -Jᵀr` (damping) if `nalgebra`'s QR solver struggles, to guarantee stability.

8. **Distance Metric Scaling Mismatch**
   - *Risk:* Domain A outputs real-world metric distance, but the Taubin Distance is an *approximation*. If the scales differ drastically, blending via Lerp could distort the shape space.
   - *Mitigation:* Visually verify the debug output to ensure the Taubin distance scale matches the real camera distance metric to avoid abrupt scalar jumps.

9. **Analytical Jacobian Errors**
   - *Risk:* Single sign errors or typos in the long chain-rule derivations for the 6 partial derivatives will prevent the fast 4-iteration convergence entirely.
   - *Mitigation:* Pre-compute and verify all derivatives in a symbolic math tool (like SymPy or Mathematica) to guarantee mathematical correctness before hardcoding them in Rust.

---

## Alternative Approaches

1. **Virtual point sampling** — Instead of Taubin distance, sample virtual points on the superquadric surface and add them to the point cloud before TSDF construction.
   - *Trade-off:* Simpler to implement (no Domain B/C logic needed), but adds many points to the BFS, increasing construction time. Less mathematically elegant but more robust to blending artifacts.

2. **Separate SDF for backside** — Build two separate SDFs (camera + superquadric) and combine them with a min/max operation.
   - *Trade-off:* Doubles memory usage. Simpler blending logic. But the min/max operation can create discontinuities at the boundary.

3. **Full 11-parameter superquadric** — Optimize rotation in addition to scale and translation (11 DOF instead of 6).
   - *Trade-off:* More flexible fitting, but significantly slower (larger Jacobian, more iterations). PCA-locked rotation is sufficient for most graspable objects and is much faster.

4. **Deep learning shape completion** — Use a neural network to predict the backside.
   - *Trade-off:* Much more accurate for complex shapes, but requires training data, GPU inference, and adds significant latency. Not suitable for the <50ms real-time constraint.

---

## File Change Summary

| File | Changes |
|------|---------|
| `src/lib.rs` | Add `pub mod superquadric;` |
| `src/superquadric.rs` | **NEW** — SuperquadricParams, F(), gradient(), taubin_distance(), PCA, Gauss-Newton, parallel template fitting |
| `src/pointcloud_helper.rs` | Modify `get_tsdf()` signature to accept `Option<&SuperquadricParams>`, add Domain B fill + Domain C blend after BFS + sign |
| `src/config.rs` | Add SQ_* constants |
| `src/c_api.rs` | Add superquadric fitting call between morton/TSDF and SMC loop |
| `src/debug_export.rs` | Extend DebugDump and export_npz with superquadric data |
| `benches/pipeline.rs` | Add superquadric fitting and fusion benchmarks |
| `scripts/visualize_grasp_debug.py` | Add superquadric wireframe rendering |

---

## Dependency Check

- `nalgebra` (already in `Cargo.toml`) — Provides eigendecomposition, matrix operations, QR solve. **No new dependency needed.**
- `rayon` (already in `Cargo.toml`) — Used for parallel template matching and Domain B fill. **No new dependency needed.**
- No new crate dependencies are required. The entire implementation uses existing dependencies.
