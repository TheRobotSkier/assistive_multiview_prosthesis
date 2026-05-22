# Taubin Distance Fix — Gradient Bug + Refinement + Guard

## Objective

Fix three verified issues in the superquadric distance computation:
1. **Gradient bug**: x/y gradient components have an extra `e2` factor for non-sphere templates
2. **Taubin underestimation**: add one Newton refinement step for ~95%+ accuracy
3. **Gradient explosion guard**: strengthen the denominator guard in `taubin_distance()`

## Verified Bugs

### Bug 1: Gradient `s_factor` includes extra `e2` factor (CRITICAL)

**Location**: `src/superquadric.rs:175`

**Current code**:
```rust
let s_factor = e2 * s.powf(e2 - 1.0);
```

**Problem**: For the x and y gradient components, the full chain rule is:
```
∂F/∂lx = ε₂ · S^(ε₂-1) · (ε₁/ε₂) · S_xy^(ε₁/ε₂-1) · (1/ε₁) · u^(1/ε₁-1) · 2·lx/a²
        = S^(ε₂-1) · S_xy^(ε₁/ε₂-1) · u^(1/ε₁-1) · 2·lx/a²
```

The `ε₂` from the outer derivative (`∂(S^ε₂)/∂S = ε₂·S^(ε₂-1)`) cancels with the `1/ε₂` from the inner derivative (`∂S/∂S_xy = (ε₁/ε₂)·S_xy^(ε₁/ε₂-1)` times `∂S_xy/∂u = (1/ε₁)·u^(1/ε₁-1)`). The correct `s_factor` for x/y is just `s.powf(e2 - 1.0)`, not `e2 * s.powf(e2 - 1.0)`.

**Impact**: For the sphere (e2=1.0), the factor is `1.0 * s^0 = 1.0` — no effect. For the box (e2=0.1), the factor is `0.1 * s^(-0.9)` instead of `s^(-0.9)` — a 10x error in the x/y gradient! This means:
- The Gauss-Newton solver's Jacobian is wrong for box and cylinder templates
- The Taubin distance's gradient norm is wrong for box and cylinder templates
- The fitting may still converge (LM damping is forgiving) but suboptimally

**Why the FD test didn't catch it**: The test at `superquadric.rs:922-965` only tests `e1=1.0, e2=1.0` (sphere), where the bug is invisible.

**Fix**: Remove the `e2` from `s_factor` for x/y, or better yet, compute x/y and z gradients consistently. The z gradient at line 198 already correctly uses `s.powf(e2 - 1.0)` without `e2`.

### Bug 2: Taubin distance systematic underestimation (MODERATE)

**Location**: `src/superquadric.rs:247-258`

**Problem**: `D = F / ||∇F||` underestimates true distance by 17-40% depending on distance from surface. This causes the TSDF zero-crossing to be displaced outward, making objects appear larger.

**Fix**: Add one Newton refinement step (see Task 2 below).

### Bug 3: Gradient explosion for box/cylinder near interior (LOW)

**Location**: `src/superquadric.rs:252`

**Problem**: The guard `grad_norm < 1e-10` only catches the exact zero-gradient case. For the box template (ε=0.1), the gradient in the flat face interior can be extremely small but non-zero (e.g., 1e-8), causing `F / 1e-8` to produce distances of thousands of cells. The clamping at `pointcloud_helper.rs:584` limits this to `truncation_cells`, but the sign may be wrong.

**Fix**: Raise the guard threshold and add a small epsilon to the denominator.

## Implementation Plan

- [ ] **Task 1.** Fix the gradient `s_factor` bug in `evaluate_gradient_f()` at `src/superquadric.rs:175`

  Change:
  ```rust
  let s_factor = e2 * s.powf(e2 - 1.0);
  ```
  To:
  ```rust
  let s_factor = s.powf(e2 - 1.0);
  ```

  This makes x/y gradients consistent with the z gradient (which already omits the `e2`).

- [ ] **Task 2.** Add Newton-refined Taubin distance in `taubin_distance()` at `src/superquadric.rs:247-258`

  Replace the current implementation with:
  ```rust
  pub fn taubin_distance(&self, world_pt: Vector3<f32>) -> f32 {
      let f_val = self.evaluate(world_pt);
      let grad = self.gradient(world_pt);
      let grad_norm = grad.norm();

      // Guard against near-zero gradient (interior of box-like shapes)
      if grad_norm < 1e-6 {
          // Fallback: use sign of F as a rough inside/outside indicator
          return if f_val < 0.0 { -0.001 } else { 0.001 };
      }

      // Standard Taubin approximation: D ≈ F / ||∇F||
      let d_taubin = f_val / grad_norm;

      // One Newton refinement step for improved accuracy.
      // Project the point along the gradient to the estimated surface,
      // then correct for the residual.
      let direction = grad / grad_norm;
      let projected_pt = world_pt - d_taubin * direction;
      let f_projected = self.evaluate(projected_pt);

      // If the projected point is very close to the surface, no correction needed
      if f_projected.abs() < 1e-6 {
          return d_taubin;
      }

      let grad_proj = self.gradient(projected_pt);
      let grad_norm_proj = grad_proj.norm();
      if grad_norm_proj < 1e-6 {
          return d_taubin;
      }

      d_taubin + f_projected / grad_norm_proj
  }
  ```

  This adds one extra F + gradient evaluation but improves accuracy from ~75% to ~95%+.

- [ ] **Task 3.** Add FD test for box template gradient in `src/superquadric.rs` (new test)

  Add a test `test_jacobian_consistency_box_template` that verifies the analytical Jacobian against finite differences for `e1=0.1, e2=0.1` (box template). This would have caught Bug 1.

- [ ] **Task 4.** Add FD test for cylinder template gradient in `src/superquadric.rs` (new test)

  Add a test `test_jacobian_consistency_cylinder_template` that verifies for `e1=0.1, e2=1.0`.

- [ ] **Task 5.** Add test for Taubin distance accuracy at various distances from surface

  Add `test_taubin_distance_accuracy` verifying that the corrected Taubin distance is within 10% of the true geometric distance for the sphere case at distances 0.5, 1.0, 2.0 radii from the surface.

- [ ] **Task 6.** Run `cargo test` to verify all tests pass

- [ ] **Task 7.** Run `cargo build` to verify clean compilation

## Verification Criteria

- [VC-1] All existing tests pass (46+)
- [VC-2] New box-template Jacobian FD test passes (verifies Bug 1 fix)
- [VC-3] New cylinder-template Jacobian FD test passes
- [VC-4] Taubin distance at 1×radius from sphere surface is within 10% of true distance (currently ~25% error)
- [VC-5] No new compiler warnings

## Potential Risks and Mitigations

1. **Gradient fix changes box/cylinder fitting behavior**
   Mitigation: The fix makes the Jacobian correct, so fitting should improve. The sphere template (which usually wins for small objects) is unaffected. Run benchmarks to verify.

2. **Newton refinement doubles Taubin computation cost**
   Mitigation: Only applies to Domain B voxels (beyond BFS reach). Typical grids have ~10-30% such voxels. The 2-6ms estimate becomes ~4-12ms, still within budget.

3. **Raised guard threshold (1e-6 instead of 1e-10) may reject valid gradients**
   Mitigation: For the box template with ε=0.1, the gradient in the face interior is genuinely near-zero. Returning a small signed constant is more correct than returning F/tiny_number = huge_value.

## Alternative Approaches

1. **Keep `e2` in `s_factor` but fix the x/y formula differently**: Could multiply the z gradient by `e2` instead. But this is inconsistent — better to remove `e2` from `s_factor` since the math says it cancels.

2. **Use `e2` in `s_factor` and divide x/y by `e2`**: Equivalent but more confusing.

3. **Skip Newton refinement, use analytical correction factor**: For sphere, `D_corrected = D × (2(a+d))/(2a+d)`. But requires knowing `d`, which is circular. Newton step is simpler and more general.
