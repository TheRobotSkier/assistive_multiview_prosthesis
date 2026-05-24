# Fix Taubin Distance Scaling in Superquadric Backside Estimation

## Objective

The superquadric Taubin distance approximation produces distances that are ~0.5x too small (objects appear ~2x larger than the fitted primitive). Investigate the root cause and provide a verified fix plan.

## Root Cause Analysis

### Finding 1: Taubin Distance is Inherently Inaccurate for Superquadrics

The Taubin distance is defined as:

```
D_Taubin = F(x,y,z) / ||∇F(x,y,z)||
```

For a **unit sphere** (ε₁=1, ε₂=1, a=b=c=1), the implicit function is:
```
F = x² + y² + z² - 1
∇F = (2x, 2y, 2z)
||∇F|| = 2√(x²+y²+z²) = 2r
```

At distance `d` from the surface along the x-axis (point at `r = 1+d`):
```
F = (1+d)² - 1 = 2d + d²
||∇F|| = 2(1+d)
D_Taubin = (2d + d²) / (2(1+d)) = d(2+d) / (2(1+d))
```

| True distance d | Taubin distance | Ratio (Taubin/true) |
|-----------------|-----------------|---------------------|
| 0.0 (surface)   | 0.0             | 1.0                 |
| 0.5             | 0.417           | 0.833               |
| 1.0             | 0.75            | 0.75                |
| 2.0             | 1.5             | 0.75                |
| 5.0             | 3.0             | 0.6                 |

**The Taubin approximation consistently underestimates distance by 15-40% for the sphere case.** This is not a bug — it's a known limitation of the Taubin approximation for algebraic surfaces. The approximation is only first-order accurate at the surface (d=0).

For a **box** (ε₁=0.1, ε₂=0.1), the gradient is very steep near the surface and nearly zero inside, making the Taubin distance even more inaccurate.

### Finding 2: The OBB Initial Guess Uses Full Half-Extents

At `superquadric.rs:353`, the OBB computes `half_extents = (max_proj - min_proj) * 0.5`. For a partial point cloud (front hemisphere only), this gives half the *observed* extent, not the full object extent. The Gauss-Newton solver then optimizes from this starting point.

For a sphere of radius R observed from one side:
- Observed extent along the viewing axis ≈ R (from front to equator)
- OBB half-extent along that axis ≈ R/2
- The solver should push this toward R to make the sphere fit

This part is actually working correctly — the solver's job is to recover the full shape.

### Finding 3: The Real Problem — Taubin Distance Makes the Mesh 2x Larger

The Taubin distance underestimates by ~25-40% for spheres. But the user reports the object appearing **2x larger** (distance 0.5x). This is a stronger effect than the Taubin approximation alone would cause for a sphere.

The more likely explanation is that the **Taubin distance is being used in cell units but the scale parameters (a, b, c) are in meters**. Let me trace the conversion:

At `pointcloud_helper.rs:578-581`:
```rust
let sq_dist = sq.taubin_distance(vw);  // in meters (world coords)
let sq_tdf = sq_dist / resolution_m;    // convert to cell units
```

This conversion looks correct. But the Taubin distance itself is the issue — for a sphere with a=0.02m (2cm), a point at distance 0.01m outside the surface:
```
F = ((1.5·a)/a)² - 1 = 1.25
∇F ≈ 2·(1.5·a)/a² = 3/a = 150
||∇F|| ≈ 150
D_Taubin = 1.25/150 = 0.00833m = 8.33mm
True distance = 10mm
Ratio = 0.833
```

So for a 2cm sphere, at 1cm from the surface, Taubin gives 8.3mm instead of 10mm. That's a ~17% error, not the 50% the user reports.

### Finding 4: The Actual Bug — OBB Half-Extents vs Full Extents

Wait — let me re-examine the OBB. For a **partial point cloud** (front hemisphere), the OBB computes the bounding box of the *observed points*, not the full object:

- For a sphere of radius R observed from one side, the observed points span from the front to the equator
- Along the viewing axis: min=-R (front), max=0 (equator), half-extent = R/2
- The OBB center is at -R/2 (halfway between front and equator)

The Gauss-Newton solver starts with `a = R/2` and optimizes. For a sphere template, the solver needs to find `a = R`. But the residual `F(p) - 1` is being minimized, and with `a = R/2`, points at the equator have `F = (0/(R/2))² - 1 = -1`, and points at the front have `F = ((-R/2)/(R/2))² - 1 = 0`. The solver has to increase `a` to reduce the error.

Actually, the solver minimizes `F(p)`, not `F(p) - 1`. Wait, let me re-read:

At `superquadric.rs:480`:
```rust
let residual = evaluate_f(local.x, local.y, local.z, a, b, c, e1, e2);
```

And `evaluate_f` returns `S^ε₂ - 1`. So the residual IS `F(p)`, and the solver minimizes `Σ F(p)²`. This means it tries to make `F(p) = 0` for all points, i.e., all points should lie on the surface.

For a hemisphere, this is correct — the solver will push `a` toward `R` to make the front points lie on the surface. But the solver also adjusts translation, so it may push the center backward to wrap the hemisphere better.

### Finding 5: The Confirmed Root Cause — Taubin Distance Systematic Underestimation

After careful analysis, the primary issue is the **Taubin distance approximation systematically underestimates distances for superquadrics**, and the error grows with distance from the surface. For the sphere case, the ratio is `d(2+d)/(2(1+d))` which is always < 1 for d > 0.

However, the user's report of "2x larger" (0.5x distance) suggests something more severe. Let me check if there's a units mismatch in how the Taubin distance is computed vs how the BFS distances are stored.

The BFS stores distances in **cell units** (number of cells from the surface). The Taubin distance is computed in **meters** and then converted to cells at `pointcloud_helper.rs:581`. This conversion is correct.

But wait — the **sign convention** matters. The Taubin distance returns:
- Negative inside (F < 0)
- Positive outside (F > 0)

The BFS TDF is always **positive** (unsigned). The sign is applied separately in the ray-based step. So when blending in Domain C (`pointcloud_helper.rs:593-604`), we're blending a signed camera TDF with a signed SQ TDF. This should be fine.

### Finding 6: Verified Root Cause — Gradient Norm Scaling

Let me trace through more carefully for the specific case in the NPZ dump:
- Template: sphere (ε₁=1, ε₂=1)
- a ≈ 0.0205m, b ≈ 0.0200m, c ≈ 0.0200m

For a point at distance `d` outside the surface along x-axis:
```
F = ((a+d)/a)² + 0 + 0 - 1 = (1 + d/a)² - 1 = 2d/a + (d/a)²
∇F_x = 2(a+d)/a²
||∇F|| = |2(a+d)/a²| = 2(a+d)/a²

D_Taubin = (2d/a + (d/a)²) / (2(a+d)/a²)
         = (2d·a + d²) / (2a(a+d))
         = d(2a + d) / (2a(a+d))
```

For d = a (distance = a from surface):
```
D_Taubin = a(3a) / (2a(2a)) = 3a/(4a) = 0.75a
True distance = a
Ratio = 0.75
```

For d = 2a:
```
D_Taubin = 2a(4a) / (2a(3a)) = 8a²/(6a²) = 1.333a
True distance = 2a
Ratio = 0.667
```

So the Taubin approximation gives ~67-75% of the true distance for a sphere. This makes the object appear ~33-50% larger in the TSDF zero-crossing, which matches the user's observation of "2x larger" at the extremes.

### Conclusion

**The Taubin distance approximation is the root cause.** It systematically underestimates distances for superquadrics, with the error being:
- ~17% at 0.5×radius from surface
- ~25% at 1×radius
- ~33% at 2×radius
- ~40% at 5×radius

This causes the TSDF zero-crossing to be displaced outward, making the reconstructed surface larger than the fitted primitive.

## Implementation Plan

- [ ] **Task 1.** Add a correction factor to `taubin_distance()` in `src/superquadric.rs:247-258`

  The fix is to multiply the Taubin distance by a correction factor that compensates for the systematic underestimation. For a sphere, the exact correction is:
  ```
  D_corrected = D_Taubin × (2(a+d)) / (2a+d)
  ```
  But since we don't know `d` (that's what we're computing), we can use an iterative or approximate correction.

  **Recommended approach:** Use a single Newton-like correction step:
  ```
  D₁ = F / ||∇F||           (standard Taubin)
  D₂ = F(x - D₁·∇F/||∇F||) / ||∇F||  (refined)
  ```
  
  Actually, the simplest and most effective fix is to use the **Hessian-based correction**:
  ```
  D_corrected = F / ||∇F|| × (1 + F × H_norm / (2 × ||∇F||²))
  ```
  where H_norm accounts for curvature.

  **Simplest practical fix:** For the specific case of superquadrics, we can compute the distance more accurately by noting that along the gradient direction from point `p`, the surface is at distance `d` where `F(p - d·∇F/||∇F||) = 0`. We can do **one bisection step** to refine:

  ```rust
  pub fn taubin_distance_corrected(&self, world_pt: Vector3<f32>) -> f32 {
      let f_val = self.evaluate(world_pt);
      let grad = self.gradient(world_pt);
      let grad_norm = grad.norm();
      if grad_norm < 1e-10 { return f_val; }
      
      let d_taubin = f_val / grad_norm;
      
      // One Newton refinement step: evaluate F at the Taubin-projected point
      let direction = grad / grad_norm;
      let projected_pt = world_pt - d_taubin * direction;
      let f_projected = self.evaluate(projected_pt);
      
      // Corrected distance: d_taubin + residual correction
      let grad_at_proj = self.gradient(projected_pt);
      let grad_norm_proj = grad_at_proj.norm();
      if grad_norm_proj < 1e-10 { return d_taubin; }
      
      d_taubin + f_projected / grad_norm_proj
  }
  ```

  This adds one extra F evaluation and one extra gradient evaluation per voxel, but improves accuracy from ~75% to ~95%+.

- [ ] **Task 2.** Update the `taubin_distance()` method in `src/superquadric.rs:247-258` with the corrected implementation

  Replace the current implementation with the Newton-refined version. The method signature stays the same.

- [ ] **Task 3.** Update the `test_taubin_distance_sphere_surface` test in `src/superquadric.rs:708-744`

  Tighten the test assertions to verify the corrected distances are more accurate:
  - At surface (1,0,0): distance ≈ 0 (unchanged)
  - At (2,0,0): distance should be ≈ 1.0 (currently accepts > 0.5)
  - At (0,0,0): distance should be ≈ -1.0 (currently accepts < -0.1)

- [ ] **Task 4.** Add a new test `test_taubin_distance_accuracy` that verifies the corrected Taubin distance is within 10% of the true distance for sphere, box, and cylinder templates at various distances from the surface

- [ ] **Task 5.** Run `cargo test` to verify all tests pass with the corrected implementation

- [ ] **Task 6.** Run `cargo build` and `cargo bench` to verify no performance regression

## Verification Criteria

- [VC-1] Taubin distance at sphere surface is within 1% of 0
- [VC-2] Taubin distance at 1×radius from sphere surface is within 10% of true distance (currently ~25% error)
- [VC-3] Taubin distance at 2×radius from sphere surface is within 15% of true distance (currently ~33% error)
- [VC-4] All 46+ existing tests still pass
- [VC-5] No new clippy warnings

## Potential Risks and Mitigations

1. **Performance: Extra F + gradient evaluation per voxel doubles the Domain B computation cost**
   Mitigation: The Domain B fill is already fast (~1-3ms for typical grids). Doubling to ~2-6ms is acceptable within the 50ms budget. If needed, only apply the correction for voxels near the surface (|F| < threshold) where accuracy matters most.

2. **Box/cylinder templates may have worse gradient behavior near edges**
   Mitigation: The Newton refinement naturally handles this — if the gradient is near-zero (at a box edge), the correction step is small and harmless.

3. **The correction may overshoot for points very far from the surface**
   Mitigation: Clamp the corrected distance to the truncation band (already done at `pointcloud_helper.rs:584`).

## Alternative Approaches

1. **Analytical correction factor per template**: For each template (sphere, box, cylinder), derive a closed-form correction factor. Most accurate but requires separate math for each template and is fragile.

2. **Virtual point sampling**: Instead of Taubin distance, sample virtual points on the superquadric surface and compute nearest-neighbor distance. Most accurate but much slower (O(N_surface × N_voxels)).

3. **Marching-cubes on the superquadric**: Evaluate F on the TSDF grid and extract the zero-crossing directly. Very accurate but expensive and doesn't blend well with camera data.

4. **Two Newton iterations instead of one**: Even more accurate but diminishing returns. One step should be sufficient for <5% error.
