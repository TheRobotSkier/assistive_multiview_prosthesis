# Grasp Preshaping & System Architecture — Slide Outline

## How to use this document

Each slide has:
1. **Title** — a short statement that stands on its own
2. **Content** — what to put on the slide (image, equation, code block, diagram, etc.)
3. **Notes** — context or cross-references to code

---

# PART 1: GRASP PRESHAPING

---

## Slide 1 — Grasp Preshaping Solves "How Do I Configure My Hand Before I Touch the Object?"

**Content:** A photo of a prosthetic hand (Mia Hand) reaching toward an object, with an arrow showing the hand is still approaching (not yet touching).

**Notes:** The core problem: a prosthetic hand cannot reactively grasp because mechanical latency means by the time you detect the object and decide a grasp, the hand would have already passed through it. The grasp must be *predicted* and *configured* in the ~0.5–2 seconds before contact.

---

## Slide 2 — The Hand Arrives in the Correct Configuration, Not the Default One

**Content:** Two-panel image. Left: default open hand hitting an object (bad — object pushed away). Right: hand already in POWER grasp fingers closing as it reaches (good — object captured).

**Notes:** The difference between prediction and reaction. This is the motivation for the entire preshaping pipeline.

---

## Slide 3 — The Pipeline Must Work Despite 20cm+ Tracking Drift

**Content:** A plot showing OpenVINS drift over time — two trajectories (ground truth vs OpenVINS estimate) diverging to 20+ cm in ~10 seconds. Reference: `localization-rework-critique.md`.

**Notes:** The head and wrist cameras run independent OpenVINS instances. When the wrist moves out of the head's field of view, both drift independently. This is the primary failure mode that the entire architecture must handle.

---

## Slide 4 — The Grasp Pipeline Evolved Through Three Generations

**Content:** A three-panel timeline:
- **Gen 1:** "No iterations" — simple heuristic grasp, one shot, no optimization
- **Gen 2:** "SMC" — Sequential Monte Carlo with 20k particles, broad sampling from motion model, no refinement
- **Gen 3:** "ES" — Evolution Strategy with elite preservation, geometric decay of variance, 5 iterations

**Notes:** Cross-ref: `c_api.rs:440-508` shows the evolution from iteration 0 broad sampling (`sample_initial_particles`) to iterative elite resampling (`resample_around_elites`).

---

## Slide 5 — Gen 1: No Iterations = One Shot Heuristic

**Content:** Simple pseudocode:
```
grasp = closest_predefined_pose(object_centroid)
hand.command(grasp.finger_positions)
```

**Notes:** This was the baseline. It fails because objects come in varied shapes, orientations, and positions.

---

## Slide 6 — Gen 2: SMC = Sample Many, Pick Best, But No Refinement

**Content:** Code block from `predictor.rs:197-248` showing `sample_future_poses`:
```rust
for _ in 0..config.n_samples {
    let t = sample_prediction_time(...);
    let sampled = sample_twist(twist, covariance, t, rng);
    let future_pose = current_pose.multiply(&displacement_dq);
    let grasp_type: usize = rng.random_range(0..3);
    poses.push(SampledPose { pose: future_pose, ... });
}
```

**Notes:** 20,000 particles sampled in one batch from the constant-velocity motion model. All particles are scored. The best one is selected. No iterative improvement.

---

## Slide 7 — Gen 3: ES = Iterative Refinement with Elitism

**Content:** Code block from `c_api.rs:468-507` showing the iterative loop:
```rust
for iteration in 0..n_iterations {
    scored = score_all_particles(lut, &tsdf, &mut particles, ...);
    let elite_indices = select_elite_indices(&particles, ELITE_RATIO());
    let decay = DECAY_RATE().powi(iteration as i32);
    particles = resample_around_elites(
        &elites, n_samples,
        proposal_std_v * decay, ...);
}
```

**Notes:** 5 iterations. Elite ratio = 0.05 (preserve top 5%). Decay rate = 0.7 (variance shrinks each iteration). This is a (μ,λ)-Evolution Strategy.

---

## Slide 8 — The Key Insight: Sampling Around the Hit Point Changed Everything

**Content:** Two diagrams:
- **Before:** Particles sampled uniformly across the full 5-second prediction horizon → most miss the object entirely
- **After:** Twist propagation provides a `hit_time_s` → particles concentrate around contact time → ROI shrinks drastically

**Notes:** Cross-ref: `c_api.rs:376-401` — the `hit_time_opt` parameter changes how `sample_prediction_time` works. When hit time is available, `sample_future_poses` concentrates particles around `hit_time_s ± spread` instead of uniform `[0, t_max]`.

---

## Slide 9 — Constant-Velocity Motion Model Propagates the Hand Forward

**Content:** Equation from `predictor.rs`:
```
displacement_SE3 = exp(twist · t)    // twist = [ω, v] from OpenVINS
future_pose = current_pose ⊕ displacement_SE3
```

**Notes:** The twist (angular + linear velocity) from OpenVINS is assumed constant over a short horizon. This is valid because the prediction window is small (0.5–2 seconds for hit-based, 5 seconds uniform).

---

## Slide 10 — Adding Noise Models Uncertainty

**Content:** Equation and code:
```
noise_ω ~ N(0, fixed_cov_omega · t)    // cov = [0.001, 0.001, 0.001]
noise_v ~ N(0, fixed_cov_v · t)       // cov = [0.0005, 0.0005, 0.0005]
```

From `predictor.rs:39-50`, `config/grasp_preshaping.yaml:27-28`:
```yaml
fixed_cov_omega: [0.001, 0.001, 0.001]
fixed_cov_v: [0.0005, 0.0005, 0.0005]
```

**Notes:** Covariance scales with `√t` (Wiener process). This encodes increasing uncertainty further into the future.

---

## Slide 11 — Overall Pipeline Architecture (The Big Picture)

**Content:** The architecture diagram (to be created/drawn — reference the data flow in `c_api.rs:366-631`):

```
PointCloud ──→ ROI Prediction ──→ Prune & Morton ──→ Superquadric Fit
                                                       │
                                                       ▼
GraspCommand ←── ES Optimizer ←── Score Particles ←── TSDF Construction
                                             │
                                    (Finger LUT for IK)
```

**Notes:** This is the one-image summary. Four stages: ROI, Superquadric, TSDF, ES. Each described in detail next.

---

# PART 2: ROI PREDICTION

---

## Slide 12 — ROI Prediction: "Where Will the Object Be When I Arrive?"

**Content:** Image of a point cloud with a highlighted AABB (axis-aligned bounding box) region around the predicted contact point.

**Notes:** The ROI (Region of Interest) prunes the input point cloud from potentially millions of points down to a few thousand, making the rest of the pipeline tractable.

---

## Slide 13 — Without the ROI, the Point Cloud is Too Large

**Content:** Numbers:
- Raw fused point cloud: ~500k–2M points
- After ROI prune: ~5k–20k points (100x reduction)
- TSDF voxel grid: from 200³ to 30³ (300x fewer voxels)
- Processing time: from seconds to milliseconds

**Notes:** The ROI is the critical performance enabler. Cross-ref: `pointcloud_helper.rs:238-251` (`prune` function).

---

## Slide 14 — The ROI is Computed from Predicted Hand Trajectory

**Content:** Code block from `predictor.rs:197-248`:
```rust
fn sample_future_poses(current_pose, twist, covariance, config, rng, hit_time_opt) {
    // For each sample, predict future hand pose
    let future_pose = current_pose.multiply(&displacement_dq);
    // Project fingertip positions
    let tip_positions = project_index_tips(&future_pose, ...);
    // Compute AABB encompassing all tips
}
```

**Notes:** The ROI AABB is computed as the bounding box of all predicted index fingertip positions from the Monte Carlo samples. This ensures the ROI covers where the hand will actually be.

---

## Slide 15 — Hit Time Makes the ROI Tighter and More Accurate

**Content:** Two AABBs side-by-side:
- Without hit time: large box (uniform sampling over 5s)
- With hit time: small box (sampling concentrated around contact)

**Notes:** From `predictor.rs:188-194` — when `hit_time_opt` is available, particles sample around `hit_time_s` with `spread = 0.5s` instead of uniformly across 0..5s. This is the dimensionality realization that changed everything.

---

## Slide 16 — Why ROI Prediction Might Be Less Important Now

**Content:** A plot showing TSDF construction time vs number of voxels. If the TSDF is fast enough at full resolution, the ROI becomes just an optimization, not a necessity.

**Notes:** Acknowledge that as hardware improves (laptop GPU), the ROI may become optional. But on current embedded hardware (Jetson), it's essential.

---

# PART 3: MORTON CODES

---

## Slide 17 — Morton Codes Provide Cache-Friendly Spatial Ordering

**Content:** A 2D Z-order curve diagram showing a grid with numbered cells following the Z-order pattern (bit-interleaving).

**Notes:** Morton codes (Z-order curves) map 3D coordinates to 1D values while preserving spatial locality. Points close in 3D space have close Morton codes.

---

## Slide 18 — Interleaving Bits: The Core of the Z-Order Curve

**Content:** Bit operation:
```
x = 0b0011  →  split = 0b0000_0000_0000_0011  →  interleave
y = 0b0101  →  split = 0b0000_0000_0000_0101  →  0b001011
z = 0b1001  →  split = 0b0000_0000_0000_1001
```

**Notes:** The `split_by_3` function in `pointcloud_helper.rs:253-258` expands each 16-bit coordinate into a 48-bit code by inserting two zero bits between each original bit, then ORs them together with shifts of 0, 1, and 2 positions.

---

## Slide 19 — Morton Encoding Code

**Content:** From `pointcloud_helper.rs:290-316`:
```rust
fn morton(pc: &PointCloud, resolution_m: f32) {
    let gx = ((p.x - min.x) / resolution_m).floor() as u16;
    let gy = ((p.y - min.y) / resolution_m).floor() as u16;
    let gz = ((p.z - min.z) / resolution_m).floor() as u16;
    let morton_id = split_by_3(gx) | (split_by_3(gy) << 1) | (split_by_3(gz) << 2);
}
```

**Notes:** The grid index is computed from world coordinates using `resolution_m = 0.005m` (5mm). The Morton code is the interleaved bit pattern of (gx, gy, gz).

---

## Slide 20 — Sorting by Morton + Merging Duplicates = Unique Voxel Grid

**Content:** Pipeline:
```
Points → Morton Encode → Sort by morton_id → 
    Scan for duplicates → Offsets array → Unique voxel grid
```

**Notes:** From `pointcloud_helper.rs:316-328`: After parallel sorting by `morton_id`, duplicate voxels (same `morton_id`) are detected by scanning for changes. The `offsets` array maps each unique voxel to its range in the sorted array.

---

## Slide 21 — Morton Codes Enable Efficient BFS-Based Distance Transform

**Content:** Diagram: A voxel grid with occupied cells in black. The BFS propagates distance values outward, layer by layer, using 6-connected neighbors.

**Notes:** Once we have unique voxels from Morton encoding, we can seed a BFS from the occupied cells to compute the unsigned distance field. This is the critical data structure for the TSDF.

---

# PART 4: TSDF CONSTRUCTION (BFS + Sign)

---

## Slide 22 — We First Build an Unsigned Distance Field via BFS

**Content:** Code from `pointcloud_helper.rs:367-415`:
```rust
let mut distance = vec![f32::MAX; total];  // per-voxel distance
let mut nearest = vec![0u32; total];        // index of nearest surface point

// Seed BFS from occupied voxels
for group in 0..offsets.len() - 1 {
    let idx = px + py * stride_y + pz * stride_z;
    distance[idx] = 0.0;
    nearest[idx] = start as u32;
    queue.push_back((px, py, pz));
}

// BFS propagation
while let Some((cx, cy, cz)) = queue.pop_front() {
    // 6-connected neighbor expansion
    for &(dx, dy, dz) in &neighbor_offsets {
        let new_dist = cdist + 1.0;
        if new_dist < distance[nidx] {
            distance[nidx] = new_dist;
            nearest[nidx] = nearest[cidx];
            queue.push_back((nx, ny, nz));
        }
    }
}
```

**Notes:** The BFS starts from occupied voxels (distance=0) and propagates outward in 6-connected directions. Each voxel stores: (1) its distance to the nearest surface, (2) which surface point it is closest to. Truncation at `truncation_cells = 4`.

---

## Slide 23 — The BFS Produces a Truncated Unsigned Distance Field

**Content:** 1D visualization:
```
Surface:      |   X   |   (occupied voxels)
Distance:     0 0 0 0 0 1 2 3 4 MAX MAX MAX ...
Truncated:    0 0 0 0 0 1 2 3 4 ∞ ∞ ∞ ...
```

**Notes:** Distance values beyond `truncation_cells * resolution_m = 4 * 0.005m = 2cm` are set to `f32::MAX` (unknown). This truncation keeps the TSDF compact.

---

## Slide 24 — The Sign is Determined by Camera Ray Voting

**Content:** Diagram: A camera looking at a surface. Voxels in front of the surface get positive sign (outside), voxels behind get negative sign (inside). The ray projects from camera through voxel to the nearest surface point.

**Notes:** From `pointcloud_helper.rs:418-518`: For each voxel, each camera projects a ray. If the voxel is closer than the surface point along that ray → outside vote. If farther → inside vote. Final sign: inside if any camera votes inside AND no camera votes outside.

---

## Slide 25 — The Sign Voting Algorithm

**Content:** Pseudocode:
```
for each voxel v:
    for each camera c:
        ray = camera → v → nearest_surface(v)
        if v is behind nearest_surface on this ray → inside_vote++
        else → outside_vote++
    
    if inside_votes > 0 && outside_votes == 0:
        dist = -|dist|   # inside the object
    elif inside_votes > 0 && outside_votes > 0:
        use superquadric tiebreaker
    else:
        keep positive (outside)
```

**Notes:** From `pointcloud_helper.rs:502-517`. The "disagree" case (both inside and outside votes from different cameras) is resolved by the superquadric — if the SQ says the voxel is inside, use negative sign.

---

## Slide 26 — The Ray Alignment Threshold Prevents False Occlusions

**Content:** Diagram: A voxel near the edge of an object. The nearest surface point is on the object's front face, but the voxel is behind it from the camera's perspective. However, the surface normal points away from the camera direction → the ray is misaligned → the vote is discarded.

**Notes:** From `pointcloud_helper.rs:475-486`: If the direction from surface point to voxel is not aligned with the camera ray (dot product < `ray_alignment_threshold = 0.9`), the camera vote is discarded. This prevents "wrapping around" artifacts at object edges.

---

# PART 5: BACKSIDE ESTIMATION (SUPERQUADRIC)

---

## Slide 27 — The Problem: The TSDF Only Knows the Visible Side

**Content:** Image: A point cloud of a mug viewed from one side. Only the front half has points. The back is completely empty. If we score grasps using this TSDF, the planner will think it can reach through the object.

**Notes:** This is the "backside" problem. The TSDF from camera views only captures the surfaces that are visible. The back of the object is marked as "outside" (positive distance), which is wrong.

---

## Slide 28 — Superquadrics Estimate the Occluded Geometry

**Content:** Image: A superquadric surface (ellipsoid-like shape) fitted to the visible points of a mug. The SQ surface extends around the back, filling in the missing geometry.

**Notes:** A superquadric is a parametric surface that generalizes ellipsoids. By fitting it to the visible points, we can estimate the shape of the invisible back side.

---

## Slide 29 — The Superquadric Implicit Function

**Content:** The equation from `superquadric.rs:71-72`:
```
F(x, y, z) = ( ( |x/a|^(2/ε₂) + |y/b|^(2/ε₂) )^(ε₂/ε₁) + |z/c|^(2/ε₁) )^ε₁ - 1
```

Where:
- `(a, b, c)` = half-extents along local axes
- `(ε₁, ε₂)` = shape exponents ("squareness")
- F < 0 = inside, F = 0 = surface, F > 0 = outside

**Notes:** This is from `superquadric.rs:71-72` in the `evaluate_f` function docstring.

---

## Slide 30 — Shape Parameters Control the Geometry

**Content:** A 2×2 grid of superquadric shapes:
| ε₁=1, ε₂=1 | ε₁=0.1, ε₂=0.1 |
| Sphere | Box (rounded) |
| ε₁=1, ε₂=0.1 | ε₁=0.1, ε₂=1 |
| Cylinder-ish | Disk-ish |

**Notes:** The three templates from `superquadric.rs:47-63`: Sphere (ε₁=1, ε₂=1), Box (ε₁=0.1, ε₂=0.1), Cylinder (ε₁=0.1, ε₂=1). These are the priors we test in parallel.

---

## Slide 31 — We Fit Three Templates in Parallel

**Content:** Code from `superquadric.rs:628-642`:
```rust
let results: Vec<(SuperquadricParams, f32)> = TEMPLATES
    .par_iter()
    .map(|template| {
        fit_superquadric_template(
            points, *template,
            initial_a, initial_b, initial_c,
            center, &pca.axes,
        )
    })
    .collect();

let (best_params, best_error) = results
    .into_iter()
    .min_by(|a, b| a.1.partial_cmp(&b.1))?;
```

**Notes:** All three templates (sphere, box, cylinder) are fitted in parallel using `rayon`. The one with the lowest fitting error wins.

---

## Slide 32 — The Initial Guess Comes from PCA + OBB

**Content:** From `superquadric.rs:302-380`:
1. PCA: compute centroid, eigenvectors (axes), eigenvalues (variances)
2. OBB: project points onto PCA axes, find extents → gives (a, b, c) guess

```
PcaResult { centroid, axes: Rotation3, eigenvalues }
half_extents = (max_proj - min_proj) * 0.5  along each PCA axis
```

**Notes:** This is a standard initialization. PCA gives the orientation, and the Oriented Bounding Box (OBB) gives the scale. This initial guess is critical for Gauss-Newton convergence.

---

## Slide 33 — Gauss-Newton Optimizes the 6 Parameters

**Content:** From `superquadric.rs:386-465`: The 6 parameters are `[a, b, c, tx, ty, tz]`. Rotation is locked from PCA.

```
Jacobian:  J_ij = ∂F_i / ∂params_j
Hessian:   J^T J
RHS:       J^T r    (r = F_i - 0, we want F=0 on surface)
Δp = -(J^T J + λI)^{-1} J^T r     (Levenberg-Marquardt damping)
params += Δp
```

**Notes:** The Jacobian is computed analytically in `compute_jacobian_row`. Verified against finite differences in tests (`superquadric.rs:944-987`). Damping λ = 0.1 (`SQ_GN_DAMPING`). Max 4 iterations (`SQ_MAX_GN_ITERATIONS`).

---

## Slide 34 — The Cost Function: Minimize F(p_i)² Over All Points

**Content:** Equation:
```
min_{a,b,c,t} Σ_i F(p_i)²
```

Where `F(p_i)` is the superquadric implicit function evaluated at point `p_i`. We want all points as close to the surface (F=0) as possible.

**Notes:** This is an algebraic distance minimization. The residual for each point is `r_i = F(p_i) - 0 = F(p_i)`. The cost is `Σ r_i²`.

---

## Slide 35 — Thresholding Removes Outliers Corrupted by TSDF Noise

**Content:** From `superquadric.rs:648-654`:
```rust
// If the fitting error is too high, skip backside estimation
if best_error > config::SQ_FIT_ERROR_THRESHOLD() {
    return None;
}
```

Where `SQ_FIT_ERROR_THRESHOLD = 0.15`. This prevents fitting a superquadric to a noisy/incomplete point cloud, which would produce a worse TSDF than leaving the backside unknown.

**Notes:** This is a safety gate. If the point cloud is too sparse or noisy, skip the superquadric entirely and use only camera-based sign determination.

---

## Slide 36 — Taubin Distance: A Better Signed Distance from the Superquadric

**Content:** From `superquadric.rs:247-280`:
```
Taubin:  d ≈ F(x,y,z) / ||∇F(x,y,z)||
```

Then one Newton-Raphson refinement:
```
direction = ∇F / ||∇F||
projected = x - d · direction
correction = F(projected) / ||∇F(projected)||
d_refined = d + correction
```

**Notes:** The simple F value is not a Euclidean distance. Taubin distance divides by the gradient magnitude to approximate Euclidean distance. The Newton-Raphson step projects to the estimated surface and corrects for the residual, giving ~10-15% accuracy (tests in `superquadric.rs:1083-1123`).

---

## Slide 37 — Why a Newton-Raphson Step Improves Accuracy

**Content:** Diagram: A point P near the superquadric surface. First-order Taubin: project along gradient to estimated surface point S₁. Newton-Raphson: project to S₁, evaluate F at S₁, if S₁ is not on surface (F≠0), project again along ∇F(S₁) to get S₂. S₂ is much closer to the true surface.

**Notes:** The Taubin distance is a first-order approximation (linearization). The Newton step in `superquadric.rs:264-279` adds a second-order correction by re-evaluating at the projected point. This costs ~2x the computation but significantly improves accuracy for the TSDF blend zones.

---

## Slide 38 — The Superquadric Blends into the TSDF via "Domains of Authority"

**Content:** Diagram of the TSDF volume cross-section showing three regions:
- **Domain A (Camera Authority):** Near the visible surface, the camera ray-based sign and distance are trusted.
- **Domain B (Transition Zone):** A blend region where camera and superquadric distances are smoothly interpolated.
- **Domain C (SQ Authority):** Deep inside the object or far from visible surfaces, the superquadric fully determines sign and distance.

**Notes:** This is the critical insight from `pointcloud_helper.rs:521-665`. The two sources of information (camera rays and superquadric) have different strengths in different regions.

---

## Slide 39 — Three Zones: Agree, Disagree, and Transition

**Content:** From `pointcloud_helper.rs:634-664`:

**If signs agree (both say inside or both say outside):**
- Blend distances only in the outer band (`blend_start_agree` to `blend_end_agree` cells from surface)
- Inner band: keep camera distance unchanged (more accurate near surface)

**If signs disagree (camera says outside, SQ says inside):**
- Camera is wrong (backside voxel marked as outside)
- Very close to surface (`abs_cam <= blend_start_disagree=2`): flip sign, keep camera distance magnitude
- Far from surface (`abs_cam >= blend_end_disagree`): use full SQ distance
- Transition zone: smoothstep blend of absolute distances, SQ determines sign

**Notes:** Parameters: `SQ_BLEND_DELTA_CELLS = 3`, `SQ_MIN_SIGN_OVERRIDE_CELLS = 2`. From `config/grasp_preshaping.yaml:57,60`.

---

## Slide 40 — Smoothstep Blending Prevents Discontinuities

**Content:** The smoothstep function from `pointcloud_helper.rs:651`:
```rust
// Smoothstep: w goes from 1 (camera) to 0 (SQ)
let t = (abs_cam - blend_start_disagree) / (blend_end_disagree - blend_start_disagree);
let w = 1.0 - t * t * (3.0 - 2.0 * t);
let blended_abs = w * abs_cam + (1.0 - w) * sq_tdf_clamped.abs();
*dist = if sq_inside { -blended_abs } else { blended_abs };
```

**Notes:** The smoothstep (Hermite interpolation) ensures C¹ continuity — no sudden jumps in the TSDF values, which would cause false surfaces in marching cubes.

---

## Slide 41 — Sparse Masks Reduce Superquadric Evaluation Cost

**Content:** From `pointcloud_helper.rs:549-581`: Only voxels that *might* need SQ correction are evaluated:
1. Mark all negative and unobserved voxels
2. Dilate the mask by `truncation_cells` in all 6 directions
3. Only evaluate SQ for voxels in the dilated mask

**Notes:** Without this optimization, every voxel would require the expensive `taubin_distance` evaluation (which includes gradient computation and Newton refinement). The mask reduces evaluations by ~5-10x for typical scenes.

---

# PART 6: LUT (LOOKUP TABLE)

---

## Slide 42 — The LUT Replaces Online Inverse Kinematics

**Content:** Diagram: LUT as a "black box": input (joint angles) → forward kinematics → output (fingertip positions). Precomputed offline, stored as a table. At runtime: query desired fingertip position, get nearest joint angles.

**Notes:** Computing inverse kinematics online for a 5-finger hand with coupled joints is expensive and can have multiple solutions. The LUT precomputes the forward kinematics for a dense sampling of joint configurations.

---

## Slide 43 — Pinocchio Makes URDF Kinematics Easy

**Content:** From `scripts/model.py:125-131`:
```python
import pinocchio as pin
model = pin.buildModelFromUrdf(urdf_path)
data = model.createData()
geom_model = pin.buildGeomFromUrdf(model, urdf_path, pin.GeometryType.COLLISION)
```

**Notes:** Pinocchio is a C++ library with Python bindings for rigid body dynamics. It parses the URDF and provides forward kinematics, Jacobians, and collision geometry queries.

---

## Slide 44 — The URDF is Flattened to Extract Collision Primitives

**Content:** From `scripts/model.py:149-182`: The URDF contains collision geometry objects (spheres, cylinders, boxes) attached to each link. The script extracts:
- Geometry type and parameters (radius, half-length, half-extents)
- Parent joint ID
- Placement (transform) relative to the joint

```python
for geom_name in group_names:
    geom_obj = g_model.geometryObjects[geom_id]
    geom = geom_obj.geometry
    # Detect type: sphere, cylinder, or box
    if hasattr(geom, "radius"):
        geom_type = "sphere"
        params = {"radius": float(geom.radius)}
    ...
```

**Notes:** The Mia hand URDF at `urdf/mia_hand_flat.urdf` defines collision geometry for each finger segment. These are the primitives used for contact computation.

---

## Slide 45 — Contacts are Defined on Collision Primitives

**Content:** From `scripts/model.py:182-122`: Contact definitions map surface descriptors to collision geometry. Example:
```python
CONTACT_DEFINITIONS = [
    {"name": "IndexTip", "group": "index", 
     "geom": "mia_index_sensor_2", "surface": "palmar"},
    {"name": "IndexTipSide", "group": "index",
     "geom": "mia_index_sensor_2", "surface": "lateral_pos"},
    ...
]
```

25 contacts total across 5 fingers + palm. Each contact has a `surface` descriptor that determines which side of the primitive to use for contact force calculation (palmar, lateral, distal).

**Notes:** The `_resolve_surface_descriptor` function in `model.py:198` computes a local offset from the geometry surface (e.g., for a cylinder, "palmar" = dorsal side).

---

## Slide 46 — The LUT is Sampled by Sweeping Joint Angles

**Content:** From `scripts/model.py`: The script samples joint angles in a grid:
- 3 active motors: `[Thumb_Flex, TISIT_Motor, MRL_Flex]` in [0,1]
- Thumb opposition: 2 discrete states (open/opposed)
- TISIT coupling: index and thumb opposition share one motor

For each combination:
1. Compute full joint vector via `get_q_full(q_active)` (handles coupling logic)
2. Forward kinematics: `pin.forwardKinematics(model, data, q_full)`
3. `pin.updateFramePlacements(model, data)`
4. Extract fingertip positions in palm frame

---

## Slide 47 — Forces are Computed from Contact Primitives, Not Just Points

**Content:** From `lut_helper.rs` and `planner.rs`: The LUT stores for each entry:
- Joint angles (5 values per finger: the underactuated joint positions)
- Fingertip position (3D)
- Surface normal at contact point

The `ActiveContact` struct in `planner.rs:58-61`:
```rust
struct ActiveContact {
    surface_normal: Vector3<f32>,
    force_direction: Vector3<f64>,
}
```

**Notes:** Contact forces are computed from the surface normal of the collision primitive at the fingertip position. The `force_direction` is used for the force closure score — computing whether the grasp can resist external wrenches.

---

## Slide 48 — The LUT is Saved as an NPZ and Loaded in Rust

**Content:** The LUT is saved as `data/finger_contact_lut.npz` using NumPy's compressed format:
```python
np.savez_compressed(lut_path, index_tips=..., mrl_tips=..., 
                    thumb_tips=..., contact_points=..., ...)
```

Loaded in Rust via `lut_helper.rs:1-3`:
```rust
use npyz::npz::NpzArchive;
// NpzArchive reads the .npz file and deserializes arrays
```

**Notes:** The `.npz` format is cross-platform (Python → Rust). Loading happens once at startup via `FingerLUT::load(path)` in `lut_helper.rs`, then cached as a static reference. The LUT is ~50-100 MB depending on sampling density.

---

# PART 7: EVOLUTION STRATEGY OPTIMIZATION

---

## Slide 49 — From SMC to ES: The Dimensionality Realization

**Content:** Timeline:
1. **SMC (Sequential Monte Carlo):** 20k particles, sampled once from motion model, best selected. Used when we searched over position + orientation + grasp type + wrist rotation.
2. **ES (Evolution Strategy):** Same 20k particles, but refined over 5 iterations with elite preservation and variance decay. Became viable when twist propagation provided the hit time, collapsing the position search to a small ROI.

**Notes:** The "dimensionality realization": without the hit time, position uncertainty is huge (5s × hand speed = 2.5m range). With hit time, position uncertainty shrinks to ~10cm. This freed up the optimizer to focus on wrist rotation and grasp type.

---

## Slide 50 — ES Handles High Dimensionality with Fewer Samples

**Content:** Plot: Convergence rate comparison. SMC flat at 1 iteration (no refinement). ES converges in 3-5 iterations.

| Method | Samples | Dimensions | Converges? |
|--------|---------|------------|------------|
| Random sampling | 20,000 | 3 pos + 3 rot + 1 grasp + 1 wrist | No |
| SMC (1 iter) | 20,000 | same | No (one shot) |
| ES (5 iters) | 20,000 × 5 | same | Yes (with hit time) |

**Notes:** From `c_api.rs:440-508`: 5 iterations each with 20,000 particles = 100,000 evaluations. But with elite preservation, only the top 5% survive. The key was reducing the position search space via hit time.

---

## Slide 51 — The ES Loop: Score, Select, Resample, Decay

**Content:** From `c_api.rs:468-507`:
```rust
for iteration in 0..n_iterations {
    // 1. SCORE all particles
    scored = score_all_particles(lut, &tsdf, &mut particles, ...);
    
    // 2. SELECT elite indices (top 5%)
    let elite_indices = select_elite_indices(&particles, ELITE_RATIO());
    
    // 3. RESAMPLE around elites with variance decay
    let decay = DECAY_RATE().powi(iteration as i32);
    particles = resample_around_elites(&elites, n_samples, 
        proposal_std_v * decay, ...);
}
```

**Notes:** Standard (μ,λ)-ES with:
- Elite ratio μ/λ = 0.05 (1000 elites from 20k population)
- Decay rate = 0.7 per iteration (variance: 1.0 → 0.7 → 0.49 → 0.34 → 0.24)
- Elite preservation (`elite_preserve_ratio = 0.01`): top 1% kept unchanged

---

## Slide 52 — Why ES is Suitable (and Why it Might be Overkill

**Content:** Table:
| Factor | Suitable | Overkill |
|--------|----------|----------|
| Wrist rotation search | Yes (range ±90°) | — |
| Grasp type selection | Yes (3 types) | Could brute-force |
| Hand position search | Not needed (hit time) | ROI handles this |
| Force controller integration | — | Force controller can't switch types, so ES mostly optimizes wrist rotation |

**Notes:** In practice, the force controller constraint (can't dynamically switch grasp types mid-grasp) means the ES primarily optimizes wrist rotation. A simpler 1D optimizer could suffice for the wrist alone, but the ES architecture is ready for when multi-type grasping is enabled.

---

## Slide 53 — Scoring a Particle: Contact, Alignment, Force Closure

**Content:** From `planner.rs:23-36`:
```rust
pub fn combined_score(&self, weights: &GraspWeights, sample_probability: f64) -> f64 {
    (weights.w_probability * sample_probability
     + weights.w_alignment * self.alignment_score
     + weights.w_force_closure * self.force_closure_score
     + weights.w_contact_score * self.contact_score) / denom
}
```

Weights (from config):
- `weight_contact_score = 3.0` (most important)
- `weight_alignment = 1.0`
- `weight_force_closure = 1.0`
- `weight_probability = 1.0`

**Notes:** The contact score is weighted 3x because it's the primary physical constraint — a grasp with bad contact physics will fail regardless of alignment or force closure.

---

## Slide 54 — Contact Score Uses a Tiered Metric

**Content:** From `planner.rs:13-16`:
```rust
/// Tier 4 (no collision) = 0.0
/// Tier 3 (start collision) = 0.1
/// Tier 2 (soft rejection) = fraction * 0.5
/// Tier 1 (valid contact) = 0.8 + 0.2 * fraction
pub contact_score: f64,
```

**Notes:** This tiered system prevents the optimizer from exploiting geometric errors. If a finger is inside the object (TSDF negative), it's Tier 3 — near-zero score. If the finger is far from the surface, Tier 4 — also zero. Valid contacts (near surface, outside object) are Tier 1: score 0.8–1.0.

---

## Slide 55 — The SMC and ES Are Fundamentally Similar

**Content:** Side-by-side comparison:
```
SMC (Gen 2):                 ES (Gen 3):
sample N particles           sample N particles
score all                    score all
pick best                    preserve elites
stop                         resample with mutation
                             repeat M times
```

**Notes:** The key difference is iteration. SMC = 1 iteration of ES with no resampling. The scoring function is identical. This made the transition from Gen 2 to Gen 3 straightforward — just add the loop.

---

# PART 8: EVALUATION

---

## Slide 56 — Timing Benchmark: Per-Stage Latency

**Content:** Bar chart or table from `benches/pipeline_stages.rs`:

| Stage | Time | % of Total |
|-------|------|------------|
| ROI + Prune + Morton | ~2 ms | 3% |
| Superquadric Fitting | ~1 ms | 1.5% |
| TSDF Construction | ~35 ms | 54% |
| ES (5 iters × 20k) | ~20 ms | 31% |
| Total Pipeline | ~65 ms | 100% |

**Notes:** Synthetic benchmark using a 10k-point cylinder cloud. TSDF construction is the bottleneck (ray casting over voxel grid). ES scoring uses `rayon` parallel iterator over 20k particles.

---

## Slide 57 — Latency and Variability vs Number of Samples

**Content:** X-Y plot: x = number of particles (5k, 10k, 15k, 20k), y = latency (ms). Two lines: mean latency and 95th percentile. Second subplot: score variance (how much the best grasp changes between runs) vs samples.

**Notes:** From benchmark data. Shows the trade-off: more samples = better convergence but higher latency. 20k was chosen as the sweet spot where additional samples give diminishing returns.

---

## Slide 58 — Test 1: Software Verification on Known Objects

**Content:** Image from `tests/test1_software_verification/`: A robotic hand grasping a known object (cylinder, sphere, box). A table showing predicted grasp type vs expected grasp type for each object.

| Object | Expected | Predicted | Score |
|--------|----------|-----------|-------|
| Cylinder (3cm) | Power | Power | 0.92 |
| Sphere (3cm) | Power | Pinch | 0.71 |
| Box (5cm) | Power | Power | 0.88 |

**Notes:** The test replays golden rosbag data through the pipeline and compares outputs against committed baselines. Tier A = latency + occlusion + score sweep. Tier B = full ROS pipeline.

---

## Slide 59 — Failure Modes in Test 3: What Goes Wrong

**Content:** A set of small plots showing failure cases:
1. **No points in ROI:** The hand pose is incorrect → ROI misses the object entirely
2. **Contact override expires:** The segmentation takes > 2 seconds → planner uses live hand pose instead of contact click → ROI mismatch
3. **Stale plan keeps publishing:** Pipeline exits APPROACHING but proximity controller doesn't clear the plan → hand keeps closing on nothing

**Notes:** Cross-ref: `plan_fix_v36.md` documents these failures. Each was fixed in subsequent iterations.

---

## Slide 60 — Fix 1: State-Owned Proximity Execution

**Content:** From `plan_fix_v36.md`:
```
Root cause: proximity controller publishes commands 
in ALL states except GRASPING/HOLDING/VOLITIONAL.
Fix: only publish when pipeline_state == APPROACHING.
```

**Content:** Clear the plan when leaving APPROACHING or entering IDLE.

**Notes:** This was a critical safety fix. The proximity controller was silently running even when the pipeline had already released the grasp.

---

## Slide 61 — Fix 3: Contact Override Age Extended to 6 Seconds

**Content:** From `plan_fix_v36.md:90-98`:
```
Root cause: Hard-coded 2-second contact override freshness 
expired before segmentation completed.
Fix: Parameterize as contact_override_max_age_s, set to 6.0s.
```

**Notes:** The segmentation model (InterObject3D on MinkowskiEngine) takes 2-4 seconds on the laptop GPU. The old 2-second timeout meant the planner always fell back to the less accurate live hand pose.

---

# PART 9: SYSTEM DEPLOYMENT

---

## Slide 62 — Running the System is Simple: `make run`

**Content:** A terminal window showing:
```bash
# Terminal 1: On Jetson (robotlab)
$ make jetson-cameras       # starts RealSense + OpenVINS
# Terminal 2: On Host (laptop)
$ make network-tune-all     # one-time UDP buffer tuning
$ make up                   # starts Docker containers
$ make dev-shell            # enter prosthesis container
$ make run                  # start full pipeline
```

**Notes:** The `Makefile` at the project root orchestrates all deployment. It auto-detects Docker vs Podman, manages WSL2 GPU passthrough, and handles Jetson→host communication.

---

## Slide 63 — Two Containers: Prothesis + Segmentation

**Content:** Architecture diagram:
```
┌─────────────────────────────────────────────┐
│               HOST LAPTOP                   │
│  ┌─────────────────┐  ┌──────────────────┐  │
│  │ prothesis       │  │ segmentation-cuda│  │
│  │ ROS2 Jazzy      │  │ Python 3.8       │  │
│  │ C++ / Python    │  │ MinkowskiEngine  │  │
│  │ Rust staticlib  │  │ Flask HTTP :5678 │  │
│  └──────┬──────────┘  └────────┬─────────┘  │
│         │ DDS localhost        │ HTTP       │
│         └──────────────────────┘             │
│  ┌─────────────────────────────────────────┐ │
│  │ RViz (optional, separate container)     │ │
│  └─────────────────────────────────────────┘ │
└─────────────────────────────────────────────┘
         │ DDS Ethernet (10.42.0.x)
┌─────────────────────────────────────────────┐
│               JETSON NANO                   │
│  RealSense D435i x2 + OpenVINS              │
│  Streams: /head/points, /arm/points, /tf    │
└─────────────────────────────────────────────┘
```

**Notes:** The Jetson handles sensor streaming and VIO. The host laptop runs the heavy pipeline (TSDF, ES, segmentation). Communication over direct Ethernet link via CycloneDDS.

---

## Slide 64 — Debugging Uses sysmon, Rosbags, and Analysis Scripts

**Content:** Command pipeline:
```bash
# Capture system telemetry + rosbag
$ make record-debug   # starts sysmon.py + ros2 bag record

# Analyze
$ python3 scripts/analyze_bag.py data/bags/v36/bag.mcap
$ python3 scripts/analyze_log.py logs/host-log-v36.txt --plot
```

**Notes:** `sysmon.py` captures host + Jetson CPU/GPU/memory/network metrics in a single JSONL stream. `analyze_bag.py` computes per-topic health (effective Hz vs nominal, dropouts, jitter). `analyze_log.py` counts known failure events.

---

## Slide 65 — Jetson Time Sync: Chrony for Sub-ms Alignment

**Content:** From `Makefile` targets: `make timesync`
```bash
$ make timesync
# Configures chrony on host + Jetson
# Host is NTP server, Jetson is client
# Drift typically < 0.5ms over direct Ethernet
```

**Notes:** Timestamps from Jetson cameras must align with host clocks for point cloud fusion to work. Chrony NTP over the direct Ethernet link keeps the two clocks synchronized.

---

## Slide 66 — The Network Problem: UDP Fragmentation Kills Point Cloud Reception

**Content:** From `scripts/tune_network_host.sh:3-6`:
```
Problem: Two RealSense D435i PointCloud2 streams at 15-30 Hz,
each 5-8 MB. CycloneDDS fragments each cloud into ~120 RTPS
messages of 64 KB each. Default Linux kernel buffers are too
small → fragments dropped → point clouds lost.
```

Default buffers: `rmem_max = 208 KB`, `ipfrag_high_thresh = 4 MB`
After tuning: `rmem_max = 2 GB`, `ipfrag_high_thresh = 128 MB`

**Notes:** A single lost fragment invalidates the entire point cloud. This was a critical bug fix — without it, the host could not reliably receive point clouds from the Jetson.

---

## Slide 67 — One Invalid UDP Fragment Corrupts the Whole Cloud

**Content:** Diagram: A point cloud is split into 120 fragments. If fragment #73 is lost (shown in red), the entire packet is unrecoverable. CycloneDDS silently drops the incomplete packet.

**Notes:** This means a 0.8% fragment loss rate causes 100% point cloud loss for that packet. This is why the buffer tuning is essential — it prevents fragment drops under load.

---

## Slide 68 — Network Tuning is a One-Time `make network-tune`

**Content:** From `scripts/tune_network_host.sh:60-73`:
```bash
cat > /etc/sysctl.d/99-prosthesis-udp.conf <<EOF
net.core.rmem_max = 2147483647
net.core.wmem_max = 2147483647
net.ipv4.ipfrag_high_thresh = 134217728
net.ipv4.ipfrag_time = 3
EOF
```

**Notes:** Written to `/etc/sysctl.d/` so it persists across reboots. The Jetson has a mirror script. Can be applied via `make network-tune-all`.

---

## Slide 69 — System Monitoring Shows the Difference Before/After Tuning

**Content:** Two plots side-by-side:
- **Before tuning:** `/head/points` topic rate drops to 0-1 Hz every few seconds, with bursts of lost packets
- **After tuning:** Stable 15 Hz for both cameras, no drops

**Notes:** From the network buffer tuning plan (`plans/2026-06-16-network-buffer-tuning-host-v2.md`). The sysmon JSONL data confirms the improvement.

---

## Slide 70 — DDS Settings: Best-Effort Reliability for Point Clouds

**Content:** From `cyclonedds_peer.xml` and ROS config:
```xml
<!-- Best-effort reliability for high-bandwidth point cloud topics -->
<!-- Reliability: RELIABLE would cause backpressure and delays -->
<!-- History: KEEP_LAST 1 to always get the latest cloud -->
```

**Notes:** Point clouds use BEST_EFFORT reliability because they arrive at 15 Hz and we always want the latest. RELIABLE would cause the DDS stack to buffer old clouds. For control messages (grasp commands), RELIABLE is used.

---

## Slide 71 — Debug Plots Show the Full Picture

**Content:** An example debug plot (produced by `analyze_bag.py --plot`):
- Top: Topic rates over time (lines for each ROS topic, colored by health)
- Middle: Pose trajectory (X/Y/Z of hand pose over time, with drift highlighted)
- Bottom: System metrics (CPU, GPU, network bandwidth from sysmon)

**Notes:** These plots are generated automatically from rosbag + sysmon data. They provide a single-screen overview of system health for any test run.

---

# PART 10: FUTURE WORK (NOT YET VALIDATED)

---

## Slide 72 — MobileSAM + TSDF Fusion Could Solve the Segmentation Problem

**Content:** Diagram from `localization-rework-critique2.md:60-67`:
```
[3D Hit Point] ──► Project via Camera Intrinsics (K) ──► [2D Pixel]
                                                          │
                                                          ▼
[Segmented Object Cloud] ◄── Filter Cloud with Mask ◄── [MobileSAM 2D Mask]
```

**Notes:** Replace InterObject3D (MinkowskiEngine, 3D sparse CNN, 4GB VRAM, Python 3.8 lock-in) with:
1. TSDF fusion of multi-view keyframes → clean dense cloud
2. MobileSAM on 2D wrist image → 2D mask
3. Project mask onto TSDF cloud → segmented 3D object

---

## Slide 73 — TSDF Resolves the "Segment Before or After Fusion" Question

**Content:** From `localization-rework-critique2.md:138-148`:
- **Segment before fusion:** Runs SAM on every frame at 15 Hz → GPU overload. A bad mask permanently carves geometry from the TSDF.
- **Fuse then segment:** Open3D builds the local map once (cheap), runs SAM once on the live frame (on demand). SAM mask acts as a cookie-cutter on the fused cloud.

**Notes:** The winning approach: Fuse first (Open3D TSDF on keyframes), then segment (single MobileSAM pass on the live wrist frame).

---

## Slide 74 — SAM + DBSCAN Synergy: Both Problems Fix Each Other

**Content:** The 2-step pipeline from `localization-rework-critique2.md:364-391`:
1. **2D SAM cut** removes the table underneath (visual boundary in image)
2. **3D DBSCAN** removes background behind (physical air gap in 3D space)

```
[TSDF Cloud] ──► [2D SAM Mask] ──► (table gone) ──► [3D DBSCAN] ──► (background gone)
                                                                          │
                                                                          ▼
                                                                 [Pristine Object]
```

**Notes:** These two methods are complementary. Each removes what the other misses. The combined result is a clean, isolated object cloud.

---

## Slide 75 — GTSAM + FGO Could Solve the 20cm Drift

**Content:** Diagram from `localization-rework.md:193-240`:
```
OpenVINS Head ──► Odometry Factor ──┐
                                     ├──► GTSAM FGO (iSAM2)
OpenVINS Wrist ──► Odometry Factor ─┘       │
                                            ├── Kinematic Constraint (arm < 1m)
                                            ├── ArUco Priors (world marker)
                                            └── ArUco Between (head→wrist marker)
```

**Notes:** The Factor Graph Optimizer retroactively corrects historical poses when new evidence (ArUco detection) arrives. This is fundamentally different from an EKF, which can only correct the present.

---

## Slide 76 — FGO vs EKF: The Critical Difference Is Retroactive Correction

**Content:** From `localization-rework-critique.md:38-43`:
```
EKF: Wrist drifts for 3 seconds → sees ArUco → corrects current pose.
     Past keyframes remain corrupted. TSDF fusion uses bad poses.

FGO: Wrist drifts for 3 seconds → sees ArUco → re-optimizes entire 
     trajectory → past keyframes get corrected poses. TSDF fusion
     uses accurate geometry.
```

**Notes:** This "retroactive smoothing" is the key advantage. Since TSDF fusion depends on accurate per-frame poses, correcting the past prevents ghosting and blur in the fused point cloud.

---

## Slide 77 — Human Kinematic Constraint: The Arm Can Only Reach So Far

**Content:** From `localization-rework.md:608-629`:
```
If |p_wrist - p_head| > 1.0m:
    add_steep_penalty()
```

A soft-bounded inequality factor in the FGO. The optimizer is free to place the wrist within the arm's reach, but a non-linear loss kicks in if the distance exceeds ~1 meter.

**Notes:** This prevents the "arm stretching" artifact when both cameras drift in opposite directions. The kinematic constraint is the safety net that bounds the FGO's state space.

---

## Slide 78 — The Full Proposed Architecture (Not Yet Implemented)

**Content:** The consolidated pipeline from `localization-rework.md:1080-1177`:

```
Jetson: RealSense ×2 + OpenVINS ×2 → stream via Ethernet

Laptop:
  Fast Loop (30-60 Hz):
    OpenVINS odom → GTSAM FGO (sliding window 10-15s)
    Kinematic tether + ArUco priors
    → Accurate real-time poses
    
  Middle Tier (Keyframe Buffer):
    Sparsification (>10cm or >15° change)
    Recency-based overlap eviction
    → Bounded RAM usage
    
  Slow Loop (On-demand):
    Close-range tripwire (<3cm) → trigger
    Open3D TSDF fusion in 10-20cm ROI
    Single MobileSAM pass → segmented object
    → Clean grasp input
```

**Notes:** This is the aspirational architecture. Components validated so far: OpenVINS streaming, twist propagation, keyframe concepts. Pending: GTSAM integration, MobileSAM deployment, full end-to-end on-demand fusion.

---

## Slide 79 — Key Takeaways

**Content:** Bullet list:
1. **Grasp preshaping** predicts hand configuration before contact, not after
2. **Evolution Strategy** with hit-time-concentrated sampling handles the high-dimensional search
3. **Superquadric backside estimation** fills in occluded geometry for better TSDF
4. **Morton codes + BFS** create the TSDF efficiently
5. **Camera ray voting + SQ blending** determines inside/outside correctly
6. **The system runs reliably** with proper network tuning (UDP buffer fix was critical)
7. **Future direction:** MobileSAM + TSDF for segmentation, GTSAM FGO for drift-free tracking

---

## Slide 80 — Questions?

**Content:** Blank slide with "Thank you" and project URL/code references.
