# Grasp Preshaping Pipeline — Report Section Skeleton

> ~10 pages. Bullet-point skeleton with every detail to include. Use as a writing checklist.

---

## 1. Introduction and Problem Statement

- Grasp preshaping: determine optimal grasp type (cylindrical/pinch/lateral), hand pose, wrist orientation, and finger closure amount **before** the hand contacts the object
- Challenges:
  - Only the front face of objects is observed (partial point cloud)
  - ~50 ms real-time budget
  - High-dimensional search: 6D pose + grasp type + wrist rotation + closure amount
  - Must handle diverse object geometries with a single model
- Key design insight: plan in **task space** using a TSDF (not joint space or object space) + pre-computed finger LUT
- **[Fig. 1]** System context diagram: hand tracking → twist propagation → segmentation → grasp_preshaping → finger commands

---

## 2. Pipeline Architecture Overview

- Pipeline flow:
  ```
  PointCloud → Prune(ROI) → Morton Sort → TSDF Construction →
  Superquadric Backside Estimation → SMC Optimization Loop →
  Score Particles → Select Best Grasp → Output
  ```
- **[Fig. 2]** Pipeline block diagram — annotate with approximate timing per stage; highlight that **TSDF is built once and reused** across all SMC iterations (key design decision)
- Modules (Rust core library):
  - `c_api.rs` — FFI entry point, orchestrates full pipeline
  - `predictor.rs` — motion model sampling, SMC particles, twist→SE(3)
  - `pointcloud_helper.rs` — pruning, Morton ordering, TSDF construction
  - `planner.rs` — grasp scoring (3 types), sweep-for-collision, binary refinement
  - `superquadric.rs` — shape completion for unobserved backside
  - `lut_helper.rs` — finger contact LUT, dual quaternion math
  - `runtime_config.rs` — YAML-based runtime config with compile-time defaults
  - `debug_export.rs` — NPZ debug dump writer
- C++ ROS 2 bridge node (`preshaping_service_bridge_node.cpp`):
  - Subscribes: `/hand_pose`, `/hand_twist`, segmented point cloud
  - Calls Rust library via FFI on service trigger
  - Publishes: per-finger closure commands, wrist orientation
- Why Rust for the core: zero-cost abstractions, no GC pauses, memory safety, seamless C FFI
- **[Table 1]** Module responsibilities and file locations

---

## 3. Region of Interest and TSDF Construction

### ROI Prediction
- Input: current hand pose + twist (6D velocity) + covariance
- Sample future hand poses over 5s prediction horizon using screw motion model
- Twist → SE(3) via exponential map (`twist_to_se3`)
- Add Gaussian noise scaled by `sqrt(t)` and covariance
- AABB of sampled fingertip positions → ROI, inflated by hand radius (0.05m), clamped to [0.1m, 0.5m]
- **[Fig. 3]** ROI prediction: sampled hand positions + resulting AABB overlaid on point cloud

### Point Cloud Pruning
- Retain only points within ROI — reduces data before expensive TSDF construction

### Morton Ordering
- Sort pruned points by Morton code (Z-order curve) → improves spatial locality for BFS

### TSDF Construction via BFS
- Build unsigned TDF using BFS from surface voxels — each voxel gets distance to nearest surface
- Truncation band: `TRUNCATION_CELLS = 4` (20mm at 5mm resolution) — keeps representation compact
- Sign determination: per voxel, check all cameras — if behind surface + voxel-to-surface aligns with camera ray → negative (inside)
- **[Fig. 4]** 2D TSDF cross-section: truncation band, positive/negative regions, zero-crossing, camera rays

### Design Decisions
- Why TSDF (not mesh/raw points): O(1) distance queries + smooth gradients → efficient collision detection and surface normals
- Why BFS (not projective TSDF): exact nearest-surface distances, no projection artifacts, works with arbitrary camera count
- Why 5mm resolution: distinguishes finger-sized features while keeping cost manageable (~60³ = 216K voxels for 0.3m ROI)

---

## 4. Superquadric Backside Estimation

### The Problem
- Cameras only see front face → TSDF has `f32::MAX` on backside → "shadow regions" → planner can't detect collisions → grasps pass through object

### The Solution
- Fit a superquadric to observed point cloud → estimate distances in unobserved regions

### Pipeline Steps
1. **PCA orientation**: compute principal components → lock rotation (not optimized)
2. **OBB initial guess**: project points onto PCA axes → initial (a, b, c) and translation
3. **Parallel template matching** (rayon): fit 3 templates simultaneously, pick best fit:
   - Sphere: ε₁=1.0, ε₂=1.0
   - Box: ε₁=0.1, ε₂=0.1
   - Cylinder: ε₁=0.1, ε₂=1.0
4. **Gauss-Newton refinement**: 4 iterations with LM damping on 6 params (3 scale + 3 translation), rotation locked
5. **TSDF fusion** — three domains:
   - **Domain A** (Camera Authority): near observed surface — camera data wins
   - **Domain B** (SQ Authority): unobserved regions (f32::MAX) — Taubin distance fills gaps
   - **Domain C** (Blending Zone): boundary — smoothstep blending for continuity

### Design Decisions
- Why superquadrics (not deep learning): 50ms budget excludes NN inference; SQ fitting ~1-2ms
- Why only 3 templates: covers vast majority of graspable household objects; more templates = diminishing returns
- Why rotation locked from PCA: optimizing all 11 params increases solver complexity and convergence risk; PCA is reliable for most objects
- Why one-directional sign override: SQ can only *add* interior (positive→negative), never remove it → prevents holes near observed surface
- Why `SQ_MIN_SIGN_OVERRIDE_CELLS = 2`: voxels within 2 cells of surface keep camera sign → prevents artifacts from slight SQ-camera misalignment
- Fallback: too few points or high fit error → skip backside estimation, proceed with camera-only TSDF (graceful degradation)

### Figures & Tables
- **[Fig. 5]** Three superquadric templates with epsilon values and example fitted shapes
- **[Fig. 6]** TSDF cross-section with Domain A/B/C color-coded, showing backside fill
- **[Table 2]** SQ config parameters: `SQ_MAX_GN_ITERATIONS=4`, `SQ_GN_DAMPING=0.1`, `SQ_BLEND_DELTA_CELLS=3`, `SQ_MIN_FIT_POINTS=20`, `SQ_FIT_ERROR_THRESHOLD=0.15`, `SQ_MIN_SIGN_OVERRIDE_CELLS=2`, `SQ_ENABLE_BACKSIDE=true`

---

## 5. Finger Contact Lookup Table (LUT)

- Pre-computed table: (contact point, closure amount) → SE(3) transform
- Captures full Mia hand kinematics offline
- **30 anatomical contact points**: index joints (8), middle/ring/little fingertips (10), thumb adduction (3), thumb abduction (3), palm (4)
- Closure: single scalar [0.0, 1.0] parameterizing coupled finger motion
- Storage: dual quaternions at discrete samples, linear interpolation between them
- **Thumb modes**: adduction (lateral/key grip) vs. abduction (opposition, cylindrical/pinch) — separate tables
- **Max closure per grasp type**: pre-computed self-collision limits stored in LUT npz; sweep clamped to these
- Why LUT (not runtime FK): O(1) lookup, no kinematics library dependency, dual quaternion naturally handles coupled motion
- **[Fig. 7]** Hand model with labeled contact points; three grasp types with active contacts highlighted

---

## 6. Grasp Scoring and Collision Detection

### Sweep-for-Collision
- Given hand pose + grasp type: sweep finger contacts from open (closure=0) to closed (closure=max)
- First TSDF collision (distance < `collision_tol` = 10mm) → determines closure amount
- Locked points (palm) checked at sample 0 — if colliding → Tier 3

### Binary Refinement
- Coarse collision sample → binary search refines closure to within `BINARY_SEARCH_TOL` (5%)

### The Four-Tier Scoring System

| Tier | Condition | Score | Meaning |
|------|-----------|-------|---------|
| **Tier 1** | Collision + ≥min_contacts + ≥min_fingers (thumb+index) | 0.8 + 0.2 × contact_fraction | Valid grasp |
| **Tier 2** | Collision but too few finger groups or missing thumb/index | 0.25 × contact_fraction + 0.25 × proximity_penalty | Soft rejection — gives gradient |
| **Tier 3** | Collision at closure=0 (palm already inside object) | 0.1 | Bad starting position |
| **Tier 4** | No collision at all | 0.0–0.05 (proximity score) | Hand misses object — but proximity gives direction |

- Why tiered (not binary hit/miss): binary gives zero gradient for optimization; tiers create smooth landscape guiding optimizer from "approaching" → "contacting" → "wrapping" → "stable grasp"
- Tier 4 proximity: sparse check (palm + index tip + thumb tip at mid-closure) → ~8× cheaper than full sweep
- Tier 2 proximity penalty: measures how close missing fingers are to surface → optimizer gets gradient to improve finger placement

### Active Contact Analysis
- `find_active_contacts`: contacts near zero-crossing (|dist| < 2×collision_tol) — filters out deep-interior contacts from negative TSDF regions (bug fix: without this, grasps not touching surface got high scores)
- `compute_alignment`: dot product of finger closing direction vs. surface normal — measures "pushing into" the surface
- `compute_force_closure`: 1 − |mean normal| — well-distributed opposing contacts score higher

### Combined Score
```
combined = (w_prob × probability + w_align × alignment + w_fc × force_closure + w_contact × contact_score)
           / (w_prob + w_align + w_fc + w_contact)
```
- Default weights: probability=1.0, alignment=1.0, force_closure=1.0, **contact_score=3.0**
- Contact score weighted 3× because it encodes the most important signal: is the hand actually forming a grasp?

### Figures & Tables
- **[Fig. 8]** Four-tier scoring landscape vs. closure amount — annotate with example hand configs at each tier
- **[Table 3]** Grasp type specs: cylindrical (17 score contacts, min=3, thumb abduction), pinch (2 score contacts, min=2, thumb abduction), lateral (5 score contacts, min=2, thumb adduction)

---

## 7. Sequential Monte Carlo (SMC) Optimization

### Why Optimization
- Search space: ~9 effective DOF (6D pose + grasp type + wrist rotation + closure)
- Exhaustive search infeasible; motion model gives prior but optimal grasp needs iterative refinement
- Alternatives considered and rejected: CEM (more complex, similar result), Bayesian optimization (overkill, doesn't parallelize), grid refinement (doesn't handle rotation), brute-force more samples (wastes compute on poor regions)

### SMC Algorithm
1. **Iteration 0**: 20,000 particles from motion model (twist + covariance); random grasp type; random wrist rotation ∈ [−π/2, π/2]
2. **Score all particles**: parallel via rayon
3. **Select elites**: top 5% (1,000) by combined score
4. **Resample**: 20,000 new particles around elites with Gaussian jitter; std decays as `initial_std × 0.7^iteration`
5. **Repeat**: up to 5 iterations; early stop if best score changes < 0.01 after min 3 iterations

### Key SMC Enhancements (each solved a specific observed problem)

- **Elite injection** (top 1% preserved as exact copies):
  - Problem: best grasp found in early iteration lost during resampling → ROS node reported worse grasp than debug visualizer
  - Solution: carry best-so-far forward unchanged
- **Score-weighted parent selection**:
  - Problem: uniform elite selection doesn't exploit best-known regions
  - Solution: parents chosen proportional to score; bounded [0,1] so no single elite dominates
- **Grasp type inheritance with 10% mutation**:
  - Problem: "type collapse" — one grasp type dominates early, eliminates exploration
  - Solution: 90% inherit parent type (exploitation), 10% mutate to performance-weighted random type (exploration)
- **Wrist rotation perturbation** (not re-randomization):
  - Problem: re-randomizing wrist each iteration destroys convergence
  - Solution: perturb parent's wrist angle with decaying Gaussian; ±17° early → ±1.4° late
- **sample_probability = 1.0 for resampled particles**:
  - Problem: inherited motion model probability is meaningless for jittered particles (position doesn't come from twist)
  - Solution: set to 1.0 → score becomes purely about grasp quality after iteration 0

### Computational Budget
- TSDF built once (dominant cost); each iteration only re-samples poses and re-scores
- 20,000 particles scored in parallel per iteration
- Target: <50ms total

### Figures & Tables
- **[Fig. 9]** SMC convergence: best score vs. iteration across multiple runs
- **[Fig. 10]** Particle distribution at iterations 0, 2, 4 — spatial clustering as variance decays
- **[Table 4]** SMC parameters: `ITERATIONS=5`, `DECAY_RATE=0.7`, `ELITE_RATIO=0.05`, `ELITE_PRESERVE_RATIO=0.01`, `GRASP_TYPE_MUTATION_RATE=0.1`, `SMC_CONVERGENCE_TOL=0.01`, `SMC_MIN_ITERATIONS=3`, `PREDICTION_SAMPLES=20000`, `INITIAL_PROPOSAL_STD_V=0.002`, `INITIAL_PROPOSAL_STD_OMEGA=0.005`, `INITIAL_PROPOSAL_STD_WRIST=0.3`

---

## 8. System Integration and ROS 2 Bridge

### FFI Boundary
- Rust exposes C ABI: `grasp_preshaping_compute(request*, response*, message_buf, buf_len)`
- Flat structs: pose (position + quaternion), twist (linear + angular), point cloud view (raw pointer + offsets), camera positions
- C++ bridge loads `.so` via `dlopen` at runtime
- API versioning: `grasp_preshaping_api_version()` returns uint (currently 5) — C++ checks compatibility

### Bridge Node Role
- Subscribes: `/hand_pose`, `/hand_twist`, segmented point cloud
- Looks up camera positions via TF2
- Calls Rust library on `/grasp_preshaping/compute_grasp` service trigger
- Publishes: per-finger closure (thumb/index/mrl), wrist orientation, target hand pose
- Applies `preshaping_closure_fraction` (ROS param, default 0.3) to scale planned closure for preshaping

### Configuration
- All tunables loaded from `config/grasp_preshaping.yaml` at runtime
- Compile-time defaults in `runtime_config.rs` as fallback
- `OnceLock` singleton for thread-safe one-time init

### Debug Visualization
- When enabled: exports NPZ with full TSDF, all scored grasps (all SMC iterations), fitted superquadric params, input pose/twist
- Python visualizer (`visualize_grasp_debug.py`): renders TSDF cross-sections, grasp poses, SQ wireframe, iteration filtering

### Figure
- **[Fig. 11]** ROS 2 node graph: preshaping_service_bridge_node + subscriptions/publications/service + twist_propagation_node context

---

## 9. Engineering Challenges and Lessons Learned

| # | Challenge | Root Cause | Resolution |
|---|-----------|------------|------------|
| 1 | TSDF sign wrong with opposing cameras | Majority vote (`behind_count > n_cams/2`) fails: with 2 opposite cameras, each sees voxel as "in front" from its side, so `behind_count` only reaches 1, never >1 | Changed to `inside_votes > 0` (any camera with good alignment confirms inside) + superquadric tiebreaker |
| 2 | High scores for grasps not touching surface | After sign fix, negative voxels (deep inside) triggered as "collisions" → contact score high despite no surface contact | Surface proximity filter in `find_active_contacts`: only count contacts with |dist| < threshold, reject deep-interior |
| 3 | SMC loses best grasp across iterations | ROS node selected from last iteration only; debug visualizer searched all iterations → different "best" | Track overall best across all iterations for output + elite injection to prevent loss |
| 4 | Closure fraction had no effect | Rust `PRESHAPING_CLOSURE_FRACTION` was dead code; actual control in C++ ROS param (default 0.3) | Removed dead Rust constant; centralized config via ROS parameter |
| 5 | SQ artifacts near observed surface | SQ surface slightly misaligned with camera surface → sign flips near boundary (holes, floating negatives) | One-directional override (only positive→negative) + `SQ_MIN_SIGN_OVERRIDE_CELLS=2` preserves camera authority near surface |

- **[Table 5]** Above table (or prose version) — shows iterative debugging process

---

## 10. Summary and Contributions

Key properties to recap:
- Real-time (~50ms) grasp planning from partial point cloud
- Three grasp types with automatic selection via SMC
- Superquadric shape completion for unobserved backsides (graceful fallback)
- Tiered scoring → optimization gradient across full quality spectrum
- Pre-computed finger LUT → no runtime kinematics
- YAML-configurable with compile-time defaults
- Comprehensive debug visualization (NPZ + Python visualizer)

Design philosophy:
- **Robustness**: graceful degradation (SQ fails → camera-only TSDF), early termination on convergence, fallback paths
- **Efficiency**: TSDF built once, parallel scoring (rayon), Morton-ordered spatial data, LUT O(1) lookups
- **Observability**: debug dumps capture everything for offline analysis

---

## Figures Checklist

| # | What to Show | Source Material |
|---|-------------|----------------|
| Fig. 1 | System context: hand tracking → twist → segmentation → preshaping → finger cmds | `plans/2026-05-07-twist_propagation_node-v1.md:190-219` |
| Fig. 2 | Pipeline block diagram with timing annotations | `c_api.rs:358-621` |
| Fig. 3 | ROI prediction: sampled poses + AABB on point cloud | `predictor.rs` motion model |
| Fig. 4 | 2D TSDF cross-section: truncation, signs, zero-crossing, camera rays | `pointcloud_helper.rs` |
| Fig. 5 | Three SQ templates with ε values and example shapes | `superquadric.rs:47-63` |
| Fig. 6 | TSDF domain fusion A/B/C color-coded cross-section | Backside estimation plan |
| Fig. 7 | Hand model with contact points; 3 grasp types highlighted | `lut_helper.rs` + `planner.rs` specs |
| Fig. 8 | Four-tier scoring landscape vs. closure amount | `planner.rs:288-450` |
| Fig. 9 | SMC convergence: best score vs. iteration | Debug dump data |
| Fig. 10 | Particle distribution at iterations 0/2/4 | Debug dump data |
| Fig. 11 | ROS 2 node graph | Bridge node + system integration |

## Tables Checklist

| # | What to Include |
|---|----------------|
| Table 1 | Module responsibilities + file locations |
| Table 2 | SQ config params with defaults |
| Table 3 | Grasp type specs: contacts, min_contacts, min_fingers, thumb mode |
| Table 4 | SMC params: iterations, decay, elite ratio, mutation rate, proposal stds |
| Table 5 | Challenges summary: problem → root cause → resolution |
