# Test 1: Software Verification — Retrospective Insights

> Captures all key findings, root causes, fixes, and design decisions from the Test 1 development session (2026-05-22 to 2026-05-23).

---

## 1. Root Causes of Low Intent Precision Δ

### 1.1 SMC Stochasticity Dominates the Signal

The SMC sampler at `predictor.rs:571` uses `rand::rng()` (unseeded CSPRNG) — a fresh entropy source every call. Randomness enters at three points:
- **Initial particle sampling** — draws 20K random grasp hypotheses from the proposal distribution
- **Resampling** — weighted random selection of elites for the next iteration
- **Grasp type mutation** — 10% chance per particle to randomly switch grasp type

**Impact**: 100 repetitions per object still produce ~30-50% variance in grasp type selection. The multi-view signal (+1–5% in score) is buried under ±20% noise from the sampler.

**Mitigation considered**: Increasing particles from 20K to 100K reduces noise by ~55% (law of large numbers) but increases latency from ~60ms to ~300ms — violating IDE (≤100ms) and approaching MAR (≤400ms).

### 1.2 Approach Geometry Was Suboptimal

**Original setup**: Hand approached straight along +X axis. Wrist camera also looked along +X from 16cm above the palm. Both cameras saw the same face of the object — just from different heights. No complementary viewing angle existed, so multi-view fusion added almost no new surface information.

**Fix applied**: Rotated all approach poses 45° around the Z-axis. The wrist camera now sees a 45°-rotated face while the head camera sees the top. This creates genuinely complementary views.

### 1.3 Back-Face Culling Was Missing

**Original occlusion simulation**: `depth_buffer_occlude` at `occlusion.py` used only frustum culling + z-buffer depth testing. Points whose surface normal faced away from the camera were still included. Since the head camera looked down at convex objects from above, it could "see" all surfaces — including the bottom.

**Fix applied**: Added back-face culling — each point's surface normal is estimated from its ~30 nearest neighbors via PCA. Points where `dot(normal, to_camera) < 0` are culled. Head camera coverage dropped from 100% to 35–53%, creating genuine occlusion for multi-view to address.

### 1.4 Config Reload Cache Bug

**Critical bug**: The Rust runtime config at `runtime_config.rs:240` uses `OnceLock<RuntimeConfig>` — loaded once and never reloaded. The score sweep created a new `GraspLibrary()` and set `GRASP_CONFIG_PATH`, but the `.so`'s static `OnceLock` persisted from the first call. **All 7 sweep iterations used 20K samples**, producing the flat score-vs-samples data.

**Fix**: Added `reload_config()` to the Rust C API via `RUNTIME_CONFIG.take()` to clear the cache, forcing a reload on the next access.

### 1.5 Tier 3 Scoring Bug

**Bug**: `planner.rs:334` — Tier 3 (hand inside object at sample 0) assigned `contact_score = 0.1`, while Tier 4 (hand far from object, no collision) assigned `contact_score = 0.0–0.05`. **The optimizer was rewarded for placing the hand inside the object**. Combined with zero TSDF gradient beyond 2cm, this created a perverse incentive.

**Fix**: Changed Tier 3 to use superquadric depth gradient (0.04–0.10), making it slightly better than Tier 4 far (0.0) but worse than Tier 4 near (0.05) — correctly incentivizing approach from outside.

---

## 2. Architectural Improvements

### 2.1 Superquadric Proximity Fallback

**Problem**: TSDF truncation at 2cm (`truncation_cells=4 × resolution=5mm`) meant `sparse_proximity_distance` at `planner.rs:538-562` returned `f32::MAX` for any point >2cm from the surface. The SMC had **zero gradient** toward the object from >2cm away. Only ~20% of particles (those landing within 2cm by random chance) got any score signal.

**Fix**: Threaded `sq_params` (superquadric parameters, already fitted to every point cloud at `c_api.rs:399`) through to `score_grasp`. When TSDF returns no gradient, fall back to `superquadric.taubin_distance()` — a global signed distance approximation that works everywhere.

### 2.2 Early Termination

**New feature**: With SQ distance available, skip the expensive `sweep_for_collision` for particles >15cm from the object surface. The check is at `planner.rs:318-324` — if `sq_dist > 0.15`, the particle is too far to possibly make contact, so assign Tier 4 with SQ-based proximity and skip collision sweeps.

**Impact**: ~80% of particles are >15cm from the object at initial sampling. These avoid the O(n×m) collision sweep, reducing mean latency by ~15–20%.

### 2.3 Score-Based Metric

**New metric**: The original `grasp_accuracy_pct` (binary type match against baseline) is too noisy. Replaced with score-based analysis:

| Metric | What it measures | Stability |
|---|---|---|
| `mean_score` | Average grasp quality across all trials | Good (100-trial average) |
| `max_score` | Best grasp found | Moderate (upper tail) |
| `score_distribution` | Full histogram SV vs MV | Excellent (visual) |

The `combined_score` formula at `planner.rs:23-36`:
```
combined = (1.0×probability + 1.0×alignment + 1.0×force_closure + 3.0×contact_score) / 6.0
```

---

## 3. Test Infrastructure Findings

### 3.1 Hardware Scaling Factor

| Metric | Desktop (i5-13600K) | Live System (Docker/Embedded) | Ratio |
|---|---|---|---|
| Mean latency | ~64 ms | 215–415 ms | 3–6× |
| P95 latency | ~80 ms | ~350 ms | 4× |

The test measurements are valid for **algorithmic correctness** but absolute numbers must be scaled to the target hardware. The pipeline passes MAR on any hardware where the ratio is <5×.

### 3.2 Point Cloud Size Realism

| Source | Points | Notes |
|---|---|---|
| Raw D435i (640×480) | ~300K per camera | Two cameras = ~600K raw |
| After distance + voxel filter | 20K–100K | 5mm voxel grid |
| After segmentation | 5K–50K | Object extraction |
| Test objects | **30K** | Matches typical live cloud |

Increased from 10K → 30K in `generate_objects.py` to match live system.

### 3.3 Convex Objects Mask Multi-View Advantage

**Finding**: Convex parametric objects (cylinders, boxes, spheres) show **no self-occlusion** — every surface point is visible from any viewpoint. Multi-view fusion cannot add new surface information for convex objects.

**Impact on test**: 5/12 test objects are convex. These objects inherently show zero multi-view advantage, diluting the mean Δ.

**Recommendation**: The test should use primarily non-convex objects (YCB meshes, L-blocks, notched shapes) to demonstrate the multi-view advantage. Alternatively, add a convexity filter to the analysis.

---

## 4. Key Config Parameters

| Parameter | Production | Baseline | Notes |
|---|---|---|---|
| `prediction_samples` | 20,000 | 100,000 | 5× more particles |
| `smc_iterations` | 5 | 10 | 2× more iterations |
| `approach_distance` | 15 cm | — | Increased from 10cm to avoid hand-object overlap |
| `truncation_cells` | 4 (2cm) | 4 (2cm) | Not increased — SQ fallback handles >2cm |
| `depth_tolerance` | 0.2% of median depth | — | Distance-scaled, not fixed 5mm |
| `back_face_culling` | Enabled | — | New — rejects points facing away from camera |
| `early_termination` | Enabled (15cm) | — | New — skips collision sweep for distant particles |

---

## 5. Figure Generation Pipeline

| Figure | Content | Key Changes |
|---|---|---|
| `fig1`–`fig2` | Latency boxplot + summary | Project colors `#5099e9`, `#fffdf6`, `#f4fbf9` |
| `fig3`–`fig4` | Intent precision + delta | Uses score metric as primary |
| `fig5` | CDF + correctness dual-panel | Back-face culling enabled |
| `fig7` | Cloud coverage | Shows real occlusion (35-53% head) |
| `fig7b` | Convexity analysis | Grouped by convex/non-convex |
| `fig7c` | Per-view coverage | **4 panels, equal axes, back-face culling** |
| `fig7d` | Benchmark violin | **Baseline (20 reps) vs SV vs MV, with max-score diamonds** |
| `fig9` | 3D scene | **4 panels: Overview, Top, Side, Rear — equal axis scaling** |
| `fig12` | Object gallery | **All 12 objects with SQ overlay, convexity coloring** |
| `fig13` | Score vs samples | **Sweep 1K–100K, proper config reload, latency tradeoff** |

---

## 6. Remaining Limitations

1. **SMC stochasticity**: The fundamental limitation. A deterministic grasp planner would produce cleaner results but would not generalize to novel objects the way SMC does.

2. **IDE not met**: Mean Δ = +0.2% falls far short of 10%. Even with all fixes, the multi-view advantage is small because:
   - Convex objects show no multi-view benefit
   - The SMC's noise dominates the signal
   - The wrist camera's forward-looking perspective from 16cm above adds limited new information

3. **Score ceiling**: Baseline at 100K samples achieves ~0.34 mean score, while real-time at 20K achieves ~0.24. The gap represents a ~30% quality loss from reduced sampling, which cannot be closed by multi-view fusion alone.

4. **Sweep validation**: The corrected sweep (using `reload_config()`) needs to run to completion to validate the score-vs-samples curve. The subprocess-per-config approach is slow but correct.

---

## 7. Future Recommendations

1. **Replace binary type match with score-based metric** as the primary Intent Precision Δ
2. **Use only non-convex test objects** to isolate and measure the multi-view advantage
3. **Add deterministic grasp planner** as a comparison baseline alongside the SMC
4. **Run on target hardware** to get realistic latency measurements with the hardware scaling factor
5. **Increase sweep baseline to 50+ reps** at each sample count for statistically robust convergence analysis
6. **Add volumetric TSDF fusion** to the multi-view pipeline (instead of simple concatenation) for a more realistic sensor fusion model
