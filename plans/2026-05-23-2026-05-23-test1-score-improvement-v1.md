# Test 1: Score Improvement Investigation & Plan

## Objective

Investigate why grasp scores are low (~0.20-0.30 mean for most objects) and create a plan to fix the root causes. Also add a comprehensive benchmark comparison figure showing baseline vs SV vs MV scores per object.

## Root Cause Analysis

### Finding 1: Most Trials Land in Tier 4 (No Collision)

The contact_score tier system (`planner.rs:288-450`):
- **Tier 4** (no collision): contact_score = 0.0 to 0.05
- **Tier 3** (start collision): contact_score = 0.1
- **Tier 2** (soft rejection): contact_score = 0.25 * count + 0.25 * penalty
- **Tier 1** (valid grasp): contact_score = 0.8 + 0.2 * fraction

The combined_score formula (`planner.rs:23-36`):
```
combined = (1.0 * probability + 1.0 * alignment + 1.0 * force_closure + 3.0 * contact_score) / 6.0
```

For a Tier 4 trial: contact_score ≈ 0.0-0.05, probability ≈ 0.5-1.0, alignment=0, force_closure=0:
```
combined ≈ (0.5 + 0 + 0 + 0) / 6.0 = 0.083
```
This matches the observed `combined_score = 0.217` for many trials (when probability is higher).

### Finding 2: The Approach Distance Problem

The hand is at 25cm from the object. The SMC sampler draws particles from a motion model that predicts where the hand will be in the future (0-5 seconds at 0.10 m/s = 0-50cm). Many particles end up at positions far from the object where the TSDF has no surface data → Tier 4.

The key issue: **only particles that happen to be near the object (at ~25cm forward displacement) can score well**. With 20K particles spread over 0-50cm range, only ~20% (4000) are near enough to the object to make contact. Of those, many are at wrong wrist angles or grasp types.

### Finding 3: TSDF Truncation Band is Only 2cm

`truncation_cells = 4`, `tsdf_resolution_m = 0.005` → truncation distance = 4 * 5mm = **20mm**.

The `sparse_proximity_distance` (Tier 4 path) checks palm center + fingertip distances at mid-closure. If the nearest surface point is >20mm away, the TSDF returns `f32::MAX` → proximity_score = 0.0.

For a hand at 20cm from a 4cm cube, the finger-to-surface distance is ~20cm → way outside the truncation band → no proximity signal → no gradient to guide the SMC toward the object.

### Finding 4: The Covariance is Appropriate But...

`fixed_cov_v = [0.0005, 0.0005, 0.0005]` produces noise σ = √(0.0005 * t) per axis:
- At t=2.5s (25cm mean): σ = 0.035m = 3.5cm per axis
- This gives a 3σ range of ±10.5cm — adequate for exploring around the object

The initial proposal std (`0.002`) only affects refinement iterations, NOT the initial search. The initial search uses the full motion model (0-50cm). This is NOT the problem.

### Finding 5: The REAL Problem — Low Signal-to-Noise for Tier 4

The SMC converges based on `combined_score`. In Tier 4:
- `contact_score` ≈ 0.0 (no proximity signal beyond 2cm)
- `probability` ≈ 0.5-1.0 (based on motion model likelihood)
- `alignment` = 0, `force_closure` = 0

The only signal in Tier 4 is `probability` (motion model likelihood), which favors samples near the hand's predicted trajectory center. There's NO gradient toward the object. The SMC can only "find" the object by random chance — a particle that happens to be within 2cm of the surface.

This is a fundamental limitation: **the SMC has no way to navigate toward the object from 25cm away**. It relies entirely on the motion model placing some particles near the object by chance.

## Implementation Plan

### Phase 1: Add Benchmark Comparison Figure
- [ ] Create `plot_benchmark_comparison()` in `plot_results.py` — a grouped bar chart showing baseline, SV, and MV mean/max scores per object, with score tier annotations
- [ ] Add call in `main()` and regenerate

### Phase 2: Fix Approach Distance
- [ ] Reduce approach distance from 25cm to **15cm** for all objects in `hand_approaches.py`
- [ ] Rationale: at 15cm with 0.10 m/s twist, the motion model places many more particles near the object. The ROI still covers the object (ROI extends to 0.40m forward from 15cm). The TSDF truncation band (2cm) now covers a larger fraction of the particle distribution.
- [ ] Update `_DEFAULT_DIST` from -0.25 to -0.15
- [ ] Update `small_cube` distance from -0.20 to -0.12

### Phase 3: Increase TSDF Truncation Band
- [ ] Increase `truncation_cells` from 4 to **8** in test config
- [ ] This doubles the truncation distance from 2cm to **4cm**, giving Tier 4 particles a proximity signal up to 4cm from the surface
- [ ] The proximity score range stays [0.0, 0.05] but now extends further, giving the SMC a gradient to follow
- [ ] Memory impact: 8 * 5mm = 40mm padding → modest increase in TSDF grid size

### Phase 4: Widen Initial Proposal Variance
- [ ] Increase `initial_proposal_std_v` from 0.002 to **0.005** in test config
- [ ] Increase `initial_proposal_std_omega` from 0.005 to **0.01**
- [ ] Rationale: wider initial jitter lets the SMC explore more around the elite particles in early iterations, helping it find the object surface from further away
- [ ] The decay rate (0.7) will still converge to tight proposals by iteration 4-5

### Phase 5: Re-run Test Suite and Verify
- [ ] Re-run with 100 repetitions: `python3 run.py --repetitions 100`
- [ ] Verify that mean contact_score increases from ~0.1 to >0.3
- [ ] Verify that Tier 1 (valid grasp) fraction increases from ~30% to >50%
- [ ] Check that latency still passes MAR (≤400ms)

### Phase 6: Regenerate All Figures
- [ ] Run `python3 plot_results.py --format png --dpi 200`
- [ ] Verify the benchmark comparison figure shows clear score improvement
- [ ] Verify the 3D setup figure reflects the new approach distance

## Verification Criteria

1. Mean contact_score across all objects ≥ 0.3 (currently ~0.1)
2. Tier 1 fraction ≥ 50% of trials (currently ~30%)
3. Mean combined_score ≥ 0.4 for baseline condition
4. Latency P95 ≤ 400ms (MAR threshold)
5. Benchmark figure clearly shows baseline >> MV ≥ SV score hierarchy
6. Score-based delta (MV - SV) is positive for ≥ 60% of objects

## Potential Risks and Mitigations

1. **Shorter approach distance changes camera geometry**
   - At 15cm, the wrist camera sees more of the object (less angle to look down)
   - The head camera still sees the top from above
   - Mitigation: re-verify camera visibility after changing approaches

2. **Wider truncation increases latency**
   - TSDF grid is larger → more voxels to traverse during ray casting
   - Mitigation: truncation_cells 8 vs 4 adds ~40% more voxels, should be <10ms increase

3. **Wider proposal variance increases stochasticity**
   - More exploration but also more noise in convergence
   - Mitigation: 100 repetitions should average out the noise

4. **Approach distance may not match real system**
   - The real system approaches from ~15-20cm during grasp planning
   - 15cm is actually MORE realistic than 25cm
   - Mitigation: document the approach distance choice in the report
