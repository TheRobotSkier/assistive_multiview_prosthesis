# Test 1: Score Improvement Investigation & Plan

## Objective

Investigate why grasp scores are low (~0.20-0.30 mean for most objects) and create a plan to fix the root causes. Also add a comprehensive benchmark comparison figure showing baseline vs SV vs MV scores per object.

## Root Cause Analysis

### Finding 1: Tier 3 Scores Higher Than Tier 4 — A Scoring Bug

The contact_score tier system (`planner.rs:288-450`):
- **Tier 4** (no collision, hand far from object): `contact_score = 0.0 to 0.05`
- **Tier 3** (hand already inside object at sample 0): `contact_score = 0.1` ← **BUG: Higher than Tier 4!**
- **Tier 2** (collision but missing thumb/index): `contact_score = 0.25 * count + 0.25 * penalty`
- **Tier 1** (valid grasp): `contact_score = 0.8 + 0.2 * fraction`

The SMC optimizer maximizes combined_score. Tier 3 (hand inside object) scores HIGHER than Tier 4 (hand outside, no collision). This means the optimizer is **incentivized to push particles into the object** rather than toward it from outside.

**Fix**: Change Tier 3 `contact_score` from `0.1` to `0.02` (below the max Tier 4 proximity score of 0.05). This correctly penalizes penetration.

### Finding 2: Tier 4 Proximity Relies on TSDF (Not Independent)

`sparse_proximity_distance` (`planner.rs:538-562`) queries `tsdf.get_distance()` for 3 key points (palm, index tip, thumb tip). It is NOT an independent distance calculation. If these points are outside the TSDF grid (>2cm from surface due to `truncation_cells=4 × resolution=5mm`), it returns `f32::MAX` → `proximity_score = 0.0`.

**Consequence**: The SMC has **zero gradient** toward the object from >2cm away. It can only find the surface by random chance — a particle that happens to land within 2cm of the surface.

### Finding 3: The Approach Distance Spreads Particles Too Thinly

At 25cm with a 0.10 m/s twist and 5s horizon, the motion model distributes 20K particles across 0–50cm of forward travel. Only ~20% of particles land near the object (~25cm). The rest are in Tier 4 with zero gradient, scoring ~0.22 combined.

## Implementation Plan

### Phase 1: Fix Tier 3 Scoring Bug
- [ ] In `src/grasp_preshaping/src/planner.rs:334`, change `contact_score: 0.1` to `contact_score: 0.02`
- [ ] Rationale: Tier 3 (hand inside object) should score BELOW Tier 4 (hand near but not penetrating). Current value of 0.1 actively rewards penetration.
- [ ] This is a production code change — it affects the live system too. The current behavior is incorrect for any use case.

### Phase 2: Add Benchmark Comparison Figure
- [ ] Create `plot_benchmark_comparison()` in `plot_results.py` — a grouped bar chart showing baseline, SV, and MV mean/max scores per object
- [ ] Use project colors: baseline in `#5099e9`, SV in lighter blue, MV in `#55a868` green
- [ ] Add score tier annotations (Tier 1/2/3/4 thresholds as horizontal lines)
- [ ] Add call in `main()` and regenerate

### Phase 3: Reduce Approach Distance
- [ ] Reduce approach distance from 25cm to **15cm** in `hand_approaches.py`
- [ ] Update `_DEFAULT_DIST` from -0.25 to -0.15
- [ ] Update `small_cube` distance from -0.20 to -0.12
- [ ] Rationale: at 15cm with 0.10 m/s twist, many more particles land near the object. The ROI still covers it (extends to ~0.40m forward). 15cm is also more realistic for the grasp planning phase.

### Phase 4: Increase TSDF Truncation Band
- [ ] Increase `truncation_cells` from 4 to **8** in the TEST config only (`tests/test1_software_verification/config/grasp_preshaping.yaml`)
- [ ] This doubles the proximity signal range from 2cm to **4cm**, giving the SMC a gradient to follow from further away
- [ ] Do NOT change the production config — this is a test-specific tuning

### Phase 5: Rebuild Rust Library and Re-run Test Suite
- [ ] Rebuild: `cd src/grasp_preshaping && cargo build --release --lib`
- [ ] Copy `.so` to `lib/` directory
- [ ] Re-run with 100 repetitions: `python3 run.py --repetitions 100`
- [ ] Verify that Tier 1 fraction increases significantly
- [ ] Verify that Tier 3 no longer attracts particles

### Phase 6: Regenerate All Figures
- [ ] Run `python3 plot_results.py --format png --dpi 200`
- [ ] Verify the benchmark comparison figure shows clear score improvement
- [ ] Verify the 3D setup figure reflects the new approach distance

## Verification Criteria

1. Tier 3 `contact_score` = 0.02 (below Tier 4 max of 0.05)
2. Mean contact_score across all objects ≥ 0.3 (currently ~0.1)
3. Tier 1 fraction ≥ 50% of trials (currently ~30%)
4. Mean combined_score ≥ 0.4 for baseline condition
5. Latency P95 ≤ 400ms (MAR threshold)
6. Benchmark figure clearly shows baseline >> MV ≥ SV score hierarchy
7. Score-based delta (MV - SV) is positive for ≥ 60% of objects

## Potential Risks and Mitigations

1. **Tier 3 fix changes production behavior**
   - The current Tier 3 score (0.1) is incorrect for ALL use cases
   - Changing to 0.02 is strictly an improvement — it removes the incentive to penetrate
   - Mitigation: this is a bug fix, not a behavioral change

2. **Shorter approach distance changes camera geometry**
   - At 15cm, the wrist camera sees more of the object (less vertical angle)
   - The head camera still sees the top from above
   - Mitigation: re-verify camera visibility after changing approaches

3. **Wider truncation increases latency**
   - TSDF grid is larger → more voxels to traverse
   - Mitigation: truncation_cells 8 vs 4 adds ~40% more voxels, should be <10ms increase

4. **Approach distance may not match real system**
   - The real system approaches from ~15-20cm during grasp planning
   - 15cm is actually MORE realistic than 25cm
   - Mitigation: document the approach distance choice in the report
