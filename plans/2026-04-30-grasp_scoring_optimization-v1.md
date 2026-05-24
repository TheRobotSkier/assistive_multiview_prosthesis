# Grasp Scoring and Sampling Optimization Plan

## Objective

Optimize the grasp preshaping pipeline to achieve high-quality grasp selection with significantly fewer samples (target: 2,000-3,000 total vs current 80,000) while maintaining or improving success rates. Address zero-score scenarios in low-sample conditions and improve score combination logic.

## Current State Analysis

### Sampling Performance
- **Current**: 10,000 samples × 8 iterations = 80,000 total evaluations
- **Time Cost**: ~80,000 grasp evaluations per planning cycle
- **Convergence**: May converge earlier but runs full iterations regardless

### Scoring Issues
- **Zero Best Scores**: When contact_score=0.0 for all samples, combined_score becomes 0.0
- **Score Combination**: Linear weighting may not be optimal for sparse successful samples
- **Gradient Signal**: Limited directional information for optimization

### SMC Parameters (config.rs:30-37)
```rust
ITERATIONS: 8
DECAY_RATE: 0.75
ELITE_RATIO: 0.1
INITIAL_PROPOSAL_STD_V: 0.001
INITIAL_PROPOSAL_STD_OMEGA: 0.002
```

## Implementation Plan

### Phase 1: SMC Parameter Optimization

- [ ] **Reduce sample count per iteration** from 10,000 to 2,500
  - Rationale: Current 10,000 is likely overkill; SMC focuses samples around elites
  - Implementation: Modify `config.rs:16` PREDICTION_SAMPLES to 2500
  - Expected impact: 4x reduction in evaluations (80,000 → 20,000)

- [ ] **Increase elite ratio** from 0.1 to 0.2
  - Rationale: With fewer samples, need more elites to maintain diversity
  - Implementation: Modify `config.rs:33` ELITE_RATIO to 0.2
  - Expected impact: Better exploration with smaller population

- [ ] **Adjust decay rate** from 0.75 to 0.7
  - Rationale: Faster decay helps convergence with fewer iterations
  - Implementation: Modify `config.rs:32` DECAY_RATE to 0.7
  - Expected impact: Earlier focus on promising regions

- [ ] **Increase initial proposal variance** for position
  - Rationale: Current 0.001m (1mm) may be too conservative for initial exploration
  - Implementation: Modify `config.rs:36` INITIAL_PROPOSAL_STD_V to 0.003
  - Expected impact: Better initial coverage of search space

### Phase 2: Adaptive Iteration Control

- [ ] **Implement early termination based on score stability**
  - Rationale: Stop iterating when scores converge, avoiding wasted computation
  - Implementation: Add convergence check in `c_api.rs:401-414` SMC loop
  - Logic: If best score change < threshold for 2 consecutive iterations, break
  - Threshold: 0.01 for combined_score
  - Expected impact: 20-40% reduction in iterations on average

- [ ] **Add minimum iteration guard** (3 iterations)
  - Rationale: Prevent premature termination before sufficient exploration
  - Implementation: Enforce minimum iterations before early termination check
  - Expected impact: Ensure basic exploration before convergence check

### Phase 3: Enhanced Score Combination

- [ ] **Implement fallback scoring for zero-contact scenarios**
  - Rationale: When contact_score=0.0 for all samples, use distance-to-object as tiebreaker
  - Implementation: Modify `planner.rs:24-37` combined_score calculation
  - Logic: If max contact_score < 0.1, weight sample_probability higher (2.0 instead of 0.5)
  - Expected impact: Meaningful selection even when no collisions found

- [ ] **Add distance-to-surface metric for Tier 4 grasps**
  - Rationale: Provide gradient signal when hand misses object entirely
  - Implementation: Modify `planner.rs:300-308` Tier 4 scoring
  - Logic: Compute minimum TSDF distance across all sweep points, convert to score [0.0-0.05]
  - Formula: `score = 0.05 * exp(-min_distance / 0.01)` (closer = higher score)
  - Expected impact: Directional guidance toward object

- [ ] **Implement adaptive weight adjustment based on sample quality**
  - Rationale: Dynamically adjust weights based on score distribution
  - Implementation: Add weight computation in `c_api.rs:402` after scoring
  - Logic: If max contact_score < 0.3, increase w_contact_score to 5.0; else use default 3.0
  - Expected impact: Stronger focus on finding contacts when scarce

### Phase 4: Gradient-Informed Sampling

- [ ] **Add biased resampling toward contact gradients**
  - Rationale: Use surface normals to guide resampling direction
  - Implementation: Modify `predictor.rs:279-362` resample_around_elites
  - Logic: For elite particles with collision, compute average contact normal, bias jitter toward object
  - Implementation detail: Add small bias vector (0.0005m) toward -normal
  - Expected impact: Faster convergence to valid grasps

- [ ] **Implement grasp-type-specific proposal adjustments**
  - Rationale: Different grasp types have different optimal search patterns
  - Implementation: Modify resampling to use different std per grasp type
  - Logic: Cylindrical: larger position variance (0.004), Pinch: smaller (0.002), Lateral: medium (0.003)
  - Expected impact: More efficient exploration per grasp type

### Phase 5: Performance Optimizations

- [ ] **Implement caching of TSDF queries**
  - Rationale: Repeated queries for same positions across samples
  - Implementation: Add LRU cache in `pointcloud_helper.rs` for TSDF distance queries
  - Cache size: 10,000 entries
  - Expected impact: 10-20% speedup in collision detection

- [ ] **Optimize collision sweep order**
  - Rationale: Check most likely collision points first for early termination
  - Implementation: Reorder sweep_points in `planner.rs:126-287` specs
  - Logic: Put fingertips first, then palm, then less critical points
  - Expected impact: Earlier collision detection, fewer TSDF queries

- [ ] **Add SIMD optimization for TSDF distance queries**
  - Rationale: Batch distance calculations can be vectorized
  - Implementation: Use packed SIMD operations for multiple position queries
  - Expected impact: 2-3x speedup in collision detection

## Verification Criteria

- [ ] **Performance**: Total evaluations reduced from 80,000 to < 15,000 on average
- [ ] **Quality**: Grasp success rate maintained or improved (> 90% of current)
- [ ] **Zero-Score Handling**: Best score > 0.0 in 95%+ of scenarios (even when no perfect grasp)
- [ ] **Convergence**: Early termination activates in 60%+ of cases
- [ ] **Runtime**: Total planning time reduced by > 50%
- [ ] **Robustness**: No regression in edge cases (single object, multiple objects, etc.)

## Potential Risks and Mitigations

1. **Risk: Reduced sample count misses good grasps**
   - Mitigation: Adaptive iteration control ensures sufficient exploration; can increase samples if success rate drops

2. **Risk: Early termination stops too early**
   - Mitigation: Minimum iteration guard (3) and conservative convergence threshold (0.01)

3. **Risk: Distance-based scoring for Tier 4 creates false positives**
   - Mitigation: Keep score range very low (0.0-0.05) so it only acts as tiebreaker

4. **Risk: Biased resampling reduces diversity**
   - Mitigation: Keep bias small (0.0005m) and maintain random component; monitor elite diversity

5. **Risk: Cache invalidation issues with TSDF**
   - Mitigation: Simple LRU with position-based keys; cache is per-planning call so no persistence issues

6. **Risk: SIMD optimization introduces bugs**
   - Mitigation: Extensive testing; fallback to scalar implementation if issues detected

## Alternative Approaches

1. **Cross-Entropy Method (CEM) instead of SMC**
   - Description: Fit Gaussian to elite distribution each iteration
   - Trade-offs: Simpler implementation, less diversity control, may converge faster but to local optima
   - Recommendation: Keep SMC for better diversity, consider CEM if SMC underperforms

2. **Bayesian Optimization with Gaussian Processes**
   - Description: Model score function as GP, use acquisition function for sampling
   - Trade-offs: Very sample-efficient, but high overhead per evaluation, complex implementation
   - Recommendation: Overkill for this problem; SMC is sufficient

3. **Hybrid: Coarse-to-fine sampling**
   - Description: Start with few samples at large variance, progressively increase samples
   - Trade-offs: Better initial coverage, more complex implementation
   - Recommendation: Good alternative if adaptive SMC doesn't suffice

4. **Gradient-based optimization from best samples**
   - Description: Use finite differences or automatic differentiation to optimize best grasp
   - Trade-offs: Very fast convergence, but requires differentiable scoring, may get stuck in local optima
   - Recommendation: Consider as post-processing step after SMC

## Implementation Priority

**High Priority (Immediate Impact):**
1. Reduce sample count to 2,500
2. Implement early termination
3. Add fallback scoring for zero-contact scenarios
4. Increase elite ratio to 0.2

**Medium Priority (Significant Improvement):**
5. Add distance-to-surface metric for Tier 4
6. Implement adaptive weight adjustment
7. Optimize collision sweep order
8. Add TSDF query caching

**Low Priority (Nice-to-have):**
9. Biased resampling toward gradients
10. Grasp-type-specific proposals
11. SIMD optimization

## Testing Strategy

1. **Benchmark current performance** on diverse scenarios (single object, multiple objects, different poses)
2. **Implement changes incrementally** and test after each phase
3. **Compare metrics**: success rate, planning time, score distribution, iteration count
4. **Edge case testing**: objects at boundaries, very small/large objects, hand starting far/close
5. **Regression testing**: ensure no degradation in existing successful scenarios

## Success Metrics

- **Primary**: 50%+ reduction in planning time with < 10% reduction in success rate
- **Secondary**: Zero-score scenarios reduced from current rate by 80%+
- **Tertiary**: Early termination activates in 60%+ of planning cycles
- **Long-term**: Framework for continuous optimization (adaptive parameters, learned heuristics)
