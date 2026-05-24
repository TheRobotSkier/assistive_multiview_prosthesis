# Twist Propagation Latency Optimization

## Objective

Reduce the twist propagation stage latency from **~160 ms mean / ~176 ms P95** to a target that, combined with the other pipeline stages (~75 ms preshaping + ~5 ms overhead), brings the total pipeline below **100 ms P95** (IDE requirement) or as close to it as possible.

The two optimization strategies:
1. **Background pre-computation** — run twist estimation and KDTree construction continuously while inactive, so they're ready on activation
2. **Higher polling rate** — reduce the timer period from 100 ms to a faster default

---

## Current Latency Breakdown (from benchmark)

| Stage | Mean (ms) | P95 (ms) | % of Total |
|-------|-----------|----------|------------|
| Pipeline Manager | 1.0 | 1.3 | 0.4% |
| **Twist Propagation** | **159.9** | **175.9** | **66.7%** |
| Segmentation (simulated) | 2.5 | 2.8 | 1.0% |
| PM Cloud Handling | 0.6 | 1.0 | 0.3% |
| Grasp Preshaping (Rust) | 74.1 | ~83 | 30.9% |
| Preshaping (ROS overhead) | 1.8 | ~2.6 | 0.8% |
| **TOTAL** | **239.7** | **257.5** | **100%** |

### Twist Propagation Latency Decomposition (estimated)

| Component | Time (ms) | Source |
|-----------|-----------|--------|
| Pose buffer accumulation (wait for ≥2 poses at 50 Hz) | ~40 | `_on_activate` clears buffer, then test publishes at 50 Hz |
| Polling wait (avg 0.5 × cycle_delay_s) | ~50 | 10 Hz timer at `twist_propagation_node.py:641` |
| KDTree build (first cycle after activation) | ~10-30 | `KDTree(xyz)` at `twist_propagation_node.py:815` |
| Propagation + KDTree queries (100 steps × ~5 µs) | ~0.5 | `_propagate_and_find_hit` loop at line 855 |
| Twist estimation (finite diff + least-squares) | ~0.1 | `_estimate_twist` at line 729 |
| DDS transport + service call overhead | ~5-10 | ROS middleware |
| **Total** | **~106-131** | |

---

## Implementation Plan

### Part 1: Background Pre-computation (Warm Cache)

**Goal**: Eliminate the ~40 ms pose accumulation wait and ~10-30 ms first-cycle KDTree build by running these continuously while inactive.

- [ ] **Task 1.1**: Add a new `_run_background_cycle()` method to `TwistPropagationNode` that runs a read-only subset of `_run_idle_cycle` when inactive.

  The method should:
  - Estimate twist from the pose buffer (writes `self._twist` — safe, will be overwritten on next cycle)
  - Build/maintain the KDTree from the latest cloud (calls `_get_kdtree()` — lazy build, safe)
  - **NOT** publish clicks, segmentation resets, or transition state machine
  - **NOT** publish visualization markers (path, spheres, hit marker)
  - Optionally publish twist and current pose for debugging (gated by a parameter)
  
  Rationale: When the node is activated, it will have a fresh twist estimate and a cached KDTree, eliminating the cold-start penalty.

- [ ] **Task 1.2**: Modify `_cycle_callback` at `twist_propagation_node.py:1123-1177` to call `_run_background_cycle()` instead of just publishing status when inactive.

  Current code (line 1133-1135):
  ```python
  if not active:
      self._publish_status()
      return
  ```
  
  Change to:
  ```python
  if not active:
      self._run_background_cycle()
      self._publish_status()
      return
  ```

- [ ] **Task 1.3**: Modify `_on_activate` at `twist_propagation_node.py:654-663` to **preserve the KDTree cache** and optionally the twist estimate.

  Current code clears the pose buffer but not the KDTree. The KDTree is already preserved (only `_on_input_cloud` invalidates it). However, the twist is zeroed by `_on_deactivate` but NOT by `_on_activate` — this is already correct: if background cycling maintained a fresh twist, activation will inherit it.
  
  **Key insight**: The pose buffer clear on activation is intentional — stale poses from the inactive period would produce incorrect velocity estimates. But since background cycling keeps `_twist` up-to-date, the first active cycle can use the pre-computed twist immediately even with only 1-2 fresh poses in the buffer.
  
  Modify `_on_activate` to set a flag `_just_activated = True` so the first active cycle knows to use the pre-computed twist rather than requiring ≥2 poses in the buffer.

- [ ] **Task 1.4**: Modify `_run_idle_cycle` at `twist_propagation_node.py:1179` to handle the `_just_activated` fast path.

  When `_just_activated = True`:
  - Skip the `len(self._pose_buf) < 2` check — use the pre-computed `_twist` directly
  - Use the latest single pose from the buffer (or the last background-cycle pose if buffer is empty)
  - Clear `_just_activated = False` after first cycle
  
  This eliminates the ~40 ms pose accumulation wait entirely.

- [ ] **Task 1.5**: Add a new parameter `background_cycle_enabled` (default: `True`) to gate the background cycle behavior. When `False`, the node behaves exactly as before (just publishes status when inactive). This preserves backward compatibility and allows A/B testing.

  Add to parameter declarations at `twist_propagation_node.py:426`:
  ```python
  self.declare_parameter("background_cycle_enabled", True)
  ```
  
  Add to config at `config/prosthesis_config.yaml:210`:
  ```yaml
  background_cycle_enabled: true
  ```

### Part 2: Higher Polling Rate

**Goal**: Reduce the average polling wait from ~50 ms (at 10 Hz) to a lower value.

- [ ] **Task 2.1**: Change the default `cycle_delay_s` from `0.1` (10 Hz) to `0.02` (50 Hz).

  Current: `config/prosthesis_config.yaml:196` — `cycle_delay_s: 0.1`
  New: `cycle_delay_s: 0.02`
  
  Also update the code default at `twist_propagation_node.py:409`:
  ```python
  self.declare_parameter("cycle_delay_s", 0.02)
  ```
  
  Rationale: The propagation computation itself takes <1 ms per cycle. Running at 50 Hz means the timer fires every 20 ms, so the average polling wait drops from ~50 ms to ~10 ms. The CPU cost is negligible — 50 cycles/sec × 1 ms/cycle = 5% CPU, and that's only when actively propagating.

- [ ] **Task 2.2**: Verify that the 50 Hz cycle rate does not cause issues with the preshaping service call timing.

  The preshaping call is gated by `_schedule_preshaping_call` which uses a one-shot timer at `preshaping_call_delay_s = 0.15` (line 1405). This is independent of the cycle timer, so increasing the cycle rate does not affect it. The WAITING_FOR_SEGMENTATION and WAITING_FOR_PRESHAPING states just check for new segmented clouds or wait — they don't re-run propagation. No issues expected.

### Part 3: KDTree Background Rebuild

**Goal**: Ensure the KDTree is always fresh without adding latency to the critical path.

- [ ] **Task 3.1**: The current lazy-rebuild pattern in `_get_kdtree()` (line 808-819) is already optimal for the background cycle approach. When `_run_background_cycle()` calls `_get_kdtree()` while inactive, it rebuilds the tree whenever a new cloud arrives. When the node is activated, the KDTree is already cached and fresh.
  
  **No code change needed** — this is automatically handled by Task 1.1 (background cycle calls `_get_kdtree()`).

### Part 4: Update Latency Benchmark

- [ ] **Task 4.1**: Update `run_latency_stages.py` to support A/B testing with the `background_cycle_enabled` parameter. Add a `--no-background-cycle` flag that sets the parameter to `False` via ROS2 CLI args before running the benchmark. This allows measuring the before/after improvement.

- [ ] **Task 4.2**: Update `run_latency_stages.py` to also test with the old `cycle_delay_s = 0.1` for comparison. Add a `--cycle-delay` argument (default: `0.02`).

### Part 5: Verification

- [ ] **Task 5.1**: Run the latency benchmark with background cycling enabled + 50 Hz cycle rate (optimized configuration). Expect:
  - Twist propagation: ~20-30 ms (down from ~160 ms)
  - Total pipeline: ~100-110 ms (down from ~240 ms)

- [ ] **Task 5.2**: Run the latency benchmark with background cycling disabled + 10 Hz cycle rate (baseline configuration). Verify results match the original ~160 ms / ~240 ms numbers.

- [ ] **Task 5.3**: Compare results and document the improvement in `results/latency_optimization_comparison.csv`.

- [ ] **Task 5.4**: Verify that the existing mock launch integration test still passes with the new default parameters.

---

## Expected Latency Improvement

| Configuration | Twist Prop (ms) | Total (ms) | vs MAR 400ms | vs IDE 100ms |
|---------------|-----------------|------------|---------------|--------------|
| Baseline (10 Hz, no bg) | ~160 | ~240 | PASS | FAIL |
| + Background cycle only | ~110 | ~190 | PASS | FAIL |
| + 50 Hz cycle only | ~120 | ~200 | PASS | FAIL |
| **Both optimizations** | **~20-30** | **~100-110** | **PASS** | **BORDERLINE** |

The background cycle eliminates pose accumulation wait (~40 ms) and KDTree build (~10-30 ms). The 50 Hz cycle reduces polling wait from ~50 ms to ~10 ms. Combined, the twist propagation stage should drop from ~160 ms to ~20-30 ms.

The IDE target of ≤100 ms is borderline because the Rust grasp planner alone takes ~74 ms. Even with zero twist propagation latency, the total would be ~80 ms (74 Rust + 5 overhead + 1 PM). Meeting IDE requires either also optimizing the Rust planner or accepting that IDE is aspirational.

---

## Verification Criteria

- [ ] Background cycle runs correctly when node is inactive (twist + KDTree maintained)
- [ ] Activation preserves cached KDTree and uses pre-computed twist
- [ ] Latency benchmark shows ≥70% reduction in twist propagation stage
- [ ] Total pipeline P95 ≤ 400 ms (MAR) — must still PASS
- [ ] No regression in existing tests (mock launch, Tier B service test)
- [ ] `background_cycle_enabled: false` produces identical results to original behavior

## Potential Risks and Mitigations

1. **Background cycle consumes CPU when inactive**
   Mitigation: The propagation computation is <1 ms per cycle. At 50 Hz, that's <5% CPU. The KDTree build only happens when a new cloud arrives (15 Hz), and each build is ~10-30 ms — so ~0.3-0.5 ms average. Total background CPU: <6%. Negligible.

2. **Pre-computed twist becomes stale between deactivation and activation**
   Mitigation: The background cycle continuously updates `_twist` from the latest poses. If the hand is moving, the twist will be fresh. If the hand stops, the speed gate (`min_twist_linear_mps: 0.02`) will prevent false hits anyway. The `_just_activated` flag ensures the first active cycle uses the pre-computed twist, and subsequent cycles re-estimate from fresh poses.

3. **50 Hz cycle rate increases DDS traffic (status, twist, pose publications)**
   Mitigation: These are small messages (<1 KB each). At 50 Hz, that's ~50 KB/s — negligible. If needed, the status publication can be throttled (publish every Nth cycle).

4. **Background cycle publishes twist/pose that downstream nodes misinterpret**
   Mitigation: The twist and pose topics are informational. Downstream nodes (pipeline manager, segmentation) react to clicks and state transitions, not to twist/pose. The background cycle explicitly skips click and state-transition publishing.

## Alternative Approaches

1. **Event-driven collision detection** (instead of timer-based): Trigger a collision check on every new pose arrival instead of waiting for the timer. This would eliminate polling wait entirely (0 ms instead of ~10 ms at 50 Hz). More complex to implement (callback-based state machine), but could save an additional ~10 ms. Can be deferred to a future optimization if the 50 Hz + background approach is insufficient.

2. **Separate KDTree rebuild thread**: Build the KDTree in a background thread whenever a new cloud arrives, making it always available without blocking the cycle. This would save the ~10-30 ms first-cycle KDTree build even without the background cycle approach. However, the background cycle approach already achieves this with simpler code.

3. **Reduce `preshaping_call_delay_s` from 150 ms to 0 ms**: The 150 ms delay was added to prevent a race condition where the preshaping bridge hasn't received the segmented cloud yet. If the segmentation bridge confirms cloud receipt via a callback, this delay could be eliminated. This would save 150 ms but requires changes to the segmentation bridge. Separate optimization — not included in this plan.
