# Twist Propagation: Minimum Speed, Minimum Hit Duration, and Minimum Time-to-Hit

## Objective

Add three safety filters to the twist propagation hit detection pipeline:
1. **Minimum linear speed threshold** — suppress hit detection when the hand is stationary or barely moving
2. **Minimum hit persistence duration** — require a hit to be sustained across multiple consecutive cycles before triggering segmentation
3. **Minimum time-to-hit** — reject hits that occur too early in the propagation horizon, as these indicate the hand is already on top of the object (likely false positive or too late for the pipeline to act)

The goal is to minimize unintentional/false hits from tracking noise, hand drift, and momentary glancing collisions, while preserving responsive hit detection for genuine grasp approaches.

---

## Current System Analysis

### How Hits Are Currently Triggered

The hit detection flow in `_run_idle_cycle()` (`twist_propagation_node.py:1171-1338`):

1. Estimate twist from pose buffer (`twist_propagation_node.py:1202`)
2. Apply origin offset (camera → fingertips) (`twist_propagation_node.py:1211-1225`)
3. Transform to cloud frame (`twist_propagation_node.py:1228-1229`)
4. Propagate pose forward by twist, checking KDTree collision at each step (`twist_propagation_node.py:1240-1243`)
5. **Immediately** trigger segmentation on first hit — no speed check, no persistence requirement, no time-to-hit check

### How Time-to-Hit Is Computed

Inside `_propagate_and_find_hit()` (`twist_propagation_node.py:848-879`), the propagation loop tracks time `t`:

```
t = 0.0
while t < self._horizon:          # horizon = 2.0s
    t += self._dt                  # dt = 0.02s
    result = _propagate_pose(...)  # advance pose by dt
    ...
    # Check collision at this time step
    dists, _ = tree.query([px, py, pz], k=...)
    if hit detected:
        return hit_point           # <-- t is the time-to-hit
```

The variable `t` at the moment of collision is the **predicted time-to-hit** — how many seconds into the future the collision occurs. This value is currently **discarded** (only the hit point is returned).

### Downstream Pipeline Latency Budget

From real-world measurements:

| Stage | Time | Source |
|-------|------|--------|
| Segmentation inference | ~1.0s | `camera-test-log-v6.txt:112-113` (clicks at 37.985s, segmented cloud at 38.975s) |
| Preshaping call delay | 0.15s | `twist_propagation_node.py:474` |
| Grasp planning (Rust FFI) | 215-415ms | `camera-test-log-v6.txt` Pipeline Time measurements |
| **Total: hit → motor command** | **~1.4 - 1.6s** | Sum of above |

From the Tier B latency test (`tests/test1_software_verification/results/tier_b_latency_results.csv`):
- Service-only path (bypassing segmentation): **118-288ms** mean
- Full EMG path: not measured (would add EMG detection + pipeline manager latency)

**Key insight**: The downstream pipeline needs **~1.4-1.6 seconds** from hit detection to motor command. If a hit occurs at `t < 0.4s` in the propagation, the hand would physically reach the object in less than 0.4 seconds — far too late for the pipeline to compute and execute a grasp. Such a hit is either:
1. A false positive (the hand is already near the object, not approaching it)
2. A genuine approach that's too fast for the system to handle
3. Tracking noise causing a spurious near-field collision

In all three cases, triggering segmentation is wasteful or harmful.

### What's Currently Missing

- **No minimum speed gate**: A stationary hand near cloud geometry triggers immediately
- **No hit persistence/debounce**: A single-cycle hit triggers the full pipeline
- **No minimum time-to-hit**: A hit at `t = 0.02s` (first propagation step) is treated identically to a hit at `t = 1.5s`
- **Time-to-hit is computed but discarded**: The `t` variable in `_propagate_and_find_hit()` is never returned or used for filtering

---

## Implementation Plan

### Phase 1: Minimum Linear Speed Threshold

- [ ] **Task 1.1**: Add new ROS parameter `min_twist_linear_mps` (default `0.0`) to `TwistPropagationNode.__init__()` at `twist_propagation_node.py:404-509`. This parameter sets the minimum linear velocity magnitude (m/s) below which hit detection is suppressed. Default of `0.0` preserves backward compatibility.
- [ ] **Task 1.2**: Read and store the parameter alongside existing parameter reads at `twist_propagation_node.py:476-509`: `self._min_twist_linear = self.get_parameter("min_twist_linear_mps").value`
- [ ] **Task 1.3**: Add a speed gate check in `_run_idle_cycle()` immediately after twist estimation (`twist_propagation_node.py:1202`) and before propagation. Compute `lin_mag = math.sqrt(vx**2 + vy**2 + vz**2)`. If `lin_mag < self._min_twist_linear`, skip propagation and publish status with `reason="below_min_speed"`. This prevents the expensive KDTree propagation when the hand is essentially stationary.
- [ ] **Task 1.4**: Add `min_twist_linear_mps: 0.0` to `src/twist_propagation/config/twist_propagation.yaml` under the "Twist estimation" section (after `twist_estimation_window`).
- [ ] **Task 1.5**: Add `min_twist_linear_mps` to `config/prosthesis_config.yaml` in the `twist_propagation` section. Set initial runtime value to `0.02` m/s (2 cm/s) — slow enough to not miss real approaches, fast enough to reject drift.

**Rationale**: The speed gate is the simplest and most effective filter. It directly addresses the "stationary hand near geometry" false positive case. It's cheap to compute (one `sqrt`) and can be tuned independently.

### Phase 2: Minimum Hit Persistence Duration

- [ ] **Task 2.1**: Add new ROS parameter `min_hit_persist_s` (default `0.0`) to `TwistPropagationNode.__init__()`. This sets the minimum duration a hit must be continuously detected before triggering segmentation. Default `0.0` preserves backward compatibility (instant trigger).
- [ ] **Task 2.2**: Read and store the parameter: `self._min_hit_persist = self.get_parameter("min_hit_persist_s").value`
- [ ] **Task 2.3**: Add new state variables in the state initialization section (`twist_propagation_node.py:527-565`):
  - `self._pending_hit: tuple | None = None` — the hit point awaiting confirmation
  - `self._pending_hit_start: float = 0.0` — wall-clock time when the pending hit was first detected
- [ ] **Task 2.4**: Modify the hit detection logic in `_run_idle_cycle()` at `twist_propagation_node.py:1252-1329`. After a hit is found:
  - If `self._min_hit_persist <= 0.0`: trigger immediately (backward-compatible path)
  - If no pending hit exists: store the hit as pending, record start time, publish status `reason="hit_pending_confirmation"`
  - If a pending hit exists and the new hit is within `segmentation_retarget_distance_m` of the pending hit: check if `time.time() - self._pending_hit_start >= self._min_hit_persist`. If yes, trigger segmentation. If no, continue waiting.
  - If a pending hit exists but the new hit is far away: reset the pending hit to the new location and restart the timer (the target moved)
  - If no hit found this cycle and a pending hit exists: clear the pending hit (hit was transient)
- [ ] **Task 2.5**: Ensure pending hit state is cleared on deactivation and on segmentation timeout (in `_cycle_callback` at `twist_propagation_node.py:1139-1146`).
- [ ] **Task 2.6**: Add `min_hit_persist_s: 0.0` to `src/twist_propagation/config/twist_propagation.yaml`.
- [ ] **Task 2.7**: Add `min_hit_persist_s` to `config/prosthesis_config.yaml`. Suggested initial value: `0.1` to `0.2` seconds (1-2 cycles at 10Hz).

**Rationale**: The persistence requirement acts as a temporal debounce. It ensures that momentary noise spikes or single-frame glitches don't trigger the full pipeline.

### Phase 3: Minimum Time-to-Hit

- [ ] **Task 3.1**: Modify `_propagate_and_find_hit()` return signature to include the time-to-hit. Change from returning `tuple | None` (just the hit point) to returning `tuple[float, float, float, float] | None` (hit_x, hit_y, hit_z, time_to_hit_s). The variable `t` at the collision point (`twist_propagation_node.py:848-879`) is already computed — it just needs to be returned.
- [ ] **Task 3.2**: Add new ROS parameter `min_time_to_hit_s` (default `0.0`) to `TwistPropagationNode.__init__()`. This sets the minimum predicted time-to-hit below which hits are rejected. Default `0.0` preserves backward compatibility.
- [ ] **Task 3.3**: Read and store the parameter: `self._min_time_to_hit = self.get_parameter("min_time_to_hit_s").value`
- [ ] **Task 3.4**: In `_run_idle_cycle()`, after unpacking the hit result from `_propagate_and_find_hit()`, check the returned `time_to_hit_s`. If `time_to_hit_s < self._min_time_to_hit`, reject the hit and publish status with `reason="hit_too_close"`, `time_to_hit_s=round(time_to_hit_s, 3)`. Do NOT transition to WAITING_FOR_SEGMENTATION.
- [ ] **Task 3.5**: Include `time_to_hit_s` in the status JSON for all hits (both accepted and rejected) for diagnostics and tuning.
- [ ] **Task 3.6**: Add `min_time_to_hit_s: 0.0` to `src/twist_propagation/config/twist_propagation.yaml` under a new "Hit validation" section.
- [ ] **Task 3.7**: Add `min_time_to_hit_s` to `config/prosthesis_config.yaml`. Suggested initial value: `0.4` seconds — this means the hand must be at least 0.4s away from the object (at current velocity) for the hit to be accepted. Given the ~1.4s downstream pipeline latency, this still leaves ~1.0s for the pipeline to compute and execute the grasp.

**Rationale**: The time-to-hit filter is elegant because it encodes a physical constraint directly: "is there enough time for the pipeline to act on this prediction?" A hit at `t = 0.02s` means the hand is already essentially touching the object — the pipeline cannot possibly compute a grasp plan and execute motor commands in 20ms. Rejecting these hits prevents wasted computation and false triggers. The `t` variable is already computed in the propagation loop; it just needs to be returned.

### Phase 4: Unit Tests

- [ ] **Task 4.1**: Add test class `TestMinSpeedFilter` to `src/twist_propagation/test/test_twist_propagation.py`:
  - `test_zero_speed_suppresses_hit`: Verify that with `min_twist_linear_mps > 0`, a zero-velocity twist does not produce a hit
  - `test_below_threshold_suppresses_hit`: Verify that a velocity just below the threshold is suppressed
  - `test_above_threshold_allows_hit`: Verify that a velocity at/above the threshold proceeds normally
  - `test_default_zero_backward_compatible`: Verify that `min_twist_linear_mps = 0.0` does not filter anything
- [ ] **Task 4.2**: Add test class `TestHitPersistence` to the same test file:
  - `test_zero_persist_triggers_immediately`: With `min_hit_persist_s = 0.0`, a single hit triggers immediately
  - `test_hit_must_persist`: With `min_hit_persist_s > 0`, a hit on one cycle is not sufficient
  - `test_hit_persist_satisfied`: A hit sustained for the required duration triggers
  - `test_transient_hit_cleared`: A hit that disappears on the next cycle is cleared
  - `test_hit_moved_resets_timer`: A hit that moves beyond retarget distance resets the timer
- [ ] **Task 4.3**: Add test class `TestMinTimeToHit` to the same test file:
  - `test_hit_below_min_time_rejected`: A hit at `t = 0.02s` with `min_time_to_hit_s = 0.4` is rejected
  - `test_hit_above_min_time_accepted`: A hit at `t = 0.5s` with `min_time_to_hit_s = 0.4` is accepted
  - `test_hit_exactly_at_threshold_accepted`: A hit at exactly `t = 0.4s` with `min_time_to_hit_s = 0.4` is accepted (boundary)
  - `test_default_zero_backward_compatible`: With `min_time_to_hit_s = 0.0`, any time-to-hit is accepted
  - `test_time_to_hit_returned_correctly`: Verify the returned time-to-hit matches the propagation step count (e.g., 10 steps × 0.02s = 0.2s)

**Rationale**: These are pure logic tests that don't require ROS 2. They test the filtering logic in isolation, consistent with the existing test pattern.

### Phase 5: Integration Tests

- [ ] **Task 5.1**: Add integration test cases to `scripts/test_twist_propagation_integration.py`:
  - Test that a stationary hand near a cloud does NOT trigger segmentation when `min_twist_linear_mps > 0`
  - Test that a moving hand DOES trigger segmentation even with the speed threshold
  - Test that a brief single-cycle hit does NOT trigger segmentation when `min_hit_persist_s > 0`
  - Test that a sustained hit DOES trigger after the persistence duration
  - Test that a near-field hit (small time-to-hit) is rejected when `min_time_to_hit_s > 0`
  - Test that a far-field hit (large time-to-hit) is accepted

---

## Verification Criteria

- [ ] With all three parameters at their defaults (`0.0`), behavior is identical to the current system (backward compatibility)
- [ ] A stationary hand (twist magnitude < threshold) does not trigger segmentation, even when positioned near cloud geometry
- [ ] A hand moving above the minimum speed threshold triggers normally
- [ ] A single-cycle hit (transient) is rejected when `min_hit_persist_s > 0`
- [ ] A hit sustained for >= `min_hit_persist_s` triggers segmentation
- [ ] A hit with time-to-hit < `min_time_to_hit_s` is rejected with appropriate status
- [ ] A hit with time-to-hit >= `min_time_to_hit_s` is accepted
- [ ] The time-to-hit value is included in status JSON for diagnostics
- [ ] All existing unit tests continue to pass unchanged
- [ ] New parameters appear in both config YAML files with sensible defaults

---

## Potential Risks and Mitigations

### Risks Shared with Previous Plan

1. **Increased Latency for Real Grasps** (from persistence filter)
   **Mitigation**: Keep `min_hit_persist_s` low (0.1-0.2s). The propagation already predicts ~2s into the future, so the system has significant timing margin.

2. **Speed Threshold Rejecting Slow but Intentional Approaches**
   **Mitigation**: Set `min_twist_linear_mps` conservatively (0.02 m/s). Make it configurable.

3. **Hit Persistence vs. Moving Target**
   **Mitigation**: Uses existing `segmentation_retarget_distance_m` (0.10m) as spatial tolerance.

4. **Pending Hit State Across Deactivation/Reactivation**
   **Mitigation**: Explicitly clear `_pending_hit` and `_pending_hit_start` in deactivation handler.

5. **EMA Smoothing Interacting with Speed Gate**
   **Mitigation**: The EMA ramp-up (~0.25s) is beneficial — it naturally rejects noise spikes.

### Risks Specific to Minimum Time-to-Hit

6. **Rejecting Legitimate Fast Approaches**
   A user might approach an object quickly, causing a time-to-hit < 0.4s. The system would reject this even though it's a real grasp attempt.
   **Mitigation**: This is actually the *correct* behavior. If the hand reaches the object in < 0.4s, the pipeline cannot compute a grasp in time anyway (~1.4s downstream latency). The hit would be wasted computation that locks the state machine for 5+ seconds. The user would need to slow down or the system would need to be faster — the filter correctly communicates "too late to act on this."

7. **Interaction with Propagation Origin Offset**
   The `propagation_origin_offset` (`[0.1543, -0.1485, 0.1352]`) shifts the start point ~21cm forward to the fingertips. This means the fingertips might already be near the object even when the wrist/camera is far away. A time-to-hit check would correctly reject these "already there" cases.
   **Mitigation**: This is beneficial — it's exactly the scenario the filter is designed to catch.

8. **Time-to-Hit Varies with Speed**
   For the same spatial distance, a faster hand produces a shorter time-to-hit. The minimum time-to-hit filter effectively creates a **maximum approach distance** that scales with speed: `min_distance = speed × min_time_to_hit_s`. At 0.1 m/s and `min_time_to_hit_s = 0.4`, the minimum distance is 4cm. At 0.5 m/s, it's 20cm. This means faster approaches need to start further away.
   **Mitigation**: This is physically correct — faster approaches need more lead time. The 2.0s propagation horizon provides ample range.

9. **Return Signature Change in `_propagate_and_find_hit`**
   Changing the return type from `tuple | None` to `tuple[float, float, float, float] | None` is a breaking change for all callers. Currently there's one caller (`_run_idle_cycle`), so the blast radius is small.
   **Mitigation**: Update the single caller at `twist_propagation_node.py:1240-1243` to unpack four values. Add a clear docstring to the function. The existing unit tests for `_propagate_pose` (the pure function) are unaffected.

10. **Time-to-Hit with Zero Velocity**
    If the speed gate (Phase 1) is active, zero-velocity cases never reach the time-to-hit check. If the speed gate is disabled (`min_twist_linear_mps = 0.0`), zero velocity means the hand never moves — the first propagation step (`t = 0.02s`) produces a hit if the hand is near geometry, and `time_to_hit_s = 0.02` would be below any reasonable `min_time_to_hit_s`.
    **Mitigation**: The time-to-hit filter provides a safety net even when the speed gate is disabled. The two filters are complementary.

11. **Covariance Truncation Affecting Time-to-Hit**
    If covariance propagation truncates the horizon early (at `twist_propagation_node.py:862-868`), hits that would have occurred later in the horizon are never found. This means the time-to-hit filter only sees hits that occur before covariance truncation — it doesn't affect the filter's correctness.
    **Mitigation**: No interaction issue. Covariance truncation reduces the search space; the time-to-hit filter operates within whatever hits are found.

---

## Alternative Approaches

### Alternative A: Speed Gate Only (No Persistence, No Time-to-Hit)
Simplest option. Only the minimum speed threshold.
**Trade-offs**: Doesn't address single-cycle noise spikes or near-field false positives. Least protection.

### Alternative B: Time-to-Hit Only (No Speed Gate, No Persistence)
Only the minimum time-to-hit filter.
**Trade-offs**: Rejects near-field hits but doesn't address stationary hands or transient glitches. A stationary hand near geometry with slight tracking jitter could produce varying time-to-hit values that pass the filter.

### Alternative C: Combined Speed + Time-to-Hit (No Persistence)
Skip the persistence requirement; rely on speed and time-to-hit.
**Trade-offs**: A single noisy frame with high velocity and a lucky collision at t > 0.4s could still trigger. Persistence catches these.

### Alternative D: Adaptive Time-to-Hit Based on Pipeline Latency
Instead of a fixed `min_time_to_hit_s`, dynamically compute it from the measured downstream latency. If segmentation took 1.2s last time, set `min_time_to_hit = 1.2s + safety_margin`.
**Trade-offs**: More complex, requires latency feedback from downstream nodes. Could be a future enhancement once the basic filter is validated.

### Alternative E: Time-to-Hit as a Confidence Weight (Soft Filter)
Instead of a hard reject/accept, weight the hit confidence by the time-to-hit. Hits with short time-to-hit are published with lower confidence, and downstream nodes decide whether to act.
**Trade-offs**: Requires changes to the segmentation and preshaping interfaces. More complex but more flexible. Overkill for current needs.

---

## How the Three Filters Interact

The three filters form a layered defense:

```
                    Raw Pose Stream
                         │
                         ▼
              ┌─────────────────────┐
              │  Phase 1: Speed Gate │  ← Is the hand moving at all?
              │  (min_twist_linear)  │
              └────────┬────────────┘
                       │ (moving)
                       ▼
              ┌─────────────────────┐
              │  Phase 3: Time-to-  │  ← Is there enough time to act?
              │  Hit (min_t)        │
              └────────┬────────────┘
                       │ (enough time)
                       ▼
              ┌─────────────────────┐
              │  Phase 2: Persist-  │  ← Is the hit stable?
              │  ence (min_dur)     │
              └────────┬────────────┘
                       │ (confirmed)
                       ▼
                Trigger Segmentation
```

**Order matters**: The speed gate is cheapest (one `sqrt`) and rejects the most common false positives, so it goes first. The time-to-hit check is next because it uses data already computed during propagation. The persistence check is most expensive (requires multi-cycle state) and goes last.

**Independence**: Each filter can be independently enabled/disabled via its parameter (default `0.0` = disabled). This allows incremental deployment and tuning.

---

## Recommended Parameter Values

| Parameter | Default (YAML) | Runtime Initial | Rationale |
|-----------|---------------|-----------------|-----------|
| `min_twist_linear_mps` | `0.0` | `0.02` | 2 cm/s rejects drift but allows slow intentional approaches |
| `min_hit_persist_s` | `0.0` | `0.15` | 1.5 cycles at 10Hz; rejects single-cycle glitches |
| `min_time_to_hit_s` | `0.0` | `0.4` | Hand must be ≥0.4s from object; leaves ~1.0s for pipeline |

The time-to-hit value of 0.4s is derived from:
- Downstream pipeline: ~1.0s (segmentation) + ~0.15s (delay) + ~0.3s (planning) = ~1.45s total
- The propagation predicts up to 2.0s ahead
- A hit at t=0.4s means the hand reaches the object in 0.4s
- The pipeline would need to complete in 0.4s, but it takes ~1.45s — impossible
- Setting min_time_to_hit = 0.4s ensures at least some margin, while not being so high that it rejects valid medium-range approaches
- This could be increased to 0.6-0.8s for more conservative filtering

These values should be validated with live testing and adjusted based on observed false positive/negative rates.
