# Twist Propagation: Minimum Speed and Minimum Time-to-Hit

## Objective

Add two safety filters to the twist propagation hit detection pipeline:
1. **Minimum linear speed threshold** — suppress hit detection when the hand is stationary or barely moving
2. **Minimum time-to-hit** — reject hits that occur too early in the propagation horizon, as these indicate the hand is already on top of the object (likely false positive or too late for the pipeline to act)

The goal is to minimize unintentional/false hits from tracking noise, hand drift, and near-field false collisions, while preserving responsive hit detection for genuine grasp approaches.

**Phase 2 (minimum hit persistence) from the previous plan has been dropped** — see the Rationale section below.

---

## Why Hit Persistence Was Dropped

### The Problem: Hit Points Jump Dramatically Between Cycles

Analysis of `camera-test-log-v6.txt` shows that consecutive hit points from the same approach vary wildly in position. Some representative examples from the log:

| Cycle | Hit Point (x, y, z) | Time Gap |
|-------|---------------------|----------|
| 1 | `(-0.251, -0.318, 0.145)` | — |
| 2 | `(-0.177, -0.231, 0.047)` | 7.5s later |
| 3 | `(-0.318, -0.272, 0.158)` | 1.2s later |

Even consecutive hits during what appears to be the same approach show large spatial jumps:

| Cycle | Hit Point (x, y, z) | Delta from Previous |
|-------|---------------------|---------------------|
| `t=150.0` | `(0.381, -0.458, 0.134)` | — |
| `t=151.2` | `(0.282, -0.519, 0.089)` | ~14cm |
| `t=155.0` | `(-0.176, -0.358, 0.143)` | ~57cm |
| `t=157.4` | `(0.123, -0.498, 0.096)` | ~33cm |

### Root Causes of Hit Jitter

1. **Cloud replacement every cycle**: The fused point cloud is fully replaced on each callback (`twist_propagation_node.py:704-708`). The KDTree is rebuilt from scratch. Different cloud frames have different point distributions, so the "nearest surface point" to the same predicted position can jump.

2. **Voxel downsampling non-determinism**: `_voxel_downsample()` at `twist_propagation_node.py:116-128` keeps the **first** point per voxel. Different input orderings (from varying camera contributions) produce different downsampled clouds, shifting surface points.

3. **Tracking jitter**: OpenVINS pose estimates jitter between frames. The EMA smoothing (alpha=0.4) dampens but doesn't eliminate this. A 1cm pose jitter at the camera translates to ~1.5cm jitter at the fingertips (due to the 21cm offset lever arm).

4. **TF transform variability**: The TF lookup at `twist_propagation_node.py:900-903` can produce slightly different transforms across cycles, especially when the TF tree has connectivity issues (as documented in the logs: "Cannot transform to palm_frame," "Lookup would require extrapolation into the past").

5. **Propagation amplifies jitter**: Small changes in twist estimate produce different propagation paths. A slightly different path hits a different part of the cloud surface, returning a different nearest point.

### Why Persistence Doesn't Work Here

A persistence filter requires comparing hit points across cycles. But with the above jitter sources:

- **Spatial matching is unreliable**: The `segmentation_retarget_distance_m` (0.10m) tolerance is too small — real hits from the same object routinely jump 10-30cm between cycles. Increasing the tolerance to, say, 0.30m would make the filter meaningless (it would match almost anything).

- **Temporal consistency is coincidental**: Whether a hit persists across N cycles depends more on cloud/tracking stability than on whether the hit is genuine. A false positive from tracking drift could persist if the drift is consistent.

- **The filter would reject real hits**: A genuine approach toward an object could produce hits at different surface points as the hand moves and the predicted path shifts. The persistence filter would see these as "different hits" and reset the timer.

**Conclusion**: Hit persistence is a good idea in principle but doesn't fit this system's characteristics. The speed gate and time-to-hit filter achieve similar false-positive reduction without depending on spatial hit consistency.

---

## Current System Analysis

### How Hits Are Currently Triggered

The hit detection flow in `_run_idle_cycle()` (`twist_propagation_node.py:1171-1338`):

1. Estimate twist from pose buffer (`twist_propagation_node.py:1202`)
2. Apply origin offset (camera → fingertips) (`twist_propagation_node.py:1211-1225`)
3. Transform to cloud frame (`twist_propagation_node.py:1228-1229`)
4. Propagate pose forward by twist, checking KDTree collision at each step (`twist_propagation_node.py:1240-1243`)
5. **Immediately** trigger segmentation on first hit — no speed check, no time-to-hit check

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
        return hit_point           # <-- t is the time-to-hit (currently discarded)
```

The variable `t` at the moment of collision is the **predicted time-to-hit** — how many seconds into the future the collision occurs. This value is currently **discarded** (only the hit point is returned).

### Downstream Pipeline Latency Budget

From real-world measurements:

| Stage | Time | Source |
|-------|------|--------|
| Segmentation inference | ~1.0s | `camera-test-log-v6.txt:112-113` (clicks at 37.985s, segmented cloud at 38.975s) |
| Preshaping call delay | 0.15s | `twist_propagation_node.py:474` |
| Grasp planning (Rust FFI) | 215-415ms | `camera-test-log-v6.txt` (24 Pipeline Time measurements) |
| **Total: hit → motor command** | **~1.4 - 1.6s** | Sum of above |

From the Tier B latency test (`tests/test1_software_verification/results/tier_b_latency_results.csv`):
- Service-only path (bypassing segmentation): **118-288ms**
- Grasp planning pipeline time (Tier A): **111-415ms** across 1000 measurements

**Key insight**: The downstream pipeline needs **~1.4-1.6 seconds** from hit detection to motor command. If a hit occurs at `t < 0.4s` in the propagation, the hand would physically reach the object in less than 0.4 seconds — far too late for the pipeline to compute and execute a grasp.

---

## Implementation Plan

### Phase 1: Minimum Linear Speed Threshold

- [ ] **Task 1.1**: Add new ROS parameter `min_twist_linear_mps` (default `0.0`) to `TwistPropagationNode.__init__()` at `twist_propagation_node.py:404-509`. This parameter sets the minimum linear velocity magnitude (m/s) below which hit detection is suppressed. Default of `0.0` preserves backward compatibility.
- [ ] **Task 1.2**: Read and store the parameter alongside existing parameter reads at `twist_propagation_node.py:476-509`: `self._min_twist_linear = self.get_parameter("min_twist_linear_mps").value`
- [ ] **Task 1.3**: Add a speed gate check in `_run_idle_cycle()` immediately after twist estimation (`twist_propagation_node.py:1202`) and before propagation. Compute `lin_mag = math.sqrt(vx**2 + vy**2 + vz**2)`. If `lin_mag < self._min_twist_linear`, skip propagation and publish status with `reason="below_min_speed"`, `twist_linear_mag=round(lin_mag, 4)`. This prevents the expensive KDTree propagation when the hand is essentially stationary.
- [ ] **Task 1.4**: Add `min_twist_linear_mps: 0.0` to `src/twist_propagation/config/twist_propagation.yaml` under the "Twist estimation" section (after `twist_estimation_window`).
- [ ] **Task 1.5**: Add `min_twist_linear_mps` to `config/prosthesis_config.yaml` in the `twist_propagation` section. Set initial runtime value to `0.02` m/s (2 cm/s) — slow enough to not miss real approaches, fast enough to reject drift.

**Rationale**: The speed gate is the simplest and most effective filter. It directly addresses the "stationary hand near geometry" false positive case. It's cheap to compute (one `sqrt`) and can be tuned independently. It does not depend on cross-cycle hit consistency.

### Phase 2: Minimum Time-to-Hit

- [ ] **Task 2.1**: Modify `_propagate_and_find_hit()` return signature to include the time-to-hit. Change from returning `tuple | None` (just the hit point) to returning `tuple[float, float, float, float] | None` (hit_x, hit_y, hit_z, time_to_hit_s). The variable `t` at the collision point (`twist_propagation_node.py:848-879`) is already computed — it just needs to be included in the return value. At `twist_propagation_node.py:879`, change `return tuple(self._cloud_xyz[idx].tolist())` to `return (float(self._cloud_xyz[idx][0]), float(self._cloud_xyz[idx][1]), float(self._cloud_xyz[idx][2]), t)`.
- [ ] **Task 2.2**: Add new ROS parameter `min_time_to_hit_s` (default `0.0`) to `TwistPropagationNode.__init__()`. This sets the minimum predicted time-to-hit below which hits are rejected. Default `0.0` preserves backward compatibility.
- [ ] **Task 2.3**: Read and store the parameter: `self._min_time_to_hit = self.get_parameter("min_time_to_hit_s").value`
- [ ] **Task 2.4**: In `_run_idle_cycle()`, update the hit result unpacking at `twist_propagation_node.py:1240-1243` to handle the new 4-element return: `hit_result = self._propagate_and_find_hit(...)` then unpack as `hit = hit_result[:3]` and `time_to_hit = hit_result[3]` if not None.
- [ ] **Task 2.5**: After unpacking, check the time-to-hit. If `time_to_hit < self._min_time_to_hit`, reject the hit and publish status with `reason="hit_too_close"`, `time_to_hit_s=round(time_to_hit, 3)`. Do NOT transition to WAITING_FOR_SEGMENTATION. Skip all click publishing and state transitions.
- [ ] **Task 2.6**: Include `time_to_hit_s` in the status JSON for all hits (both accepted and rejected) for diagnostics and tuning. Update the accepted-hit status at `twist_propagation_node.py:1321-1329` and the no-hit status at `twist_propagation_node.py:1331-1338`.
- [ ] **Task 2.7**: Add `min_time_to_hit_s: 0.0` to `src/twist_propagation/config/twist_propagation.yaml` under a new "Hit validation" section.
- [ ] **Task 2.8**: Add `min_time_to_hit_s` to `config/prosthesis_config.yaml`. Suggested initial value: `0.4` seconds.

**Rationale**: The time-to-hit filter is elegant because it encodes a physical constraint directly: "is there enough time for the pipeline to act on this prediction?" A hit at `t = 0.02s` means the hand is already essentially touching the object — the pipeline cannot possibly compute a grasp plan and execute motor commands in 20ms. The `t` variable is already computed in the propagation loop; it just needs to be returned. This filter does not depend on cross-cycle hit consistency.

### Phase 3: Unit Tests

- [ ] **Task 3.1**: Add test class `TestMinSpeedFilter` to `src/twist_propagation/test/test_twist_propagation.py`:
  - `test_zero_speed_suppresses_hit`: Verify that with `min_twist_linear_mps > 0`, a zero-velocity twist does not produce a hit
  - `test_below_threshold_suppresses_hit`: Verify that a velocity just below the threshold is suppressed
  - `test_above_threshold_allows_hit`: Verify that a velocity at/above the threshold proceeds normally
  - `test_default_zero_backward_compatible`: Verify that `min_twist_linear_mps = 0.0` does not filter anything
- [ ] **Task 3.2**: Add test class `TestMinTimeToHit` to the same test file:
  - `test_hit_below_min_time_rejected`: A hit at `t = 0.02s` with `min_time_to_hit_s = 0.4` is rejected
  - `test_hit_above_min_time_accepted`: A hit at `t = 0.5s` with `min_time_to_hit_s = 0.4` is accepted
  - `test_hit_exactly_at_threshold_accepted`: A hit at exactly `t = 0.4s` with `min_time_to_hit_s = 0.4` is accepted (boundary)
  - `test_default_zero_backward_compatible`: With `min_time_to_hit_s = 0.0`, any time-to-hit is accepted
  - `test_time_to_hit_returned_correctly`: Verify the returned time-to-hit matches the propagation step count (e.g., 10 steps x 0.02s = 0.2s)
- [ ] **Task 3.3**: Update existing `TestProximity` tests at `test_twist_propagation.py:160-204` to handle the new 4-element return from `_propagate_and_find_hit` if any test directly calls that function (currently they test the KDTree query logic directly, so may not need changes — verify).

### Phase 4: Integration Tests

- [ ] **Task 4.1**: Add integration test cases to `scripts/test_twist_propagation_integration.py`:
  - Test that a stationary hand near a cloud does NOT trigger segmentation when `min_twist_linear_mps > 0`
  - Test that a moving hand DOES trigger segmentation even with the speed threshold
  - Test that a near-field hit (small time-to-hit) is rejected when `min_time_to_hit_s > 0`
  - Test that a far-field hit (large time-to-hit) is accepted
  - Test that status messages include `time_to_hit_s` in the JSON payload

---

## Verification Criteria

- [ ] With both parameters at their defaults (`0.0`), behavior is identical to the current system (backward compatibility)
- [ ] A stationary hand (twist magnitude < threshold) does not trigger segmentation, even when positioned near cloud geometry
- [ ] A hand moving above the minimum speed threshold triggers normally
- [ ] A hit with time-to-hit < `min_time_to_hit_s` is rejected with appropriate status message
- [ ] A hit with time-to-hit >= `min_time_to_hit_s` is accepted
- [ ] The time-to-hit value is included in status JSON for diagnostics
- [ ] All existing unit tests continue to pass unchanged
- [ ] New parameters appear in both config YAML files with sensible defaults

---

## Potential Risks and Mitigations

### 1. Speed Threshold Rejecting Slow Intentional Approaches
Some users may approach objects very slowly (< 2 cm/s), especially during rehabilitation.
**Mitigation**: Set `min_twist_linear_mps` conservatively (0.02 m/s). Make it fully configurable so it can be tuned per-user or disabled entirely (`0.0`).

### 2. EMA Smoothing Delays Speed Gate Opening
The EMA (alpha=0.4) means the twist estimate lags behind actual velocity. When the hand starts moving, estimated speed ramps up gradually (~0.25s time constant).
**Mitigation**: This is actually beneficial — it adds a natural ramp-up that rejects sudden noise spikes without explicit persistence logic.

### 3. Time-to-Hit Rejecting Legitimate Fast Approaches
A user approaching at 0.5 m/s with an object 15cm away would have a time-to-hit of 0.3s, which gets rejected by a 0.4s threshold.
**Mitigation**: This is correct behavior — the pipeline takes ~1.4s downstream and cannot act in 0.3s. Triggering would waste 5+ seconds of state machine lockout. The user needs to approach more slowly or from further away.

### 4. Interaction with Propagation Origin Offset
The offset `[0.1543, -0.1485, 0.1352]` shifts the start point ~21cm to the fingertips. The fingertips might be near the object even when the camera is far away.
**Mitigation**: The time-to-hit filter correctly catches these "already there" scenarios — this is exactly what it's designed for.

### 5. Time-to-Hit Scales with Speed (Implicit Minimum Distance)
For the same spatial distance, a faster hand produces a shorter time-to-hit. The filter effectively creates a minimum approach distance: `min_distance = speed × min_time_to_hit_s`.
**Mitigation**: This is physically correct — faster approaches need more lead time. The 2.0s propagation horizon provides ample range.

### 6. Return Signature Change in `_propagate_and_find_hit`
Changing from `tuple | None` to `tuple[float, float, float, float] | None` is a breaking change.
**Mitigation**: There is exactly one caller at `twist_propagation_node.py:1240-1243`. Update that single call site. Existing unit tests for `_propagate_pose` (the pure function) are unaffected.

### 7. Zero Velocity + Near Geometry = t = 0.02s
With zero twist, the first propagation step stays at the same position. If near geometry, a hit fires at `t = 0.02s`.
**Mitigation**: The speed gate (Phase 1) catches zero-velocity cases before propagation. The time-to-hit filter provides a second safety net if the speed gate is disabled.

### 8. Covariance Truncation Interaction
Covariance truncation can cut the horizon short, so some far-field hits are never found.
**Mitigation**: No interaction issue. The time-to-hit filter operates within whatever hits are found before truncation.

---

## How the Two Filters Interact

```
                    Raw Pose Stream
                         │
                         ▼
              ┌─────────────────────┐
              │  Phase 1: Speed Gate │  ← Is the hand moving at all?
              │  (min_twist_linear)  │     Cheapest check (one sqrt)
              └────────┬────────────┘
                       │ (moving)
                       ▼
              ┌─────────────────────┐
              │  Phase 2: Time-to-  │  ← Is there enough time to act?
              │  Hit (min_t)        │     Uses data already computed
              └────────┬────────────┘
                       │ (enough time)
                       ▼
                Trigger Segmentation
```

Each filter catches different false positive modes:

| False Positive Mode | Caught By |
|---------------------|-----------|
| Stationary hand near table surface | Speed gate |
| Hand already on top of / very close to object | Time-to-hit |
| Tracking drift producing slow movement | Speed gate |
| Fast approach that pipeline can't handle | Time-to-hit |
| Noise-induced near-field collision | Both (speed gate filters low-velocity noise; time-to-hit filters near-field hits) |

**Independence**: Each filter can be independently enabled/disabled via its parameter (default `0.0` = disabled). This allows incremental deployment and tuning.

**No cross-cycle state**: Unlike the dropped persistence filter, neither of these filters requires comparing hits across cycles. This makes them robust against the hit point jitter documented above.

---

## Recommended Parameter Values

| Parameter | Default (YAML) | Runtime Initial | Rationale |
|-----------|---------------|-----------------|-----------|
| `min_twist_linear_mps` | `0.0` | `0.02` | 2 cm/s rejects drift but allows slow intentional approaches |
| `min_time_to_hit_s` | `0.0` | `0.4` | Hand must be >=0.4s from object; leaves ~1.0s for pipeline |

The time-to-hit value of 0.4s is derived from:
- Downstream pipeline: ~1.0s (segmentation) + ~0.15s (delay) + ~0.3s (planning) = ~1.45s total
- A hit at t=0.4s means the hand reaches the object in 0.4s
- The pipeline would need to complete in 0.4s, but it takes ~1.45s — impossible
- 0.4s is a conservative lower bound; could be increased to 0.6-0.8s

These values should be validated with live testing and adjusted based on observed false positive/negative rates.
