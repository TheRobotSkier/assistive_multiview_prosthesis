# Twist Propagation: Minimum Speed & Minimum Hit Duration Thresholds

## Objective

Add two new safety filters to the twist propagation hit detection pipeline:
1. **Minimum linear speed threshold** — suppress hit detection when the hand is stationary or barely moving
2. **Minimum hit persistence duration** — require a hit to be sustained across multiple consecutive cycles before triggering segmentation

The goal is to minimize unintentional/false hits from tracking noise, hand drift, and momentary glancing collisions, while preserving responsive hit detection for genuine grasp approaches.

---

## Current System Analysis

### How Hits Are Currently Triggered

The hit detection flow in `_run_idle_cycle()` (`twist_propagation_node.py:1171-1338`):

1. Estimate twist from pose buffer (`twist_propagation_node.py:1202`)
2. Apply origin offset (camera → fingertips) (`twist_propagation_node.py:1211-1225`)
3. Transform to cloud frame (`twist_propagation_node.py:1228-1229`)
4. Propagate pose forward by twist, checking KDTree collision at each step (`twist_propagation_node.py:1240-1243`)
5. **Immediately** trigger segmentation on first hit — no speed check, no persistence requirement

### What's Currently Missing

- **No minimum speed gate**: A stationary hand (zero twist) that happens to be near cloud geometry will trigger a hit immediately. The propagation starts from the current position, and with zero velocity, the first propagation step (at `propagation_dt_s = 0.02s`) stays at the same spot. If that spot is within the effective collision threshold (0.10m) of cloud points, a hit fires.
- **No hit persistence/debounce**: A hit on a single cycle immediately triggers the full segmentation → preshaping pipeline. There's no requirement for the hit to be stable or confirmed across multiple cycles.
- **The EMA smoothing (alpha=0.4)** helps with twist noise but doesn't prevent near-zero velocities from being propagated.

### Why This Matters

From the camera test logs and known issues:
- **Tracking drift** (`test-noter.md`: "arm drifted away at the end", "PC drifted from marker 0") can cause false velocity estimates
- **The propagation origin offset** (`[0.1543, -0.1485, 0.1352]`) places the start point at the fingertips, which may be near objects even when the user hasn't initiated a grasp
- **The 0.10m effective collision threshold** is generous enough that a hand resting near a table surface could trigger
- **State machine lockout**: once a hit triggers, the node enters WAITING_FOR_SEGMENTATION → WAITING_FOR_PRESHAPING, blocking further detection for potentially 5+ seconds

---

## Implementation Plan

### Phase 1: Minimum Linear Speed Threshold

- [ ] **Task 1.1**: Add new ROS parameters `min_twist_linear_mps` (default `0.0`) to `TwistPropagationNode.__init__()` at `twist_propagation_node.py:404-509`. This parameter sets the minimum linear velocity magnitude (m/s) below which hit detection is suppressed. Default of `0.0` preserves backward compatibility.
- [ ] **Task 1.2**: Read and store the parameter alongside existing parameter reads at `twist_propagation_node.py:476-509`: `self._min_twist_linear = self.get_parameter("min_twist_linear_mps").value`
- [ ] **Task 1.3**: Add a speed gate check in `_run_idle_cycle()` immediately after twist estimation (`twist_propagation_node.py:1202`) and before propagation. Compute `lin_mag = math.sqrt(vx**2 + vy**2 + vz**2)`. If `lin_mag < self._min_twist_linear`, skip propagation and publish status with `reason="below_min_speed"`. This prevents the expensive KDTree propagation when the hand is essentially stationary.
- [ ] **Task 1.4**: Add `min_twist_linear_mps: 0.0` to `src/twist_propagation/config/twist_propagation.yaml` under the "Twist estimation" section (after `twist_estimation_window`).
- [ ] **Task 1.5**: Add `min_twist_linear_mps` to `config/prosthesis_config.yaml` in the `twist_propagation` section (after `twist_estimation_window`). Set initial runtime value to something like `0.02` or `0.03` m/s (2-3 cm/s) — slow enough to not miss real approaches, fast enough to reject drift.

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
- [ ] **Task 2.7**: Add `min_hit_persist_s` to `config/prosthesis_config.yaml`. Suggested initial value: `0.1` to `0.2` seconds (1-2 cycles at 10Hz). This is short enough to not add noticeable latency but long enough to reject single-cycle glitches.

**Rationale**: The persistence requirement acts as a temporal debounce. It ensures that momentary noise spikes or single-frame glitches don't trigger the full pipeline. The spatial proximity check for "same hit" reuses the existing `segmentation_retarget_distance_m` parameter.

### Phase 3: Unit Tests

- [ ] **Task 3.1**: Add test class `TestMinSpeedFilter` to `src/twist_propagation/test/test_twist_propagation.py`:
  - `test_zero_speed_suppresses_hit`: Verify that with `min_twist_linear_mps > 0`, a zero-velocity twist does not produce a hit
  - `test_below_threshold_suppresses_hit`: Verify that a velocity just below the threshold is suppressed
  - `test_above_threshold_allows_hit`: Verify that a velocity at/above the threshold proceeds normally
  - `test_default_zero_backward_compatible`: Verify that `min_twist_linear_mps = 0.0` does not filter anything
- [ ] **Task 3.2**: Add test class `TestHitPersistence` to the same test file:
  - `test_zero_persist_triggers_immediately`: With `min_hit_persist_s = 0.0`, a single hit triggers immediately
  - `test_hit_must_persist`: With `min_hit_persist_s > 0`, a hit on one cycle is not sufficient
  - `test_hit_persist_satisfied`: A hit sustained for the required duration triggers
  - `test_transient_hit_cleared`: A hit that disappears on the next cycle is cleared
  - `test_hit_moved_resets_timer`: A hit that moves beyond retarget distance resets the timer

**Rationale**: These are pure logic tests that don't require ROS 2. They test the filtering logic in isolation, consistent with the existing test pattern.

### Phase 4: Integration Tests

- [ ] **Task 4.1**: Add integration test cases to `scripts/test_twist_propagation_integration.py`:
  - Test that a stationary hand near a cloud does NOT trigger segmentation when `min_twist_linear_mps > 0`
  - Test that a moving hand DOES trigger segmentation even with the speed threshold
  - Test that a brief single-cycle hit does NOT trigger segmentation when `min_hit_persist_s > 0`
  - Test that a sustained hit DOES trigger after the persistence duration

**Rationale**: Integration tests verify the full node behavior with actual ROS 2 message passing.

---

## Verification Criteria

- [ ] With `min_twist_linear_mps = 0.0` and `min_hit_persist_s = 0.0`, behavior is identical to the current system (backward compatibility)
- [ ] A stationary hand (twist magnitude < threshold) does not trigger segmentation, even when positioned near cloud geometry
- [ ] A hand moving above the minimum speed threshold triggers normally
- [ ] A single-cycle hit (transient) is rejected when `min_hit_persist_s > 0`
- [ ] A hit sustained for >= `min_hit_persist_s` triggers segmentation
- [ ] All existing unit tests continue to pass unchanged
- [ ] New parameters appear in both config YAML files with sensible defaults
- [ ] Status messages include the rejection reason when hits are filtered

---

## Potential Risks and Mitigations

### 1. **Increased Latency for Real Grasps**
The persistence duration adds direct latency to every hit detection. At `min_hit_persist_s = 0.2s`, the system reacts 200ms later.
**Mitigation**: Keep the default low (0.1-0.2s). The propagation already predicts ~2s into the future, so the system has significant timing margin. The speed gate adds zero latency.

### 2. **Speed Threshold Rejecting Slow but Intentional Approaches**
Some users may approach objects very slowly (< 2-3 cm/s), especially during rehabilitation or fine manipulation.
**Mitigation**: Set `min_twist_linear_mps` conservatively (0.02-0.03 m/s). Make it configurable so it can be tuned per-user or disabled entirely (`0.0`). Document the trade-off.

### 3. **Hit Persistence vs. Moving Target**
If the user is tracking a moving object (e.g., someone hands them an object), the hit point may shift between cycles, causing the persistence timer to reset.
**Mitigation**: The persistence check uses `segmentation_retarget_distance_m` (0.10m) as the spatial tolerance for "same hit." This is generous enough to accommodate small target movements. For large movements, resetting the timer is actually correct behavior.

### 4. **Interaction with Retarget Policy**
The existing retarget policy (`_should_retarget()` at `twist_propagation_node.py:323-339`) prevents re-segmenting the same object. The new persistence filter operates *before* the retarget check, so a pending hit that gets confirmed will still go through retarget logic.
**Mitigation**: This is correct behavior — persistence confirms the hit is real, then retarget decides whether it's a new target. No conflict.

### 5. **Pending Hit State Across Deactivation/Reactivation**
If the node is deactivated while a hit is pending, the pending state must be cleared. Otherwise, reactivation could immediately trigger on stale state.
**Mitigation**: Explicitly clear `_pending_hit` and `_pending_hit_start` in the deactivation service handler (`twist_propagation_node.py:658-672`).

### 6. **EMA Smoothing Interacting with Speed Gate**
The EMA (alpha=0.4) means the twist estimate lags behind the actual velocity. When the hand starts moving, the estimated speed ramps up gradually. This could delay the speed gate opening.
**Mitigation**: This is actually beneficial — it adds a natural ramp-up that rejects sudden noise spikes. The EMA time constant (~0.25s at alpha=0.4, 10Hz) is shorter than typical approach durations.

### 7. **Covariance Propagation Still Runs on Zero-Twist**
Even with the speed gate, if `min_twist_linear_mps = 0.0` (disabled), the propagation still runs with near-zero twist. The covariance grows slowly (process noise accumulates), and the horizon may not truncate early enough.
**Mitigation**: The speed gate at `min_twist_linear_mps > 0` short-circuits before propagation, avoiding this entirely. When disabled, the existing covariance truncation still applies.

### 8. **Visualization Continuity**
When the speed gate suppresses propagation, no predicted path or collision spheres are published. This could cause RViz to show stale or missing visualization.
**Mitigation**: Publish an empty path/marker array when the speed gate triggers, or publish the status with the current (non-propagated) position. The existing "no hit" path at `twist_propagation_node.py:1248-1250` already handles this.

---

## Alternative Approaches

### Alternative A: Speed Gate Only (No Persistence)
Skip the persistence requirement entirely and rely solely on the minimum speed threshold. This is simpler to implement and has zero added latency.
**Trade-offs**: Less robust against noise spikes that produce high instantaneous velocity estimates. A single noisy pose could produce a velocity spike above threshold and trigger. The persistence filter catches these.

### Alternative B: Persistence Only (No Speed Gate)
Skip the speed threshold and rely solely on hit persistence. A stationary hand near geometry would still produce persistent hits, but this is less likely if the hand is truly still (the collision sphere would need to overlap cloud points consistently).
**Trade-offs**: Doesn't address the core problem of stationary-hand false positives. The speed gate is a more direct solution for this case.

### Alternative C: Confidence-Based Gating
Instead of fixed thresholds, use the propagated covariance to compute a "hit confidence" score. Only trigger if the confidence exceeds a threshold. This would be more principled but significantly more complex.
**Trade-offs**: Much more implementation effort. The covariance is already used for horizon truncation but not for hit scoring. Would require careful tuning and more testing. Overkill for the current problem.

### Alternative D: Hysteresis on Speed Threshold (Recommended Enhancement)
Add separate enter/exit speed thresholds (like the proximity controller's `proximity_enter_threshold_m` / `proximity_exit_threshold_m`). Once the speed gate opens (speed > enter threshold), it stays open until speed drops below a lower exit threshold.
**Trade-offs**: Prevents chattering at the boundary. Adds one more parameter but follows an established pattern in the codebase. Could be added as a follow-up if boundary oscillation is observed.

---

## Recommended Parameter Values

| Parameter | Default (YAML) | Runtime Initial | Rationale |
|-----------|---------------|-----------------|-----------|
| `min_twist_linear_mps` | `0.0` | `0.02` | 2 cm/s rejects drift but allows slow intentional approaches |
| `min_hit_persist_s` | `0.0` | `0.15` | 1.5 cycles at 10Hz; rejects single-cycle glitches without noticeable latency |

These values should be validated with live testing and adjusted based on observed false positive/negative rates.
