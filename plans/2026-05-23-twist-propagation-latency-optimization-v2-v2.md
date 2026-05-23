# Twist Propagation Latency Optimization — Revised Plan

## Objective

Reduce the twist propagation stage latency from **~160 ms mean / ~176 ms P95** by implementing three complementary optimizations:

1. **External twist subscription** — use the odometry-provided twist directly instead of estimating from pose finite differences
2. **Background warm cache** — run twist estimation and KDTree construction continuously while inactive
3. **Higher cycle rate** — reduce timer period from 100 ms to 20 ms

---

## Current Architecture

### Data Flow

There are two launch configurations that feed `/hand_pose`:

**Pipeline / Twist Test launches** (real hardware path):
```
OpenVINS (/ov_msckf_arm/odomimu)
  → odom_to_pose_relay.py
    → /hand_pose (PoseStamped)     ← position + orientation only
    → /hand_twist (TwistStamped)   ← real fused linear+angular velocity
    → /hand_odom   (Odometry)      ← full odometry with covariance
```

**Digital Twin / Grasp Test / Mock launches** (no odometry):
```
hand_pose_publisher.py (reads TF at 50 Hz)
  → /hand_pose (PoseStamped)       ← position + orientation only
  (no /hand_twist, no /hand_odom)
```

### Current Twist Estimation

The twist propagation node currently **ignores** `/hand_twist` and `/hand_odom` twist data. It estimates velocity itself from the pose buffer via finite differences at `twist_propagation_node.py:729-791`:
1. Takes last 2 poses from buffer
2. Computes `dx/dt` for linear velocity
3. If ≥3 poses available, uses least-squares fit for smoother estimate
4. Applies exponential moving average (alpha=0.4)

This requires ≥2 poses in the buffer (cleared on activation) and introduces ~40 ms accumulation delay.

---

## Implementation Plan

### Part 1: External Twist Subscription

**Goal**: Use the real odometry twist when available, eliminating the pose buffer requirement for velocity estimation.

- [ ] **Task 1.1**: Add a new parameter `hand_twist_input_topic` (default: `/hand_twist`) to the parameter declarations at `twist_propagation_node.py:434`.

  ```python
  self.declare_parameter("hand_twist_input_topic", "/hand_twist")
  ```

  Rationale: Separate from `hand_twist_topic` (the output topic the node publishes to). This is the input subscription.

- [ ] **Task 1.2**: Add a new state variable `_external_twist` (tuple or None) and `_external_twist_time` (float) at `twist_propagation_node.py:542` (alongside existing state).

  ```python
  self._external_twist: tuple | None = None  # (vx,vy,vz,wx,wy,wz) from odometry
  self._external_twist_time: float = 0.0     # timestamp of last external twist
  ```

- [ ] **Task 1.3**: Add a subscription to the external twist topic in the subscriptions section at `twist_propagation_node.py:580`. Subscribe conditionally — only if the topic parameter is non-empty (matching the pattern used for `odom_topic` at line 594).

  ```python
  hand_twist_input = self.get_parameter("hand_twist_input_topic").value
  if hand_twist_input:
      self.create_subscription(TwistStamped, hand_twist_input, self._on_hand_twist, 10)
  ```

- [ ] **Task 1.4**: Implement `_on_hand_twist` callback (near `_on_hand_pose` at line 682). Stores the latest twist values and timestamp.

  ```python
  def _on_hand_twist(self, msg: TwistStamped):
      t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
      with self._lock:
          self._external_twist = (
              msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z,
              msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z,
          )
          self._external_twist_time = t
  ```

- [ ] **Task 1.5**: Add a staleness threshold parameter `external_twist_max_age_s` (default: `0.5`) to guard against stale twist data. If the external twist is older than this, fall back to finite-difference estimation.

  ```python
  self.declare_parameter("external_twist_max_age_s", 0.5)
  ```

- [ ] **Task 1.6**: Modify `_estimate_twist()` at `twist_propagation_node.py:729` to check for a fresh external twist first. New logic:

  1. Check if `_external_twist` is not None and age < `external_twist_max_age_s`
  2. If yes: return `_external_twist` directly (skip finite differences)
  3. If no: fall back to current finite-difference + least-squares logic

  This preserves full backward compatibility — when no external twist topic is available, the method works exactly as before.

- [ ] **Task 1.7**: Add a `_twist_source` tracking variable (string: `"external"` or `"estimated"`) that records which source was used in the last cycle. Include this in the status JSON published by `_publish_status()`. Log the chosen method once at startup and on source transitions.

  In `_estimate_twist()`:
  ```python
  if external_twist is fresh:
      self._twist_source = "external"
      return external_twist
  else:
      self._twist_source = "estimated"
      # ... existing finite-diff logic
  ```

  In `_publish_status()`, add:
  ```python
  "twist_source": self._twist_source
  ```

  At startup (end of `__init__`), log:
  ```python
  if hand_twist_input:
      self.get_logger().info(f"External twist source: {hand_twist_input}")
  else:
      self.get_logger().info("No external twist topic — using pose finite-difference estimation")
  ```

- [ ] **Task 1.8**: Modify `_run_idle_cycle()` at `twist_propagation_node.py:1182` to relax the pose buffer requirement when external twist is available. Current check:

  ```python
  if len(self._pose_buf) < 2:
      self._publish_status(reason="waiting_for_poses")
      return
  ```

  New logic: when `_external_twist` is fresh, only require ≥1 pose (for the current position/orientation). When using estimated twist, keep the ≥2 requirement.

  ```python
  min_poses = 1 if self._twist_source == "external" else 2
  if len(self._pose_buf) < min_poses:
      self._publish_status(reason="waiting_for_poses")
      return
  ```

  Rationale: With external twist, the node only needs the current pose to know *where* the hand is. The velocity comes from odometry, not from pose differences.

- [ ] **Task 1.9**: Add the `hand_twist_input_topic` parameter to `config/prosthesis_config.yaml` in both the ROS2 section (line 194) and the flat reference section (line 395):

  ```yaml
  # In ROS2 section (after hand_twist_topic at line 216):
  hand_twist_input_topic: "/hand_twist"

  # In flat section (after hand_twist_topic):
  hand_twist_input_topic: "/hand_twist"
  ```

  Note: `odom_to_pose_relay.py` already publishes to `/hand_twist` at `odom_to_pose_relay.py:74`. No changes needed to the relay.

- [ ] **Task 1.10**: Update the mock launch (`mock.launch.py`) to also run `odom_to_pose_relay` so the mock system provides external twist. The mock test script already publishes approach trajectories to `/hand_pose` — it should additionally publish to `/hand_twist` or we should add a simple relay node.

  **Recommended approach**: Have the latency test script (`run_latency_stages.py`) publish `TwistStamped` messages alongside `PoseStamped` messages in the approach trajectory thread. This avoids adding a node to the mock launch and keeps the test self-contained.

  In the test's `_publish_approach_trajectory` background thread, after publishing each `PoseStamped`, also compute and publish a `TwistStamped` with the approach velocity (constant direction, known speed). This feeds the external twist subscription directly.

### Part 2: Background Warm Cache

**Goal**: Run a read-only subset of the idle cycle while inactive so the KDTree and twist are pre-computed on activation.

- [ ] **Task 2.1**: Add a new parameter `background_cycle_enabled` (default: `True`) at `twist_propagation_node.py:426`:

  ```python
  self.declare_parameter("background_cycle_enabled", True)
  ```

  And in config:
  ```yaml
  background_cycle_enabled: true
  ```

- [ ] **Task 2.2**: Implement `_run_background_cycle()` method. This is a read-only version of `_run_idle_cycle()` that:

  **Does**:
  - Check for external twist or estimate from pose buffer (calls `_estimate_twist()`)
  - Call `_get_kdtree()` to build/maintain the KDTree cache
  - Update `self._twist` with the latest estimate

  **Does NOT**:
  - Publish clicks (`_click_pub`)
  - Publish segmentation resets (`_reset_pub`)
  - Transition state machine (`_cycle_state`)
  - Publish visualization markers (path, spheres, hit marker, trajectory line)
  - Call the preshaping service

  Optionally publishes twist and current pose (for debugging), gated by a parameter or just always (they're small messages and downstream nodes ignore them when the status shows `active: false`).

  Implementation: extract the twist estimation + KDTree warming into a shared helper that both `_run_background_cycle` and `_run_idle_cycle` call, or simply duplicate the relevant ~10 lines (simpler, less coupling).

  ```python
  def _run_background_cycle(self):
      """Warm the twist and KDTree caches while inactive."""
      # Update twist from external source or pose buffer
      self._estimate_twist()  # updates self._twist as side effect

      # Ensure KDTree is built from current cloud
      if self._cloud_xyz is not None:
          self._get_kdtree()
  ```

- [ ] **Task 2.3**: Modify `_cycle_callback()` at `twist_propagation_node.py:1133-1135` to call the background cycle when inactive:

  ```python
  if not active:
      if self._background_cycle_enabled:
          self._run_background_cycle()
      self._publish_status()
      return
  ```

- [ ] **Task 2.4**: Add a `_just_activated` flag. Modify `_on_activate()` at `twist_propagation_node.py:654` to set it:

  ```python
  def _on_activate(self, _req, resp):
      with self._lock:
          self._active = True
          self._cycle_state = CycleState.IDLE
          self._pose_buf.clear()
          self._seg_trigger_time = 0.0
          self._just_activated = True  # NEW
      ...
  ```

  Rationale: Signals to `_run_idle_cycle` that the first cycle after activation can use the pre-computed twist and cached KDTree without waiting for the pose buffer to fill.

- [ ] **Task 2.5**: Modify `_run_idle_cycle()` to handle the `_just_activated` fast path. When `_just_activated = True`:

  - If external twist is available: only need ≥1 pose (already handled by Task 1.8)
  - If estimated twist: accept the pre-computed `_twist` from the background cycle even with <2 poses. Use the latest single pose for position, and the background-maintained twist for velocity.
  - Clear `_just_activated = False` after first cycle.

  The key change: when `_just_activated` and `_twist_source == "estimated"` and `len(pose_buf) < 2`, skip the pose buffer check and use the pre-computed twist directly. This eliminates the ~40 ms pose accumulation wait even in the estimated-twist path.

  ```python
  if self._just_activated:
      # Background cycle already warmed the twist — allow single pose
      if len(self._pose_buf) < 1:
          self._publish_status(reason="waiting_for_poses")
          return
      self._just_activated = False
      # Use self._twist as-is (pre-computed by background cycle)
  else:
      min_poses = 1 if self._twist_source == "external" else 2
      if len(self._pose_buf) < min_poses:
          self._publish_status(reason="waiting_for_poses")
          return
      self._twist = self._estimate_twist()
  ```

### Part 3: Higher Cycle Rate

**Goal**: Reduce average polling wait from ~50 ms to ~10 ms.

- [ ] **Task 3.1**: Change `cycle_delay_s` default from `0.1` to `0.02` in two places:

  1. Code default at `twist_propagation_node.py:409`:
     ```python
     self.declare_parameter("cycle_delay_s", 0.02)
     ```

  2. Config default at `config/prosthesis_config.yaml:196`:
     ```yaml
     cycle_delay_s: 0.02
     ```

  Also update the flat reference section at `config/prosthesis_config.yaml:396`.

  Rationale: Propagation computation per cycle is <1 ms. At 50 Hz (20 ms period), average polling wait = ~10 ms. CPU cost: 50 cycles/s × 1 ms = 5% — negligible.

- [ ] **Task 3.2**: Throttle status publication to avoid flooding at 50 Hz. The status JSON is published every cycle. At 50 Hz that's 50 messages/sec of ~500 bytes each = ~25 KB/s. Not harmful but unnecessary.

  Add a `_status_throttle_counter` and only publish status every Nth cycle:

  ```python
  self._status_counter = 0
  # In _cycle_callback:
  self._status_counter += 1
  if self._status_counter % 5 == 0:  # every 5th cycle = 10 Hz status
      self._publish_status()
  ```

  Always publish status on state transitions (hit found, activation, etc.) — only throttle the periodic "no hit" / "waiting" status.

### Part 4: Update Latency Benchmark

- [ ] **Task 4.1**: Update `run_latency_stages.py` to publish `TwistStamped` alongside `PoseStamped` in the approach trajectory thread. The twist should reflect the actual approach velocity (e.g., `linear.x = 0.10` m/s for a forward-moving trajectory).

- [ ] **Task 4.2**: Add CLI flags for A/B testing:
  - `--no-background-cycle` — sets `background_cycle_enabled: false`
  - `--cycle-delay SECONDS` — overrides `cycle_delay_s` (default: `0.02`)
  - `--no-external-twist` — disables the external twist subscription for comparison

- [ ] **Task 4.3**: Output a comparison CSV `results/latency_optimization_comparison.csv` with columns for each configuration tested.

### Part 5: Verification

- [ ] **Task 5.1**: Build and run syntax check on modified files.

- [ ] **Task 5.2**: Run the latency benchmark with the optimized configuration (external twist + background cycle + 50 Hz). Record results.

- [ ] **Task 5.3**: Run the latency benchmark with the baseline configuration (no external twist + no background cycle + 10 Hz). Verify results match the original ~160 ms / ~240 ms numbers.

- [ ] **Task 5.4**: Verify that the digital twin / grasp test launches still work correctly (no external twist available — should fall back to finite-difference estimation gracefully).

- [ ] **Task 5.5**: Verify the mock launch test still passes end-to-end.

- [ ] **Task 5.6**: Check that the log output clearly shows which twist source is being used (external vs estimated).

---

## Expected Latency Improvement

| Configuration | Twist Prop (ms) | Total Pipeline (ms) | vs MAR 400ms | vs IDE 100ms |
|---|---|---|---|---|
| Baseline (10 Hz, no bg, estimated twist) | ~160 | ~240 | PASS | FAIL |
| + External twist only | ~120 | ~200 | PASS | FAIL |
| + Background cycle only | ~110 | ~190 | PASS | FAIL |
| + 50 Hz cycle only | ~120 | ~200 | PASS | FAIL |
| **All three combined** | **~15-25** | **~95-105** | **PASS** | **BORDERLINE** |

### How each optimization contributes

| Optimization | Savings | Mechanism |
|---|---|---|
| External twist | ~40 ms | Eliminates pose buffer accumulation wait (0 poses needed instead of 2) |
| Background cycle | ~20-30 ms | Pre-computes KDTree, maintains twist estimate before activation |
| 50 Hz cycle | ~40 ms | Reduces average polling wait from ~50 ms to ~10 ms |
| **Combined** | **~100-110 ms** | All savings stack |

---

## Files to Modify

| File | Changes |
|---|---|
| `src/twist_propagation/twist_propagation/twist_propagation_node.py` | Tasks 1.1–1.8, 2.1–2.5, 3.1–3.2 |
| `config/prosthesis_config.yaml` | Tasks 1.9, 2.1, 3.1 (parameter updates) |
| `tests/test1_software_verification/run_latency_stages.py` | Tasks 4.1–4.3 |
| `src/prosthesis_launch/launch/mock.launch.py` | No changes (test publishes twist directly) |

---

## Verification Criteria

- [ ] Log output shows `"External twist source: /hand_twist"` when odometry relay is running
- [ ] Log output shows `"No external twist topic — using pose finite-difference estimation"` when no relay
- [ ] Status JSON includes `"twist_source": "external"` or `"twist_source": "estimated"`
- [ ] Latency benchmark shows ≥80% reduction in twist propagation stage with all optimizations
- [ ] Total pipeline P95 ≤ 400 ms (MAR) — must still PASS
- [ ] Baseline A/B test matches original ~160 ms numbers
- [ ] No regression in existing tests

## Potential Risks and Mitigations

1. **External twist and pose are from different clock domains**
   Mitigation: The staleness check (`external_twist_max_age_s: 0.5`) ensures the twist is reasonably fresh. Both come from the same odometry message in `odom_to_pose_relay.py`, so they're inherently synchronized when the relay is used.

2. **Background cycle consumes CPU when inactive**
   Mitigation: `_estimate_twist()` takes <0.1 ms, `_get_kdtree()` takes ~10-30 ms but only when a new cloud arrives (15 Hz → ~0.3 ms average). At 50 Hz cycle rate, total background CPU is <5%. Negligible.

3. **50 Hz cycle rate increases DDS traffic**
   Mitigation: Status publication is throttled to 10 Hz (Task 3.2). Other publications (twist, pose) are small messages. Click/state-transition publications only happen on hits, not every cycle.

4. **External twist unavailable in some launch configurations**
   Mitigation: The parameter `hand_twist_input_topic` defaults to `/hand_twist` but can be set to empty string to disable. The `_estimate_twist()` fallback preserves the original finite-difference behavior. The mock launch and digital twin paths work without changes.

5. **Background cycle twist estimate becomes stale**
   Mitigation: The background cycle continuously updates `_twist` from the latest poses. On activation, the `_just_activated` flag allows using this pre-computed twist for the first cycle only. Subsequent cycles re-estimate from fresh poses (or use external twist).
