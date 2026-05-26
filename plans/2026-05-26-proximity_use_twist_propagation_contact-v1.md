# Proximity Controller: Use Twist Propagation Contact Point as Target

## Objective

Change the proximity controller to compute distance from the **current hand's grasp contact frame** (fingertips) to the **twist propagation predicted contact point** (hit point), instead of the current approach which computes distance from the current hand pose to the preshaping bridge's `target_hand_pose` (planner output). This is a simpler, more direct proximity measure that answers the question: "how far are the fingertips from where twist propagation predicts they'll contact the object?"

## Background & Problem Analysis

### Current Data Flow

```
Twist Propagation → hit point (hit_x, hit_y, hit_z)
  → /grasp_preshaping/contact_pose (PoseStamped, position=hit point)
  → Preshaping Bridge caches contact_pose
  → Preshaping Bridge calls Rust FFI planner → target_px/py/pz
  → /grasp_preshaping/target_hand_pose (PoseStamped, position=planner target)
  → Proximity Controller computes distance:
      current_hand_pose (offset to fingertips) vs target_hand_pose (offset to fingertips)
```

The problem: the proximity controller compares the current hand position against the **planner's output target** (`target_hand_pose`), which is the planner's optimized grasp position — a different point from the predicted contact. The distance being measured is "how far is the hand from where the planner says to grasp," not "how far is the hand from the predicted contact point."

### Proposed Data Flow

```
Twist Propagation → hit point (hit_x, hit_y, hit_z)
  → /grasp_preshaping/contact_pose (PoseStamped, position=hit point)
  → Proximity Controller subscribes directly
  → Proximity Controller computes distance:
      current_hand_pose (offset to fingertips) vs contact_pose (hit point, no offset needed)
```

The hit point from twist propagation is already in the **cloud frame** (same world-frame coordinate system as the hand pose). It represents where the fingertips will collide with the object. The distance is simply: fingertip position (computed via offset) to hit point (already a world-frame 3D point).

### Key Insight: The Hit Point IS Already at the Contact Location

The twist propagation node propagates from the **grasp contact offset position** (fingertips), not from the camera origin. Looking at `twist_propagation_node.py:1567-1573`, the propagation offset is applied to shift the start point from the camera to the fingertips before trajectory prediction. This means the predicted hit point `(hit_x, hit_y, hit_z)` is already where the **fingertips** will contact the object surface.

Therefore:
- **Current hand pose**: needs the `grasp_contact_offset` applied (to get from camera origin to fingertips)
- **Contact/hit point**: does NOT need an offset — it's already at the fingertip contact location

## Implementation Plan

### Step 1: Add Subscription to Contact Pose in Proximity Controller

- [ ] **1.1** Add a new subscription to `/grasp_preshaping/contact_pose` (`PoseStamped`) in `src/grasp_preshaping/nodes/grasp_proximity_controller_node.py`. Add a parameter `contact_pose_topic` with default `"/grasp_preshaping/contact_pose"` (declared alongside the existing topic parameters around line 90-98). Create the subscription in the subscriptions section (after line 211), with a callback `_on_contact_pose`.

- [ ] **1.2** Add a new state variable `self._contact_pose: Pose | None = None` in the state section (around line 136). This stores the latest predicted contact point.

- [ ] **1.3** Implement `_on_contact_pose(self, msg: PoseStamped)` callback. Store `msg.pose` into `self._contact_pose`. This is a simple cache — no plan commit logic needed since the contact pose is used directly as the distance target, not as part of the 3-part plan (closures + wrist + hand frame).

### Step 2: Modify Distance Computation

- [ ] **2.1** Modify `_compute_proximity_distance()` at `src/grasp_preshaping/nodes/grasp_proximity_controller_node.py:438-457`. Change the method signature to accept the contact pose instead of the planned hand frame:

  ```
  Current:  _compute_proximity_distance(current: PoseStamped, planned: Pose)
  Proposed: _compute_proximity_distance(current: PoseStamped, contact: Pose)
  ```

  The new logic:
  - `cur_pos = self._apply_offset(current.pose, self._grasp_contact_offset)` — shift current hand pose from camera origin to fingertips (unchanged)
  - `contact_pos = (contact.position.x, contact.position.y, contact.position.z)` — use the hit point directly, **no offset applied** (the hit point is already at the fingertip contact location)
  - Compute Euclidean distance between `cur_pos` and `contact_pos` as before

- [ ] **2.2** Remove the `_apply_offset` call for the planned/target pose. The contact pose from twist propagation is already in world-frame coordinates at the predicted fingertip contact location. Applying the offset again would double-shift it.

### Step 3: Update Control Loop to Use Contact Pose

- [ ] **3.1** Modify the `_control_loop()` method at `src/grasp_preshaping/nodes/grasp_proximity_controller_node.py:335-434`. Change the readiness check (lines 339-342) to require `_contact_pose` instead of `_planned_hand_frame`:

  ```
  Current:  self._planned_hand_frame is None → missing 'hand_frame'
  Proposed: self._contact_pose is None → missing 'contact_pose'
  ```

- [ ] **3.2** Update the distance computation call (line 357-358) to use the contact pose:

  ```
  Current:  dist, (dx, dy, dz) = self._compute_proximity_distance(
                self._current_hand_pose, self._planned_hand_frame)
  Proposed: dist, (dx, dy, dz) = self._compute_proximity_distance(
                self._current_hand_pose, self._contact_pose)
  ```

### Step 4: Remove Unused Target Hand Frame Subscription

- [ ] **4.1** Remove the subscription to `/grasp_preshaping/target_hand_pose` (lines 182-187) and its callback `_on_planned_hand_frame` (lines 251-253).

- [ ] **4.2** Remove `self._planned_hand_frame` state variable (line 136) and `self._buf_hand_frame` (line 141).

- [ ] **4.3** Remove `_planned_hand_frame` from the plan commit logic in `_try_commit_plan()` (lines 300-332). The plan now only requires closures + wrist rotation (2 parts, not 3). Remove the `_buf_hand_frame` check from the commit condition (line 304) and the assignment (line 306).

- [ ] **4.4** Remove `_planned_hand_frame` from `_clear_plan()` (line 280, 288).

- [ ] **4.5** Remove the `target_hand_pose_topic` parameter declaration (line 92) and its config entry in `config/prosthesis_config.yaml:138`.

### Step 5: Handle Contact Pose Freshness

- [ ] **5.1** Add a freshness check for the contact pose. Store the receive timestamp alongside the pose: `self._contact_pose_time: float | None = None` (set to `time.time()` in the callback). In the control loop, check that the contact pose is not stale (e.g., less than 5 seconds old). If stale, log a warning and skip the control cycle. This prevents using an outdated hit point if twist propagation has deactivated or the hand has moved significantly.

- [ ] **5.2** Clear `_contact_pose` when the pipeline leaves APPROACHING state (in `_on_pipeline_state`, around line 265-266). This ensures a fresh contact pose is required for each approach cycle.

### Step 6: Configuration Updates

- [ ] **6.1** Add `contact_pose_topic` parameter to `config/prosthesis_config.yaml` under `proximity_controller.ros__parameters` (around line 144):

  ```yaml
  contact_pose_topic: "/grasp_preshaping/contact_pose"
  ```

- [ ] **6.2** Remove `target_hand_pose_topic` from `config/prosthesis_config.yaml:138` since it is no longer used.

- [ ] **6.3** Keep the existing `grasp_contact_offset` parameter and its value in `config/prosthesis_config.yaml:155`. It is still needed to shift the current hand pose from the camera origin to the fingertips. The Z-sign bug (`-0.1352` vs `+0.1352`) should be fixed to `+0.1352` to match twist propagation, but this is a separate concern and can be done as part of this change or as a follow-up.

### Step 7: Fix Z-Sign Bug (Optional but Recommended)

- [ ] **7.1** Fix the `grasp_contact_offset` Z-sign in `config/prosthesis_config.yaml:155`. Change `[0.1543, -0.1485, -0.1352]` to `[0.1543, -0.1485, 0.1352]` to match the twist propagation node's `propagation_origin_offset` at line 332. Since the proximity controller now uses the same offset concept (shifting camera origin to fingertips), the values should be identical.

## Verification Criteria

1. **Distance correctness**: The proximity distance is now computed as: `distance(fingertip_of_current_hand, predicted_hit_point)`. The hit point is already at the fingertip contact location, so only the current hand pose needs the offset.
2. **Near/far zone transitions**: The hysteresis behavior (enter at 8cm, exit at 20cm, 3-sample debounce) should work identically — only the distance source has changed.
3. **Pipeline integration**: The `/proximity/near_zone_entered` topic is still published correctly, and the pipeline manager transitions from APPROACHING to GRASPING as before.
4. **No stale data**: The contact pose freshness check prevents using outdated hit points.
5. **Plan commit still works**: Closures and wrist rotation still commit atomically. The contact pose is independent and always uses the latest value.

## Potential Risks and Mitigations

1. **Contact pose may not be available when APPROACHING starts**
   - Mitigation: The control loop already waits for all required data before proceeding. If `_contact_pose` is None, the loop logs "waiting for: contact_pose" and returns. The twist propagation node publishes contact_pose before the pipeline transitions to APPROACHING, so this should be available.

2. **Contact pose frame may differ from hand pose frame**
   - Mitigation: Both are in the same world-frame coordinate system. The hand pose comes from `hand_pose_publisher.py` which looks up `world → wrist_link`. The contact pose comes from twist propagation in `_cloud_frame`, which is the fused cloud frame (also world-frame). If there is a frame mismatch, it would need a TF transform — but in the current system both are in the same frame.

3. **Removing target_hand_pose breaks other consumers**
   - Mitigation: Check that no other node subscribes to `/grasp_preshaping/target_hand_pose` besides the proximity controller. The `hand_trajectory_publisher.py` in pipeline_manager also subscribes to it — verify whether it needs this topic independently. If so, keep the publisher in the preshaping bridge but remove only the proximity controller's subscription.

4. **Hit point drifts if hand moves after hit detection**
   - Mitigation: The twist propagation node continuously re-detects hits and publishes updated contact poses. The proximity controller always uses the latest value. The freshness check (5 seconds) prevents using very stale data.

5. **Double-offset if hit point was NOT computed from offset position**
   - Mitigation: Verify that the twist propagation node applies `propagation_origin_offset` before propagation (confirmed at `twist_propagation_node.py:1567-1573`). The hit point is indeed at the fingertip location, so no offset should be applied to it.

## Alternative Approaches

1. **Keep both targets (planned + contact) and use contact as primary**: Subscribe to both `/grasp_preshaping/contact_pose` and `/grasp_preshaping/target_hand_pose`, prefer contact pose when available, fall back to target_hand_pose. This provides redundancy but adds complexity.

2. **Use `/twist_propagation/collision_distance` directly**: The twist propagation node already publishes the collision distance as a `Float64` on `/twist_propagation/collision_distance` (line 703-704). The proximity controller could subscribe to this directly instead of computing its own distance. However, this distance is computed from the **propagation start point** (which includes the offset), not from the live hand position, so it may be stale by the time the proximity controller reads it. Computing the distance fresh in the proximity controller is more accurate.

3. **Keep target_hand_pose, just fix the offset**: The minimal fix — only fix the Z-sign bug and keep using `target_hand_pose`. This doesn't address the fundamental issue that the planner's output target is a different point from the predicted contact location.
