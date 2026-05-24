# Contact-Pose-Based Preshaping Input

## Objective

Replace the live hand pose + twist (which may be far from the object) with the predicted contact position + zero twist as input to the grasp preshaping pipeline. This concentrates the ROI and SMC particles around where the object actually is, eliminating "No points in ROI" failures and dramatically improving grasp score stability.

**Core insight**: The twist propagation node already predicts where the hand will contact the object ("hit point"). Instead of ignoring this and feeding the current (potentially distant) hand pose to the planner, we publish a synthetic pose at the hit position with zero velocity. The segmentation round-trip (200–800 ms) provides more than enough time for DDS to deliver these messages to the preshaping bridge before the Trigger service call, requiring no artificial delays.

## Implementation Plan

### Phase 1: Publish contact pose/twist from twist propagation node

- [x] **1.1** Add two new publishers in `__init__` (near line 713):
  - `self._contact_pose_pub` → `"/grasp_preshaping/contact_pose"` (`PoseStamped`, QoS 10)
  - `self._contact_twist_pub` → `"/grasp_preshaping/contact_twist"` (`TwistStamped`, QoS 10)
  - Rationale: Dedicated override topics keep `/hand_pose` and `/hand_twist` ownership clean (no competing publishers)

- [x] **1.2** Add a helper method `_publish_contact_state(hit_x, hit_y, hit_z)` that:
  1. Retrieves the most recent hand orientation from `self._pose_buf` (the quaternion `qx, qy, qz, qw`)
  2. Constructs a `PoseStamped` with `header.frame_id = self._cloud_frame`, position = (hit_x, hit_y, hit_z), orientation = current hand quaternion
  3. Constructs a `TwistStamped` with `header.frame_id = self._cloud_frame`, all linear and angular velocities set to 0.0
  4. Publishes both
  - Rationale: The hit position provides *where* contact will occur; the current hand orientation provides *how* the wrist is oriented. Zero twist enforces the stationary assumption.

- [x] **1.3** Call `_publish_contact_state()` at line 1702 (after `_publish_hit_marker`), BEFORE the click cluster publishing.
  - This is the earliest moment after a valid hit is confirmed (retarget check passed)
  - The contact pose/twist are published *before* the click cluster, ensuring the preshaping bridge receives them well before the Trigger service call
  - Rationale: The segmentation round-trip provides 200–800 ms of delivery headroom — no artificial delay needed

### Phase 2: Subscribe to contact override topics in preshaping bridge

- [x] **2.1** Add five new member variables in `PreshapingServiceBridgeNode` (line 600–608):
  ```cpp
  bool has_contact_pose_ = false;
  geometry_msgs::msg::PoseStamped latest_contact_pose_;
  bool has_contact_twist_ = false;
  geometry_msgs::msg::TwistStamped latest_contact_twist_;
  ```

- [x] **2.2** Add two new subscriptions in the constructor (lines 140–163):
  - `"/grasp_preshaping/contact_pose"` → store in `latest_contact_pose_`, set `has_contact_pose_ = true`
  - `"/grasp_preshaping/contact_twist"` → store in `latest_contact_twist_`, set `has_contact_twist_ = true`
  - Both protected by `input_mutex_` (consistent with existing subscriptions)
  - Rationale: These are the dedicated override channels; the bridge already follows the pattern of caching latest values from subscriptions

- [x] **2.3** The recency check is inlined in `try_handle_direct_request()` at lines 321–323:
  - `has_contact_pose_` and `has_contact_twist_` are both true
  - The contact pose header stamp is within 2.0 seconds of `now()`
  - Rationale: A 2-second recency window is generous (segmentation takes <1s) while preventing stale overrides from expired hit detections

### Phase 3: Modify pose/twist selection in bridge service handler

- [x] **3.1** In `try_handle_direct_request()` at lines 321–327, the override logic:
  ```cpp
  pose = latest_pose_;
  twist = latest_twist_;
  ```
  with:
  ```cpp
  if (has_contact_pose_ && has_contact_twist_ &&
      (now() - latest_contact_pose_.header.stamp) < rclcpp::Duration(2, 0)) {
    pose = latest_contact_pose_;
    twist = latest_contact_twist_;
  } else {
    pose = latest_pose_;
    twist = latest_twist_;
  }
  ```
  - Rationale: Prefer the contact override when available and recent. Fall back to live hand pose for backward compatibility (manual Trigger calls, testing without twist propagation)

- [x] **3.2** Added log line at lines 330–336:
  ```
  "Planner called with contact override: target=(x,y,z), zero twist"
  ```
  - Rationale: Makes it visible in logs when the new path is active, aiding debugging

### Phase 4: Rebuild and validate

- [ ] **4.1** Rebuild both packages: `make tonight-build` or `colcon build --packages-select twist_propagation grasp_preshaping`

- [ ] **4.2** Run existing camera-test scenario and verify:
  - No more "No points in ROI" errors in logs
  - Pipeline times stay similar or improve (smaller ROI → faster TSDF)
  - Combined scores consistently above 0.3
  - The log line "Planner called with contact override" appears when twist propagation triggers preshaping

- [ ] **4.3** Verify backward compatibility: call `/grasp_preshaping/compute_grasp` manually (without twist propagation) — the bridge should fall back to live hand pose and function identically to before

## Implementation Status

All code changes are complete. Python syntax verified. C++ brace balance verified. Build requires Docker/Jetson environment.

### Changes Made

1. **`src/twist_propagation/twist_propagation/twist_propagation_node.py`**:
   - Lines 720–723: Two new publishers (`_contact_pose_pub`, `_contact_twist_pub`)
   - Lines 1294–1336: New `_publish_contact_state()` method
   - Line 1702: Call to `_publish_contact_state()` after hit marker, before click cluster

2. **`src/grasp_preshaping/nodes/preshaping_service_bridge_node.cpp`**:
   - Lines 140–163: Two new subscriptions with topic parameters
   - Lines 319–327: Override logic inside `input_mutex_` lock
   - Lines 330–336: Log line when override is used
   - Lines 600–608: Member variables for cached contact state
   - Lines 627–629: Subscription smart pointers

## Verification Criteria

- Contact pose published immediately at hit detection (visible via `ros2 topic echo /grasp_preshaping/contact_pose`)
- Contact twist contains all zeros (visible via `ros2 topic echo /grasp_preshaping/contact_twist`)
- Bridge log shows "contact override" when twist propagation triggers preshaping
- Bridge log shows "Planner called" with camera positions near the contact point (not the current hand position)
- Combined grasp scores increase from the 0.18–0.22 baseline to 0.35+ in dynamic scenarios
- "No points in ROI" errors are eliminated in dynamic testing
- Manual Trigger calls (no twist propagation) still work correctly via fallback

## Potential Risks and Mitigations

1. **Risk: Contact pose arrives after Trigger service call**
   Mitigation: The contact pose is published BEFORE the click cluster (line 1680), and the click triggers segmentation which takes 200–800 ms. The Trigger is only called after segmentation completes. This provides hundreds of milliseconds for DDS delivery — far more than needed (ROS 2 intra-process delivery is typically <1 ms; inter-process <5 ms).

2. **Risk: Stale contact override used for wrong object**
   Mitigation: The 2-second recency check in `is_contact_override_valid()` prevents stale overrides. Additionally, the twist propagation node transitions back to IDLE after preshaping completes, and only publishes a new contact override on the next valid hit.

3. **Risk: Contact orientation is wrong (hit point has no orientation)**
   Mitigation: Using the current hand orientation is the best available — the wrist hasn't rotated yet at the moment of hit detection. The SMC still samples wrist rotations (±π/2 around local Y), so it will explore orientation space around this initial value.

4. **Risk: Zero twist assumption is wrong for fast-moving hand**
   Mitigation: At the contact point, the hand would be near-zero velocity if it were actually grasping. The SMC samples still add positional noise via the proposal variance, so the planner isn't forced to exactly the contact point — it explores a region around it.

## Alternative Approaches Considered

1. **Change Trigger service to custom `.srv` with pose/twist fields**: Cleaner but requires service definition changes, rebuild of all clients, and breaks backward compatibility. Rejected in favor of the topic-based approach which is non-breaking.

2. **Have twist propagation publish directly to `/hand_pose` and `/hand_twist`**: Simpler in code but creates competing publishers on topics owned by OpenVINS/odom_to_pose_relay. This causes message interleaving and confuses other subscribers (e.g., grasp_proximity_controller). Rejected.

3. **Use PCA-only orientation with no TSDF/SMC**: Much faster (~10 ms vs ~300 ms) but loses collision checking and validated grasp scoring. Better suited as a future optimization after this change is validated. Rejected for now, kept as future option.
