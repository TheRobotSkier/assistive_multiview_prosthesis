# Twist Propagation Node — Bug Fixes & Improvements

## Objective

Fix 9 identified issues in the twist propagation node and surrounding system. The most critical fix is the frame mismatch (Issue 1) where hand poses are not transformed to the cloud frame before propagation, making the node non-functional in the real system. Additionally, upgrade `/hand_pose` from `Pose` to `PoseStamped` and `/hand_twist` from `TwistWithCovarianceStamped` to `TwistStamped` for consistency and proper frame propagation.

## Key Design Decision: PoseStamped + TwistStamped

### Why PoseStamped instead of Pose?

`/hand_pose` is currently `geometry_msgs/Pose` — a bare message with no header, no frame_id, no timestamp. This is the root cause of Issue 1 (frame mismatch): without knowing what frame the pose is in, the node cannot transform it to the cloud frame. `PoseStamped` carries `header.frame_id` and `header.stamp`, enabling:
- Correct TF2 frame transformation before propagation
- Using message timestamps instead of wall-clock `time.time()` for twist estimation

### Why TwistStamped instead of TwistWithCovarianceStamped?

The preshaping bridge already ignores covariance (`preshaping_service_bridge_node.cpp:278`: "Covariance is intentionally ignored — fixed values are used on the Rust side"). The Rust backend uses fixed covariance from `config/grasp_preshaping.yaml:27-28`. Publishing a 36-element covariance array that no one reads is unnecessary overhead. `TwistStamped` is simpler and still carries the frame_id + timestamp.

### Impact on downstream consumers

| Consumer | Current type | Change needed |
|----------|-------------|---------------|
| `preshaping_service_bridge_node.cpp:101` | subscribes to `Pose` | change to `PoseStamped`, extract `.pose` |
| `preshaping_service_bridge_node.cpp:109` | subscribes to `TwistWithCovarianceStamped` | change to `TwistStamped`, adjust field access |
| `grasp_proximity_controller_node.py:92` | subscribes to `Pose` | change to `PoseStamped`, extract `.pose` |
| `test_twist_propagation_integration.py` | publishes `Pose`, receives `TwistWithCovarianceStamped` | update both |

The FFI layer (`GraspTwistFFI` in `ffi_types.hpp`) only has 6 fields (lx, ly, lz, ax, ay, az) — no covariance. So the C++ bridge change is purely mechanical: swap the message type and adjust field access from `twist.twist.twist.linear.x` to `twist.twist.linear.x`.

---

## Relevant Files

### Primary (twist propagation node)
- `src/twist_propagation/twist_propagation/twist_propagation_node.py` — main node (all 9 issues)

### Downstream consumers (message type changes)
- `src/grasp_preshaping/nodes/preshaping_service_bridge_node.cpp` — subscribes to `/hand_pose` (Pose→PoseStamped) and `/hand_twist` (TwistWithCovarianceStamped→TwistStamped)
- `src/grasp_preshaping/nodes/grasp_proximity_controller_node.py` — subscribes to `/hand_pose` (Pose→PoseStamped)

### Test
- `scripts/test_twist_propagation_integration.py` — publishes mock Pose, receives TwistWithCovarianceStamped

### Launch
- `src/prosthesis_launch/launch/mock.launch.py` — missing `mock_cloud` in LaunchDescription

---

## Implementation Plan

### Phase 1: Change /hand_pose to PoseStamped (3 files)

- [ ] **1.1** Update `twist_propagation_node.py` — change subscription from `Pose` to `PoseStamped` at line 263, update `_on_hand_pose` callback (line 332-340) to extract `msg.pose` and use `msg.header.stamp` for timestamps instead of `time.time()`. Store `frame_id` in the pose buffer tuple (add a 9th element or store separately as `self._hand_pose_frame`). Update the docstring at line 17. Update the import at line 56 (add `PoseStamped`, can remove `Pose` if no longer used directly).

- [ ] **1.2** Update `preshaping_service_bridge_node.cpp` — change subscription at line 101 from `geometry_msgs::msg::Pose` to `geometry_msgs::msg::PoseStamped`. Update the callback at lines 103-107 to store `msg->pose` (or store the full `PoseStamped` and extract `.pose` when building the FFI request at line 258). Update the member type at line 487 from `geometry_msgs::msg::Pose` to `geometry_msgs::msg::PoseStamped`. Add the `#include` for `geometry_msgs/msg/pose_stamped.hpp` if not already transitively included.

- [ ] **1.3** Update `grasp_proximity_controller_node.py` — change subscription at line 91-96 from `Pose` to `PoseStamped`. Update `_on_current_hand_pose` at line 137 to extract `msg.pose`. Update the type hint at line 55.

### Phase 2: Change /hand_twist to TwistStamped (2 files)

- [ ] **2.1** Update `twist_propagation_node.py` — change publisher at line 271-272 from `TwistWithCovarianceStamped` to `TwistStamped`. Update `_publish_twist` at lines 429-445 to construct a `TwistStamped` instead (no covariance, simpler message). Set `msg.header.frame_id` from the stored hand pose frame instead of hardcoded `"world"`. Update the import at line 58. Update the docstring at line 25.

- [ ] **2.2** Update `preshaping_service_bridge_node.cpp` — change subscription at line 109 from `geometry_msgs::msg::TwistWithCovarianceStamped` to `geometry_msgs::msg::TwistStamped`. Update the callback at lines 111-115. Update the member type at line 487. Update `try_handle_direct_request` at lines 238, 272-278: field access changes from `twist.twist.twist.linear.x` to `twist.twist.linear.x`. Add `#include` for `geometry_msgs/msg/twist_stamped.hpp`.

### Phase 3: Fix frame mismatch — transform pose to cloud frame before propagation (1 file)

- [ ] **3.1** Update `twist_propagation_node.py` `_cycle_callback` — after getting the latest pose at lines 607-608, call `_transform_pose_to_cloud_frame()` with the hand pose's `frame_id` and the position. This transforms the hand position from its source frame to the cloud frame before running propagation. The helper `_transform_pose_to_cloud_frame` already exists at line 497 but is currently never called. The fix is to actually invoke it with the correct frame_id from the PoseStamped header. Remove the comment at lines 610-613 that acknowledges the issue.

### Phase 4: Fix cloud age to use ROS time instead of wall clock (1 file)

- [ ] **4.1** Update `twist_propagation_node.py` `_cycle_callback` at lines 593-594 — replace `now = time.time()` with `now = self.get_clock().now().nanoseconds / 1e9` so that cloud age is computed consistently against the same time source used for cloud stamps. This ensures correctness when using ROS simulated time.

### Phase 5: Remove dead code — unused `_cloud_kdtree_stamp` (1 file)

- [ ] **5.1** Update `twist_propagation_node.py` — remove `self._cloud_kdtree_stamp` declaration at line 244. The KDTree invalidation is handled by setting `self._cloud_kdtree = None` at line 352, so the stamp field is never used.

### Phase 6: Fix lock contention — move `_call_preshaping_service` outside the lock (1 file)

- [ ] **6.1** Update `twist_propagation_node.py` `_cycle_callback` — refactor the WAITING_FOR_SEGMENTATION block (lines 554-575) to release the lock before calling `_call_preshaping_service()`. Currently the entire `_cycle_callback` holds `self._lock`, and `_call_preshaping_service` calls `wait_for_service(timeout_sec=1.0)` which blocks for up to 1 second while holding the lock, preventing all subscription callbacks from executing. The fix: extract the decision of whether to call preshaping under the lock, then release the lock before actually calling it. Similarly, move the `wait_for_service` check outside the lock in `_call_preshaping_service` itself (lines 655-660).

### Phase 7: Fix mock.launch.py — add mock_cloud to LaunchDescription (1 file)

- [ ] **7.1** Update `mock.launch.py` — add `mock_cloud` to the `LaunchDescription` return at line 109-119. The node is defined at lines 44-49 but never included. Add it after `config_arg` in the list.

### Phase 8: Update integration test for new message types (1 file)

- [ ] **8.1** Update `test_twist_propagation_integration.py` — change `_make_pose` helper (line 86) to return `PoseStamped` instead of `Pose`, with a frame_id of `"world"` and a timestamp. Change `pose_pub` at line 109 to publish `PoseStamped`. Change the twist subscriber at lines 120-122 from `TwistWithCovarianceStamped` to `TwistStamped`. Update `_twist_msgs` type hint at line 116. Update `latest_twist` property at line 215. Update the twist magnitude check at line 270 from `tw.twist.twist.linear` to `tw.twist.linear`.

---

## Verification Criteria

- [ ] `twist_propagation_node.py` subscribes to `PoseStamped` on `/hand_pose`, not `Pose`
- [ ] `twist_propagation_node.py` publishes `TwistStamped` on `/hand_twist`, not `TwistWithCovarianceStamped`
- [ ] The hand pose position is transformed to the cloud frame via TF2 before propagation in `_cycle_callback`
- [ ] Cloud age is computed using ROS clock time, not `time.time()`
- [ ] `_cloud_kdtree_stamp` is removed
- [ ] `_call_preshaping_service` does not hold the lock during `wait_for_service`
- [ ] `mock.launch.py` includes `mock_cloud` in the LaunchDescription
- [ ] `preshaping_service_bridge_node.cpp` compiles and subscribes to `PoseStamped` and `TwistStamped`
- [ ] `grasp_proximity_controller_node.py` subscribes to `PoseStamped` on `/hand_pose`
- [ ] Integration test uses `PoseStamped` for publishing and `TwistStamped` for receiving
- [ ] All Python files pass `py_compile` syntax check
- [ ] YAML config is valid

## Potential Risks and Mitigations

1. **External hand tracking system publishes `Pose`, not `PoseStamped`**
   Mitigation: The hand tracking system is external and its message type can be adapted. If it currently publishes `Pose`, a thin relay node can wrap it in `PoseStamped` with the correct frame_id. Alternatively, the twist propagation node could accept both via separate subscriptions, but this adds complexity for no real benefit.

2. **Preshaping bridge C++ change requires rebuild**
   Mitigation: The change is purely mechanical (swap message type, adjust field access paths). The FFI layer is unchanged. Risk is minimal — just a recompile.

3. **TF2 transform may not be available at startup**
   Mitigation: The existing `_transform_pose_to_cloud_frame` helper already falls back to raw coordinates on failure (line 524-528). The node logs a debug message and continues. This is acceptable behavior.

4. **Lock refactoring may introduce race conditions**
   Mitigation: The refactoring only moves the service call outside the lock. The state transition (`WAITING_FOR_SEGMENTATION` → `WAITING_FOR_PRESHAPING`) is set under the lock before releasing it. The `_on_preshaping_response` callback already acquires the lock separately. No new concurrent access patterns are introduced.

## Alternative Approaches

1. **Keep `TwistWithCovarianceStamped` and just fix the frame issue**: Simpler change (fewer files), but leaves the unused covariance overhead. The covariance is never consumed by anyone in the system.

2. **Add a `hand_pose_frame` parameter instead of using `PoseStamped`**: Avoids changing downstream consumers, but is fragile — the frame must be manually configured and could be wrong. Using `PoseStamped` is the ROS-idiomatic approach.

3. **Use `TwistWithCovarianceStamped` but drop covariance values to zero**: Minimal change to the preshaping bridge, but still carries the unnecessary 36-element array. Less clean than `TwistStamped`.
