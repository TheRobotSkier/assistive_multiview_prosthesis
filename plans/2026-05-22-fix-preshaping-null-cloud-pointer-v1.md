# Fix: PointCloud Data Pointer is Null in Preshaping Bridge

## Objective

Fix the race condition where the preshaping service is called with a null/stale `cloud.data` pointer, causing nearly every preshaping call to fail with "PointCloud data pointer is null". The root cause is a timing mismatch: the twist propagation node detects that a *new* segmented cloud timestamp has appeared and calls the preshaping service, but the preshaping bridge's cloud subscriber has not yet received the new cloud message — it still holds the previous (potentially empty/reset) cloud.

## Root Cause Analysis

### The Race Condition (Step-by-step)

1. **Click → Segmentation**: twist_propagation publishes a click, transitions to `WAITING_FOR_SEGMENTATION`, and records `_seg_cloud_stamp_at_trigger`.

2. **Reset publishes empty cloud**: The segmentation node receives a reset message (published by twist_propagation for each new object) and publishes an **empty** `PointCloud2` on `/segmentation/object_cloud` to clear RViz2 (see `segmentation_ros2_node.py:224-227`).

3. **Twist propagation's `_on_segmented_cloud` fires**: The empty cloud's timestamp updates `_seg_cloud_stamp` — but the preshaping bridge's `cloud_sub_` callback **also** receives this empty cloud and sets `latest_cloud_` to an empty message with `data.size() == 0`.

4. **Inference completes — real cloud published**: Segmentation finishes and publishes the real segmented cloud with points on `/segmentation/object_cloud`.

5. **Twist propagation detects stamp change**: In the next timer cycle, `_seg_cloud_stamp > _seg_cloud_stamp_at_trigger` is true, so it logs "Segmented cloud received, calling preshaping service" and calls `compute_grasp`.

6. **Preshaping bridge uses stale/empty cloud**: The preshaping bridge's `try_handle_direct_request()` copies `latest_cloud_` under the mutex. If the real segmented cloud hasn't arrived yet (or if it's still the empty reset cloud), `cloud.data.data()` returns a null pointer and `cloud.data.size()` is 0. The Rust FFI `pointcloud_view_to_pointcloud()` checks `view.data_ptr.is_null()` and returns the error "PointCloud data pointer is null".

### Evidence from Log (camera-test-log-v3.txt)

The timing is visible at lines 135-149:
- `515.329` — twist_propagation: "Segmented cloud received, calling preshaping service"
- `515.331` — preshaping_service: "Planner called with 1 camera(s)"
- `515.342` — twist_propagation: "Preshaping failed: PointCloud data pointer is null"
- `515.630` — segmentation_bridge: "Published segmented cloud: 5363/18422 points."

The segmentation bridge publishes the real cloud **~300ms after** preshaping was already called. The preshaping bridge received the earlier empty/reset cloud, not the real one.

### Why It Happens Nearly Every Time

The segmentation node's reset publishes an empty cloud with a *new timestamp*, which immediately updates `_seg_cloud_stamp` in twist_propagation. Since the comparison is `_seg_cloud_stamp > _seg_cloud_stamp_at_trigger`, the empty cloud from the reset satisfies the condition and triggers preshaping prematurely.

## Implementation Plan

### Phase 1: Fix the Segmented Cloud Detection in Twist Propagation (Primary Fix)

- [ ] **1.1** Modify `_on_segmented_cloud` in `twist_propagation_node.py` to verify the cloud is non-empty before updating `_seg_cloud_stamp`. Currently at `src/twist_propagation/twist_propagation/twist_propagation_node.py:700-703`, the callback only records the timestamp. Add a check: `if msg.width * msg.height == 0: return` (skip empty clouds like the reset-empty ones). This prevents the empty reset cloud from being treated as a valid segmentation result.

- [ ] **1.2** Optionally store the latest segmented cloud message (or at least its point count) in `_on_segmented_cloud` so the timer cycle can make a more informed decision. Store `self._seg_cloud_point_count = msg.width * msg.height` alongside the stamp for diagnostics.

### Phase 2: Add Cloud Validity Guard in Preshaping Bridge (Defense in Depth)

- [ ] **2.1** In `preshaping_service_bridge_node.cpp:239-264`, add a validation after copying the cloud: check that `cloud.data.size() > 0` and `cloud.width * cloud.height > 0`. If the cloud is empty, return a descriptive error message like "Point cloud is empty (0 points)" instead of letting it reach the FFI layer where it becomes a confusing "data pointer is null" error. This makes the failure mode clearer and prevents UB if a null pointer were ever dereferenced.

- [ ] **2.2** Add a cloud freshness check in the preshaping bridge. Store the cloud's `header.stamp` alongside `latest_cloud_` and compare it against the current time. If the cloud is older than a configurable threshold (e.g., 2 seconds), return an error like "Point cloud is stale (X seconds old)". This catches the case where the bridge has received *some* cloud but it's from a previous segmentation cycle.

### Phase 3: Fix the Segmentation Reset Race (Root Cause Elimination)

- [ ] **3.1** In `segmentation_ros2_node.py:214-227`, change the reset callback to NOT publish an empty cloud on `/segmentation/object_cloud`. Instead, only clear internal clicks. The empty cloud publication is meant to clear the RViz2 display, but it creates the race condition. The RViz2 display can be cleared by other means (e.g., a dedicated "clear display" topic, or simply letting the new segmented cloud overwrite the old one).

- [ ] **3.2** If removing the empty cloud publication is undesirable (RViz2 may show stale data), add a dedicated topic like `/segmentation/clear_display` that only the RViz2 display subscribes to, or use the existing `/segmentation/object_cloud_snapshot` from the cloud_snapshot_node to manage display state. The key is to not publish on `/segmentation/object_cloud` during reset.

### Phase 4: Add Retry/Wait Logic for Preshaping Service Call

- [ ] **4.1** In `twist_propagation_node.py`, after detecting a new segmented cloud stamp, add a short delay (e.g., 100-200ms) or a spin cycle before calling the preshaping service. This gives the preshaping bridge time to receive the same cloud message via its own subscription. Implement this as a one-shot timer that fires after a configurable `preshaping_call_delay_s` parameter.

- [ ] **4.2** Alternatively, instead of a fixed delay, have twist_propagation verify that the preshaping bridge has received the cloud by checking a shared topic or adding a health-check mechanism. However, this is more complex; the delay approach in 4.1 is simpler and sufficient.

## Verification Criteria

- [ ] **V1**: No more "PointCloud data pointer is null" errors in the camera-test log when segmentation produces a valid non-empty cloud.
- [ ] **V2**: Preshaping service receives a cloud with `data.size() > 0` and `width * height > 0` on every call after segmentation completes.
- [ ] **V3**: The segmentation reset (empty cloud on `/segmentation/object_cloud`) does NOT trigger a premature preshaping call.
- [ ] **V4**: Existing integration tests pass: `python3 -m pytest src/twist_propagation/test/test_twist_propagation.py -v` and `scripts/test_twist_propagation_integration.py`.
- [ ] **V5**: RViz2 display still updates correctly after segmentation (if Phase 3 changes are applied).

## Potential Risks and Mitigations

1. **Removing the empty cloud publication breaks RViz2 display clearing**
   - Mitigation: Test RViz2 display behavior after removing the empty cloud publication. If stale segments remain visible, use the cloud_snapshot_node or a dedicated clear-display topic as an alternative.

2. **Adding a delay before preshaping introduces latency in the grasp pipeline**
   - Mitigation: Keep the delay small (100-200ms). The segmentation inference already takes 50-300ms, so this is negligible. The delay can be configured via a parameter and set to 0 if not needed.

3. **The empty-cloud check in `_on_segmented_cloud` might skip legitimate zero-point segmentation results**
   - Mitigation: A zero-point segmentation result is not useful for preshaping anyway (no object to grasp). Skipping it is correct behavior. The twist_propagation node will time out and return to IDLE, which is appropriate.

4. **Phase 2's freshness check might reject valid clouds in slow TF scenarios**
   - Mitigation: Use a generous staleness threshold (2-5 seconds) and make it configurable. The primary fix (Phase 1) should prevent the stale-cloud case from occurring in the first place.

5. **Race condition could still occur if network/message delivery order is non-deterministic**
   - Mitigation: The defense-in-depth approach (all four phases) makes this extremely unlikely. Phase 1 prevents the empty-cloud trigger, Phase 2 catches it at the bridge, Phase 3 removes the root cause, and Phase 4 adds a timing buffer.

## Alternative Approaches

1. **Use a ROS2 service instead of topic for segmented cloud delivery**: Replace the `/segmentation/object_cloud` topic with a service that returns the segmented cloud. This guarantees the caller receives the cloud directly. However, this is a major architectural change affecting multiple nodes (segmentation_bridge, cloud_snapshot_node, twist_propagation, pipeline_manager) and breaks the decoupled pub/sub design.

2. **Pass the segmented cloud inline in the preshaping service request**: Instead of having the preshaping bridge subscribe to the cloud topic separately, modify the `compute_grasp` service to accept a `PointCloud2` as part of the request. The caller (twist_propagation or pipeline_manager) would include the cloud it already received. This eliminates the synchronization issue entirely. However, it requires changing the service type from `std_srvs/srv/Trigger` to a custom service type, which is a larger refactor.

3. **Use message_filters::TimeSynchronizer or ApproximateTimeSynchronizer**: Synchronize the cloud subscription in the preshaping bridge with a timestamp from the service call. This is complex and overengineered for this use case.

4. **Recommended approach**: Combine Phases 1 and 2 (minimum viable fix). Phase 1 (skip empty clouds in `_on_segmented_cloud`) is the most impactful single change — it directly prevents the race condition. Phase 2 (validate cloud in the bridge) provides defense in depth with minimal code change. Phases 3 and 4 are optional improvements.
