# Camera-Test Performance Fixes — Findings 1-4

## Objective

Fix the four highest-impact issues causing TF frames to go missing and point clouds jumping around in RViz during `make camera-test`.

---

## Finding 1: TF Bridge — Static Transform Needlessly Republished via Timer

### Problem

`config/prosthesis_config.yaml:167` sets `publish_rate_hz: 2.0`. The bridge edge (`cam0 -> link`) is a **static hardware extrinsic** — it's the fixed geometric relationship between the color camera and the RealSense body (D435i factory calibration). It never changes.

**Why does the timer exist at all?** Two reasons:

1. **Startup race condition**: The bridge needs to look up `T(color_optical -> link)` from its own static chain (`openvins_realsense_tf_bridge_node.py:332`). But `/tf_static` latching in CycloneDDS can be unreliable — the bridge might start before its own static chain has propagated through the TF buffer. So it uses a timer to keep retrying.

2. **`/tf_static` latch workaround**: The code at lines 377-380 publishes on both `/tf_static` AND `/tf` (dynamic). The comment says "for nodes with stale `/tf_static` caches." This is a workaround for CycloneDDS late-joiner issues.

**Why this causes jumping**: At 2 Hz, the bridge only re-stamps and sends the transform every 500ms. Point clouds arriving at 15-30 Hz from the Jetson are resolved against a TF that only "updates" every 500ms. Since the TF is actually static, this shouldn't matter in theory — but in practice, the TF buffer's time-based expiry and the way RViz interpolates means the low publication rate creates perceptible jitter.

### Solution

Refactor the bridge into two phases:

1. **Startup phase** (~10 Hz poll): Keep retrying until the extrinsic is resolved. Once resolved for all cameras, publish on `/tf_static` and cancel the startup timer.

2. **Liveness phase** (~0.2 Hz = once every 5 seconds): Periodically re-publish ONLY the bridge edge (2 transforms) on `/tf` as a safety net for late-joining nodes. Do NOT re-publish the full nominal static chain (20 transforms) on every tick.

This means the bridge effectively becomes a "publish once, occasionally refresh" node instead of a continuous 2 Hz broadcaster.

### Files to change

- [ ] **`src/camera/camera/openvins_realsense_tf_bridge_node.py`** — Refactor `_publish_all` into `_startup_publish` (called at ~10 Hz until all extrinsics resolved) and `_liveness_publish` (called at ~0.2 Hz after startup). Cancel startup timer once all specs are resolved. Liveness timer only sends the bridge edge transforms (2), not the full nominal chain (20). The nominal chain is published once at startup via `/tf_static` (already done in `_build_and_send_nominal_static_chain`).

- [ ] **`config/prosthesis_config.yaml`** — Remove or update `publish_rate_hz: 2.0` to `liveness_rate_hz: 0.2`.

---

## Finding 2: Twist Propagation — 500ms Blocking TF Timeout

### Problem

`src/twist_propagation/twist_propagation/twist_propagation_node.py:803-806`:
```python
transformed = self._tf_buffer.transform(
    ps, self._cloud_frame,
    timeout=rclpy.duration.Duration(seconds=0.5),
)
```

This is the **exact same executor-starvation bug** documented in the retrospective (Discovery 1) that was fixed in the fusion node and the TF bridge but never fixed here. A 500ms blocking call in the cycle callback freezes the single-threaded executor, preventing TF updates, cloud subscriptions, and pose callbacks from being processed.

Note: The same bug also exists in `segmentation_ros2_node.py:155` (same 500ms timeout), but that node is less critical for the camera-test scenario.

### Solution

Replace the blocking `transform()` call with a non-blocking `lookup_transform` using `Time()` (zero time = latest available), matching the pattern already used in `pointcloud_fusion_node.py:155-191`. Apply the quaternion rotation + translation manually.

### Files to change

- [ ] **`src/twist_propagation/twist_propagation/twist_propagation_node.py:779-813`** — Rewrite `_transform_pose_to_cloud_frame` to:
  1. Check if `frame_id == cloud_frame` (early return, no TF needed)
  2. Use `self._tf_buffer.lookup_transform(cloud_frame, frame_id, Time())` (non-blocking, raises on failure)
  3. Apply the transform manually: extract translation + rotation from the TF message, apply to the point
  4. On exception, return the original coordinates (matching current fallback behavior)

---

## Finding 3: Cloud Max Age Too High

### Problem

`config/prosthesis_config.yaml:145` — `cloud_max_age_s: 1.5`. The code default in `pointcloud_fusion_node.py:220` is `0.5`, but the config overrides it to `1.5`. A cloud up to 1.5 seconds old can be transformed with a TF that has moved significantly since the cloud was captured, causing visible offset/jumping as stale and fresh data are mixed.

### Solution

Revert to 0.5s. Config-only change.

### Files to change

- [ ] **`config/prosthesis_config.yaml:145`** — Change `cloud_max_age_s: 1.5` to `cloud_max_age_s: 0.5`

---

## Finding 4: Pointcloud Relay is an Unnecessary Copy Hop

### Problem

`src/camera/camera/pointcloud_relay_node.py` subscribes to `/fused_pointcloud` and republishes on `/segmentation/input_cloud`. This is an entire additional ROS node (with its own executor, subscriptions, publishers) doing nothing but `msg -> publish(msg)`.

**Who subscribes to `/segmentation/input_cloud`?** Only one node:
- `src/segmentation/segmentation_bridge/segmentation_ros2_node.py:128` — the segmentation bridge

**Who else references the relay?**
- `prosthesis_launch/launch/pipeline.launch.py:197-203` — launches the relay node
- `prosthesis_launch/launch/twist_propagation_test.launch.py:102-110` — launches the relay node
- `prosthesis_launch/launch/grasp_test.launch.py:109` — uses a remapping instead of the relay (already bypasses it!)

The `grasp_test.launch.py` already demonstrates the right approach: it uses a `remappings` argument to point the segmentation node directly at the cloud topic. The relay is redundant.

### Solution

Remove the relay node from all launch files and have the segmentation bridge subscribe directly to `/fused_pointcloud`.

### Files to change

- [ ] **`src/prosthesis_launch/launch/pipeline.launch.py:197-203`** — Remove the `pointcloud_relay_node` from the `camera_nodes` list. Add a `remappings` argument to the segmentation_bridge node: `remappings={("/segmentation/input_cloud", "/fused_pointcloud")}`.

- [ ] **`src/prosthesis_launch/launch/twist_propagation_test.launch.py:102-110`** — Remove the relay node. Add the same remapping to the segmentation_bridge node in this launch file (if it has one — verify).

- [ ] **`src/segmentation/segmentation_bridge/segmentation_ros2_node.py:128`** — Change the default subscription topic from `"/segmentation/input_cloud"` to `"/fused_pointcloud"`. This makes the node work correctly even without the remapping.

- [ ] **`config/prosthesis_config.yaml`** — Update the `topics.segmentation_input_cloud` reference from `/segmentation/input_cloud` to `/fused_pointcloud` (if it exists in the flat reference section).

---

## Verification Criteria

- [ ] `ros2 topic hz /fused_pointcloud` shows consistent rate with no gaps
- [ ] `ros2 run tf2_ros tf2_echo marker_map head_d435i_head_depth_optical_frame` resolves immediately with no timeouts
- [ ] RViz displays the fused point cloud without visible jumping or flickering
- [ ] `ros2 node list` no longer shows `pointcloud_relay` when running camera-test
- [ ] `ros2 topic list` no longer shows `/segmentation/input_cloud` when running camera-test
- [ ] No new warnings in `ros2 topic echo /rosout` related to TF or stale data
- [ ] Bridge logs "publishing ... from RealSense extrinsic" once at startup, then goes quiet (no repeated logging)

---

## Potential Risks and Mitigations

1. **Risk: Bridge startup race — extrinsic not available immediately after refactor**
   Mitigation: The startup timer polls at 10 Hz, so the extrinsic is resolved within 100ms of the static chain being available. This is faster than the current 2 Hz approach.

2. **Risk: Late-joining nodes miss the `/tf_static` latch**
   Mitigation: The liveness timer re-publishes the bridge edge on `/tf` every 5 seconds. This covers the CycloneDDS late-joiner case without flooding the network.

3. **Risk: Removing the relay breaks `twist_propagation_test.launch.py`**
   Mitigation: Update that launch file too, using the same remapping approach. Verify by checking all references to `/segmentation/input_cloud`.

4. **Risk: Reducing cloud_max_age_s to 0.5 causes gaps if network latency spikes**
   Mitigation: Monitor fusion node stats. If `cam1_only` count increases significantly, bump to 0.75s as a compromise.

---

## Alternative Approaches

1. **For Finding 1 (simplest fix)**: Instead of the two-phase refactor, just increase the rate to 30 Hz and accept the wasted bandwidth. This is the quickest fix but sends ~600 unnecessary transform messages per minute and re-broadcasts the full 20-transform nominal chain at 30 Hz. Not recommended — the two-phase refactor is cleaner.

2. **For Finding 4 (minimal change)**: Instead of removing the relay, just add a `remappings` to the segmentation bridge in the launch file, pointing it directly to `/fused_pointcloud`. Leave the relay running but effectively unused. This avoids touching multiple launch files but leaves dead code running.
