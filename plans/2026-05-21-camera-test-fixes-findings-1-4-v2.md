# Camera-Test Performance Fixes — Findings 1-4

## Objective

Fix the four highest-impact issues causing TF frames to go missing and point clouds jumping around in RViz during `make camera-test`.

---

## Finding 1: TF Bridge — Static Transform Republished at 2 Hz via Timer

### Problem

`config/prosthesis_config.yaml:167` sets `publish_rate_hz: 2.0`. The bridge edge (`cam0 -> link`) is a **static hardware extrinsic** — it never changes. The timer exists only to handle a startup race condition (waiting for the bridge's own static chain to appear in the TF buffer) and as a workaround for CycloneDDS `/tf_static` latch unreliability.

At 2 Hz, point clouds arriving at 15-30 Hz from the Jetson are resolved against a TF that only updates every 500ms, causing visible jumping in RViz.

### Solution

Restructure the bridge to:
1. **Startup phase**: Use a timer at ~10 Hz to poll for the extrinsic. Once resolved, publish on `/tf_static` and cancel the startup timer.
2. **Liveness phase**: After startup, use a slow periodic timer (~0.2 Hz = once every 5 seconds) to re-publish the static chain on `/tf` as a safety net for late-joining nodes. Do NOT re-publish the nominal chain (20 transforms) on every tick — only the bridge edge (2 transforms).
3. **Remove the `publish_rate_hz` parameter** — replace with `startup_poll_rate_hz` (default 10) and `liveness_rate_hz` (default 0.2).

### Files to change

- [ ] `src/camera/camera/openvins_realsense_tf_bridge_node.py` — Refactor `_publish_all` into two phases (startup + liveness). Once the extrinsic is resolved for all cameras, cancel the startup timer and start the liveness timer. The liveness timer only re-publishes the bridge edge (2 transforms), not the full nominal chain (20 transforms).
- [ ] `config/prosthesis_config.yaml` — Replace `publish_rate_hz: 2.0` with `liveness_rate_hz: 0.2` (or remove the parameter entirely and use hardcoded defaults).

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

This is the **exact same executor-starvation bug** documented in the retrospective (Discovery 1) that was fixed in the fusion node and the TF bridge but never fixed here. A 500ms blocking call in the cycle callback freezes the single-threaded executor, preventing TF updates, cloud subscriptions, and pose callbacks from being processed. This cascades into stale data across the entire pipeline.

### Solution

Replace the blocking `transform()` call with a non-blocking `lookup_transform` using `Time()` (zero time = latest available), matching the pattern already used in the fusion node and bridge.

### Files to change

- [ ] `src/twist_propagation/twist_propagation/twist_propagation_node.py:779-813` — Rewrite `_transform_pose_to_cloud_frame` to use non-blocking `lookup_transform` with `Time()` instead of the blocking `transform()` with timeout. Apply the quaternion-to-rotation + translation manually (same pattern as `pointcloud_fusion_node.py:155-191`).

---

## Finding 3: Cloud Max Age Too High

### Problem

`config/prosthesis_config.yaml:145` — `cloud_max_age_s: 1.5`. The code default in `pointcloud_fusion_node.py:220` was `0.5`, but the config overrides it to `1.5`. A cloud up to 1.5 seconds old can be transformed with a TF that has moved significantly since the cloud was captured, causing visible offset/jumping as stale and fresh data are mixed.

### Solution

Revert to 0.5s. This is a config-only change.

### Files to change

- [ ] `config/prosthesis_config.yaml:145` — Change `cloud_max_age_s: 1.5` to `cloud_max_age_s: 0.5`

---

## Finding 4: Pointcloud Relay is an Unnecessary Copy Hop

### Problem

`src/camera/camera/pointcloud_relay_node.py` subscribes to `/fused_pointcloud` and republishes on `/segmentation/input_cloud`. This is an entire additional ROS node (with its own executor, subscriptions, publishers) doing nothing but `msg -> publish(msg)`. It adds latency, consumes DDS resources, and is one more node competing for executor time in the container.

### Solution

Remove the relay node from the pipeline and have downstream consumers subscribe directly to `/fused_pointcloud`. The segmentation bridge already has a configurable `input_cloud_topic` parameter — just point it to `/fused_pointcloud`.

### Files to change

- [ ] `src/prosthesis_launch/launch/pipeline.launch.py:197-203` — Remove the `pointcloud_relay_node` from the camera_nodes list in the launch file.
- [ ] `src/segmentation/segmentation/segmentation_ros2_node.py` — Update the default `input_cloud_topic` parameter from `/segmentation/input_cloud` to `/fused_pointcloud`.
- [ ] `config/prosthesis_config.yaml` — Add/update the segmentation bridge's `input_cloud_topic` parameter if it exists in the config, or add it to the segmentation section.
- [ ] Verify no other nodes subscribe to `/segmentation/input_cloud` — search the codebase for references and update them.

---

## Verification Criteria

- [ ] `ros2 topic hz /fused_pointcloud` shows consistent rate with no gaps
- [ ] `ros2 run tf2_ros tf2_echo marker_map head_d435i_head_depth_optical_frame` resolves immediately with no timeouts
- [ ] RViz displays the fused point cloud without visible jumping or flickering
- [ ] `ros2 node list` no longer shows `pointcloud_relay`
- [ ] `ros2 topic info /segmentation/input_cloud` shows no subscribers (topic should no longer exist)
- [ ] No new warnings in `ros2 topic echo /rosout` related to TF or stale data

---

## Potential Risks and Mitigations

1. **Risk: Bridge startup race — extrinsic not available immediately**
   Mitigation: The startup timer polls at 10 Hz, so the extrinsic is resolved within 100ms of the static chain being available. This is faster than the current 2 Hz approach.

2. **Risk: Late-joining nodes miss the `/tf_static` latch**
   Mitigation: The liveness timer re-publishes the bridge edge on `/tf` every 5 seconds. This is a reasonable trade-off between traffic and reliability.

3. **Risk: Removing the relay breaks other launch files that depend on `/segmentation/input_cloud`**
   Mitigation: Search all launch files and node source for references to `/segmentation/input_cloud` and update them. The segmentation bridge is the only consumer.

4. **Risk: Reducing cloud_max_age_s causes gaps if network latency spikes**
   Mitigation: Monitor fusion node stats. If cam1_only count increases, bump to 0.75s.

---

## Alternative Approaches

1. **For Finding 1**: Instead of refactoring the bridge into two phases, simply increase the rate to 30 Hz and accept the wasted bandwidth. This is the quickest fix but sends 600 unnecessary transform messages per minute.

2. **For Finding 4**: Keep the relay node but make it a simple topic remap in the launch file instead of a full Python node. This is less code but still adds a DDS hop.
