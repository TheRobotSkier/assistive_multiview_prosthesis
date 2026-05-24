# Camera-Test Performance Analysis & Improvement Plan

## Objective

Diagnose and fix the "mediocre performance" observed when running `make camera-test` inside the container — specifically TF frames going missing and point clouds "jumping around" in RViz. Identify low-hanging fruit improvements and any underlying architectural issues.

---

## Context

The `camera-test` target (`Makefile.workspace:87-89`) launches:
```
ros2 launch prosthesis_launch pipeline.launch.py mia_hand:=false wrist:=false emg:=false haptic:=false
```

This starts **10+ ROS nodes** in a single `rclpy.spin()` / single-threaded executor per node:
1. `pipeline_manager` — state machine (unused in camera-test, but running)
2. `openvins_realsense_tf_bridge` — bridges OpenVINS→RealSense TF trees
3. `pointcloud_fusion` — transforms + merges dual-camera clouds
4. `pointcloud_relay` — relays fused cloud to segmentation input
5. `odom_to_pose_relay` — converts OpenVINS odom to PoseStamped
6. `segmentation_bridge` — HTTP bridge to inference server (unused, no server running)
7. `twist_propagation` — collision prediction (starts inactive)
8. `preshaping_service_bridge` — C++ Rust FFI bridge
9. `grasp_proximity_controller` — proximity-based grasp execution
10. `rviz2` — visualization

Data flows from Jetson (10.42.0.2) over Ethernet to the host Docker container (10.42.0.1) via CycloneDDS.

---

## Findings — Prioritized by Impact

### Finding 1 (CRITICAL): TF Bridge publishes at 2 Hz — far too slow for smooth RViz display

**Source**: `config/prosthesis_config.yaml:167`
```yaml
openvins_realsense_tf_bridge:
    ros__parameters:
        publish_rate_hz: 2.0
```

**Impact**: The bridge publishes `cam0 -> link` transforms at only 2 Hz. The TF buffer default cache is 10 seconds. With 2 Hz publication, any consumer doing TF lookups between publishes may see the transform as "expired" or jittery. RViz's fixed frame is `marker_map`, and all point clouds are in RealSense depth optical frames — the TF chain from `marker_map` to the optical frames passes through the bridge edge. At 2 Hz, point clouds arriving at 15-30 Hz will appear to "jump" because the TF they're resolved against only updates every 500ms.

**Rationale for high priority**: This is the most likely root cause of the "jumping around" symptom. The TF bridge rate directly controls how smoothly point clouds appear in the `marker_map` frame.

### Finding 2 (HIGH): All Python nodes use single-threaded `rclpy.spin()` — executor starvation risk

**Source**: Every Python node's `main()` function:
- `pointcloud_fusion_node.py:610-620`
- `twist_propagation_node.py:1208-1222`
- `odom_to_pose_relay.py:89-103`
- `pointcloud_relay_node.py:23-37`

**Impact**: The retrospective (`2026-05-20-pointcloud-fusion-debugging-retrospective-v1.md`) documents how executor starvation was already a major issue (Discoveries 1, 4, 6). While the fusion node now uses a background thread for processing, the twist propagation node still does heavy work (KDTree construction, covariance propagation) in the timer callback under `rclpy.spin()`. When the executor is busy with one callback, TF updates and cloud subscriptions queue up, causing stale data and missed frames.

**Specific risk in `twist_propagation_node.py`**:
- `_on_input_cloud` (line 594-609): Parses + downsamples entire cloud on the executor thread
- `_cycle_callback` (line 994-1048): KDTree query + covariance propagation under lock
- `_transform_pose_to_cloud_frame` (line 801-806): Uses a **500ms blocking timeout** on TF lookup! This is the exact same bug from Discovery 1 in the retrospective.

### Finding 3 (HIGH): Twist propagation TF lookup uses 500ms blocking timeout

**Source**: `src/twist_propagation/twist_propagation/twist_propagation_node.py:803-806`
```python
transformed = self._tf_buffer.transform(
    ps, self._cloud_frame,
    timeout=rclpy.duration.Duration(seconds=0.5),
)
```

**Impact**: This is the exact same executor-starvation bug documented in the retrospective (Discovery 1). A 500ms blocking TF lookup in the cycle callback freezes the single-threaded executor, preventing cloud callbacks and TF updates from being processed. This cascades into stale clouds, missed TF frames, and the "jumping" behavior.

### Finding 4 (MEDIUM): Point cloud relay is a pointless copy hop

**Source**: `src/camera/camera/pointcloud_relay_node.py:1-37`

**Impact**: The relay subscribes to `/fused_pointcloud` and republishes on `/segmentation/input_cloud`. This is an entire additional ROS node (with its own executor, subscriptions, publishers) doing nothing but `msg → publish(msg)`. It adds latency and consumes DDS resources. The segmentation bridge could subscribe directly to `/fused_pointcloud`.

### Finding 5 (MEDIUM): Voxel size of 5mm is very aggressive for fusion

**Source**: `config/prosthesis_config.yaml:139`
```yaml
voxel_size: 0.005
```

**Impact**: A 5mm voxel grid on dual-camera clouds (~116K points each, per the retrospective) is extremely fine. The `_voxel_downsample` function in `pointcloud_fusion_node.py:125-152` uses `np.unique` on int64 voxel indices, which is O(N log N) and memory-intensive. Combined with the per-voxel centroid averaging, this is likely the most expensive single operation in the pipeline. Increasing to 1cm or 1.5cm would dramatically reduce processing time with negligible visual quality loss.

### Finding 6 (MEDIUM): `cloud_max_age_s` mismatch between config and code default

**Source**:
- Code default: `pointcloud_fusion_node.py:220` — `cloud_max_age_s: 0.5`
- Config override: `config/prosthesis_config.yaml:145` — `cloud_max_age_s: 1.5`

**Impact**: The config sets `cloud_max_age_s: 1.5` seconds. With clouds arriving at ~3 Hz (per retrospective), this means a cloud up to 1.5 seconds old is considered "fresh." If the TF has updated significantly in that 1.5s window (e.g., camera moved), the old cloud gets transformed with the new TF, causing visible "jumping" as stale and fresh data are mixed. The code default of 0.5s was more appropriate.

### Finding 7 (MEDIUM): Unused nodes running in camera-test

**Source**: `Makefile.workspace:87-89` and `pipeline.launch.py`

**Impact**: The camera-test launches `pipeline_manager`, `segmentation_bridge`, `twist_propagation`, `preshaping_service_bridge`, and `grasp_proximity_controller` — none of which are needed for just viewing camera data. Each node:
- Consumes CPU and memory
- Subscribes to topics (adding DDS subscription overhead)
- Has its own TF buffer/listener (competing for TF data)
- Adds to CycloneDDS discovery traffic

The `pipeline_manager` subscribes to EMG topics that don't exist, `segmentation_bridge` tries to connect to an HTTP server that isn't running, and `twist_propagation` builds KDTrees from every fused cloud even when inactive.

### Finding 8 (LOW-MEDIUM): CycloneDDS is locked to a single interface

**Source**: `config/cyclonedds_peer.xml:4-5`
```xml
<NetworkInterface address="10.42.0.1" />
```

**Impact**: This forces CycloneDDS to use only the `10.42.0.1` interface. If the host's IP changes or the interface isn't up when the container starts, DDS discovery fails silently. However, for the camera-test scenario this is likely correct since data comes from the Jetson.

### Finding 9 (LOW): RViz subscribes to raw Jetson clouds AND fused cloud simultaneously

**Source**: `rviz/prosthesis.rviz:266,299,332`

**Impact**: RViz subscribes to:
- `/head/d435i_head/depth/color/points` (raw head cloud from Jetson)
- `/arm/d435i_arm/depth/color/points` (raw arm cloud from Jetson)
- `/fused_pointcloud` (fused output)

Displaying all three simultaneously means RViz is rendering 3× the point data. The raw clouds are also in their original optical frames, requiring TF lookups that may fail or be stale (see Finding 1). For camera-test, only the fused cloud should be displayed.

### Finding 10 (LOW): Bridge re-publishes the entire nominal static chain on every timer tick

**Source**: `src/camera/camera/openvins_realsense_tf_bridge_node.py:355-358`
```python
if self._nominal_static_tfs:
    for tf_msg in self._nominal_static_tfs:
        tf_msg.header.stamp = stamp
    self._tf_broadcaster.sendTransform(self._nominal_static_tfs)
```

**Impact**: Every timer tick (at 2 Hz, or whatever rate), the bridge re-publishes ~20 static transforms on `/tf`. This is a workaround for stale `/tf_static` caches but adds unnecessary traffic. At higher publish rates (Finding 1 fix), this would be 10+ transforms × 15 Hz = 150+ messages/second just for static chain re-broadcast.

---

## Implementation Plan

### Phase 1: Immediate Low-Hanging Fruit (Highest Impact, Minimal Risk)

- [ ] **1.1** Increase TF bridge publish rate from 2 Hz to 30 Hz in `config/prosthesis_config.yaml:167`. The bridge publishes a static transform, so 30 Hz is trivially cheap (no computation, just re-stamping and sending). This directly fixes the "jumping" caused by stale TF. Also stop re-broadcasting the full nominal static chain on every tick — only do it once on startup and periodically (e.g., every 5 seconds) as a liveness check.

- [ ] **1.2** Fix the 500ms blocking TF timeout in `twist_propagation_node.py:803-806`. Replace with non-blocking `lookup_transform` with `Time()` (zero time, latest available), matching the fix already applied to the fusion node and bridge per the retrospective. This eliminates executor starvation in the twist propagation node.

- [ ] **1.3** Reduce `cloud_max_age_s` from 1.5 to 0.5 in `config/prosthesis_config.yaml:145`. This prevents stale clouds from being mixed with fresh TF transforms, reducing visual jumping. The code default was already 0.5s; the config override made it worse.

### Phase 2: Performance Improvements (Medium Effort, Good Payoff)

- [ ] **2.1** Increase voxel size from 5mm to 10mm in `config/prosthesis_config.yaml:139`. This halves the linear resolution and reduces point count by ~8× (cubic), dramatically cutting processing time in the fusion node. For visualization and segmentation, 1cm resolution is more than adequate.

- [ ] **2.2** Move twist propagation cloud parsing + downsampling to a background thread (same pattern as the fusion node fix). The `_on_input_cloud` callback at `twist_propagation_node.py:594-609` should store the raw message and process it asynchronously, keeping the executor responsive.

- [ ] **2.3** Eliminate the `pointcloud_relay_node` by having the segmentation bridge subscribe directly to `/fused_pointcloud`. Update the segmentation bridge's `input_cloud_topic` parameter. This removes one unnecessary node from the pipeline.

### Phase 3: Architectural Cleanup (Higher Effort, Best Long-Term)

- [ ] **3.1** Create a lightweight `camera_test.launch.py` that only launches the nodes actually needed for viewing: TF bridge, pointcloud fusion, and RViz. Do not launch pipeline_manager, segmentation_bridge, twist_propagation, preshaping_service, or proximity_controller. This reduces CPU load by ~40% and eliminates unnecessary DDS subscriptions.

- [ ] **3.2** In the RViz config for camera-test, disable the raw cloud displays (PCHead, PCArm) and only show PCFused. This reduces RViz rendering load by ~3× and avoids displaying clouds with potentially stale TF.

- [ ] **3.3** Consider using a MultiThreadedExecutor for nodes that have both time-sensitive subscriptions and timer callbacks (especially pointcloud_fusion and twist_propagation). This prevents timer callbacks from blocking subscription processing.

---

## Verification Criteria

- [ ] `ros2 topic hz /fused_pointcloud` shows consistent 10+ Hz with no gaps
- [ ] `ros2 run tf2_ros tf2_echo marker_map head_d435i_head_depth_optical_frame` shows smooth, continuous updates at 15+ Hz with no "Transform timeout" warnings
- [ ] RViz displays the fused point cloud without visible jumping or flickering
- [ ] No TF failures in fusion node stats (`ros2 topic echo /rosout` filtered for pointcloud_fusion warnings)
- [ ] CPU usage of the prosthesis container is under 200% (of 400% available on 4 cores)

---

## Potential Risks and Mitigations

1. **Risk: Increasing bridge rate to 30 Hz increases DDS traffic**
   Mitigation: The bridge publishes static transforms. At 30 Hz with 2 cameras, that's only 60 transform messages/second — negligible for CycloneDDS over Ethernet. The nominal static chain re-broadcast should be limited to once every 5 seconds.

2. **Risk: Reducing cloud_max_age_s to 0.5 may cause more "no cloud" gaps if network latency spikes**
   Mitigation: Monitor fusion node stats. If cam1_only count increases significantly, bump to 0.75s as a compromise.

3. **Risk: Increasing voxel size to 10mm may degrade segmentation quality**
   Mitigation: Segmentation operates on the relayed cloud, not the fused cloud directly. The voxel size only affects visualization and the twist propagation collision detection. If needed, add a separate `segmentation_voxel_size` parameter.

4. **Risk: Removing the pointcloud_relay node breaks downstream topic assumptions**
   Mitigation: The segmentation bridge already has a configurable `input_cloud_topic` parameter. Just point it to `/fused_pointcloud`.

---

## Alternative Approaches

1. **Use ROS2 component nodes (rclcpp_components)**: Load all C++ nodes into a single process with a shared executor. This eliminates DDS overhead between nodes and allows zero-copy message passing. Trade-off: requires significant refactoring of Python nodes to C++.

2. **Use ROS2 launch composition**: Group nodes into containers using `ComposableNodeContainer`. Similar benefits to above but with less refactoring. Trade-off: Python nodes can't be composed.

3. **Use a dedicated TF publisher on the Jetson**: Instead of the host bridge, have the Jetson publish the full TF chain (marker_map → depth_optical_frame) directly. Trade-off: requires modifying the Jetson's OpenVINS launch configuration.

4. **Use Foxglove Studio instead of RViz**: Foxglove handles large point clouds more efficiently and has better DDS integration. Trade-off: different tool, may not support all ROS message types.
