# Pointcloud Fusion Node — TF Timing Investigation & Fix Plan

## Objective

Diagnose and resolve the TF timing issues preventing the arm camera point cloud from being fused. The arm camera cloud (`arm_d435i_arm_depth_optical_frame`) consistently fails to transform to `marker_map`, resulting in only head-camera-only output despite both cameras streaming.

---

## Root Cause Analysis

### Evidence from Logs

1. **TF bridge resolves arm via fallback** (`openvins_realsense_tf_bridge_node.py:343-346`):
   ```
   arm: publishing arm_cam0->arm_d435i_arm_link from fallback assuming arm_cam0 == arm_d435i_arm_depth_optical_frame
   ```
   This means the OpenVINS `arm_cam0` frame was found, but the anchor frame `arm_d435i_arm_color_optical_frame_body_display` was NOT found. The bridge fell back to assuming `arm_cam0` is identity-equivalent to `arm_d435i_arm_depth_optical_frame`, which is almost certainly wrong — OpenVINS camera frame and RealSense depth optical frame have different orientations/positions.

2. **Fusion node TF failures are 100% arm** (`pointcloud_fusion_node.py:351-375`):
   ```
   tf_fail={arm_d435i_arm_depth_optical_frame:10}
   ```
   Every single TF failure is for the arm optical frame. The head camera transforms fine.

3. **RViz confirms timestamp issue**:
   ```
   Message Filter dropping message: frame 'arm_d435i_arm_depth_frame' at time 1779293284.474
   for reason 'the timestamp on the message is earlier than all the data in the transform cache'
   ```
   This is the smoking gun: the arm camera's TF data has timestamps that lag behind the point cloud timestamps.

4. **Stats show no progress** — identical numbers across all 10-second intervals:
   ```
   published=22 (dual=8, cam1_only=14) dist_removed=1051475 bbox_removed=0 tf_fail={arm_d435i_arm_depth_optical_frame:10}
   ```
   The `dual=8` and `cam1_only=14` never change, meaning after the initial burst, no new arm clouds are being fused. The `tf_fail` count stays at 10 (the initial warm-up failures) and doesn't grow because the timer-based merge stops trying after the arm cloud's age exceeds `cloud_max_age_s=0.5`.

### Identified Problems (Prioritized)

**P1 (Critical): Fusion node uses `rclpy.time.Time()` (time=0) for TF lookups instead of cloud timestamps**

At `pointcloud_fusion_node.py:354-359`, the fusion node calls:
```python
rclpy.time.Time()  # This is time zero, not the latest time
```
for `can_transform` and `lookup_transform`. While this gets the "latest available" transform, it means the fusion node ignores the actual cloud timestamp entirely. The TF2 buffer uses time-zero semantics which should return the latest transform, BUT the transform must actually exist in the buffer at that point.

The real issue is that the arm TF chain has a **clock domain mismatch** or **startup race condition**:
- The Jetson publishes point clouds with RealSense device timestamps
- The TF bridge publishes transforms with `self.get_clock().now()` (ROS system time)
- If the Jetson's clock and the host's clock are not synchronized, the point cloud timestamps may fall outside the TF buffer's time window

**P2 (High): TF bridge arm fallback is geometrically incorrect**

The arm bridge falls back to `arm_cam0 == arm_d435i_arm_depth_optical_frame` (`openvins_realsense_tf_bridge_node.py:394-404`). This is an identity assumption between two frames that have different optical conventions. OpenVINS `arm_cam0` is the OpenVINS camera coordinate frame, while `arm_d435i_arm_depth_optical_frame` uses RealSense optical conventions (ROS optical: X-right, Y-down, Z-forward). These are NOT the same frame, and this fallback produces an incorrect transform.

The head camera works because its anchor frame `head_d435i_head_color_optical_frame_body_display` IS available in the TF tree (from OpenVINS on the Jetson), so it uses the correct path. The arm's anchor frame is never published by OpenVINS.

**P3 (Medium): Fusion node uses time-zero for distance filter and bbox removal TF lookups**

At `pointcloud_fusion_node.py:484-485` and `pointcloud_fusion_node.py:443-445`, the distance filter and bbox removal also use `rclpy.time.Time()` which, while generally fine for static transforms, will fail if the arm frame's TF chain is incomplete.

**P4 (Medium): Timer-based merge uses wall-clock age, not cloud header age**

At `pointcloud_fusion_node.py:312-320`, the freshness check uses `self.get_clock().now()` vs arrival time, not the cloud's own header timestamp. If clouds arrive with old timestamps (e.g., network delay from Jetson), they'll be considered "fresh" by wall clock but their TF may not be available.

---

## Implementation Plan

### Phase 1: Fix TF Timestamp Synchronization

- [ ] **1.1** In `pointcloud_fusion_node.py`, change `_process_clouds` to use each cloud's `header.stamp` for TF lookups instead of `rclpy.time.Time()`. Specifically, at lines 354-359, replace `rclpy.time.Time()` with `rclpy.time.Time.from_msg(cloud.header.stamp)`. This ensures the TF lookup uses the correct timestamp that matches when the point cloud was captured, which is the timestamp the TF tree should have data for.
  - **Rationale**: Using time-zero means "get latest," but if the arm camera's TF data has a different clock domain, the latest may not cover the cloud's capture time. Using the cloud's own timestamp makes the lookup deterministic and correct.

- [ ] **1.2** Add a `rclpy.duration.Duration(seconds=0.5)` timeout to the `can_transform` call in `_process_clouds` (line 355). Currently it uses 0.05s which may be too short for the arm camera TF chain which involves a multi-hop relay from the Jetson.
  - **Rationale**: The arm TF chain is: `marker_map -> arm_cam0 -> arm_d435i_arm_link -> arm_d435i_arm_depth_frame -> arm_d435i_arm_depth_optical_frame`. This multi-hop chain may take slightly longer to become available.

- [ ] **1.3** Similarly update the distance filter TF lookup at `_get_frame_origin_in_target` (line 484) and the bbox removal TF lookup at `_transform_points_to_frame` (line 443) to accept and use the cloud's timestamp instead of `rclpy.time.Time()`.
  - **Rationale**: Consistency — all TF lookups should use the same timestamp source.

### Phase 2: Fix TF Bridge Arm Fallback

- [ ] **2.1** In `openvins_realsense_tf_bridge_node.py`, investigate why the arm anchor frame `arm_d435i_arm_color_optical_frame_body_display` is not available. This frame is published by OpenVINS running on the Jetson. Check if the arm OpenVINS instance is configured to publish `body_display` frames.
  - **Rationale**: The head works because its `body_display` frame is available. The arm's is not. This is the root cause of the incorrect fallback.

- [ ] **2.2** If the arm OpenVINS does not publish `body_display` frames, change the arm's `anchor_frame_mode` from `"link"` to `"optical"` in the config. This tells the bridge to compose the transform as `parent_to_anchor @ optical_to_link` instead of just using `parent_to_anchor` directly, which is geometrically more correct when the anchor frame is an optical frame.
  - **Rationale**: With `anchor_frame_mode="link"`, the bridge assumes the anchor frame IS the link frame (identity transform between them). With `"optical"`, it properly chains the optical-to-link extrinsic. If the arm's anchor frame is an optical frame, the mode must be `"optical"`.

- [ ] **2.3** As a safety improvement, add a validation check in `_resolve_bridge_transform` that logs a warning when the fallback is activated, clearly indicating the assumed identity transform is approximate and may cause fusion failures.
  - **Rationale**: The current log message `fallback assuming arm_cam0 == arm_d435i_arm_depth_optical_frame` is easy to miss. A more prominent warning would help debugging.

### Phase 3: Improve Fusion Node Robustness

- [ ] **3.1** Add a TF timestamp diagnostics log in `_process_clouds` that, when a TF lookup fails, logs both the cloud timestamp and the latest available timestamp in the TF buffer for that frame pair. This helps diagnose clock skew issues.
  - **Rationale**: The current error message just says "TF not available" with no timing context. Adding timestamp info makes debugging much faster.

- [ ] **3.2** Consider adding a `tf_timeout_s` parameter to the fusion node (default 0.1s) and use it consistently for all TF lookups. The current 0.05s timeout may be marginal.
  - **Rationale**: A configurable timeout lets operators tune for their network latency without code changes.

- [ ] **3.3** In `_timer_merge`, consider also checking the cloud's header stamp age (not just the arrival-time age) to detect stale clouds that arrived late.
  - **Rationale**: A cloud could arrive "fresh" by wall clock but have a header timestamp that is seconds old, meaning its TF data may have been evicted from the buffer.

### Phase 4: Investigate Jetson-Host Clock Synchronization

- [ ] **4.1** Verify that the Jetson and the host machine have synchronized clocks (NTP/PTP). The RViz error "timestamp on the message is earlier than all the data in the transform cache" strongly suggests clock skew between the Jetson (publishing point clouds) and the host (running the TF bridge and fusion node).
  - **Rationale**: The Jetson publishes point clouds with its own clock. The TF bridge publishes transforms with the host clock. If clocks differ by more than the TF buffer window, lookups will fail.

- [ ] **4.2** If clock sync cannot be guaranteed, consider using `use_sim_time=true` with a shared time source, or have the fusion node strip and re-stamp cloud headers with the host clock upon receipt.
  - **Rationale**: This is a common pattern in multi-machine ROS2 deployments where PTP/NTP is unreliable.

---

## Verification Criteria

- [ ] `tf_fail` in fusion stats shows 0 (or near-0) for `arm_d435i_arm_depth_optical_frame` after initial warm-up
- [ ] Fusion stats show `dual=` count increasing steadily (not stuck at 8)
- [ ] No more RViz "timestamp earlier than all data in transform cache" warnings
- [ ] Both head and arm camera point clouds appear in the fused output in RViz
- [ ] The arm TF bridge resolves via its anchor frame (not fallback) or the fallback produces geometrically correct results

## Potential Risks and Mitigations

1. **Changing TF lookup from time-zero to cloud timestamp may break things further if clocks are skewed**
   Mitigation: First verify/fix clock synchronization (Phase 4) before changing TF lookup timestamps. Alternatively, add a parameter to choose between "latest" and "cloud stamp" modes.

2. **The arm OpenVINS may genuinely not support body_display frames**
   Mitigation: Use the `anchor_frame_mode="optical"` path which chains the optical-to-link extrinsic properly, or configure the arm OpenVINS to publish the body_display frame.

3. **Increasing TF timeout may add latency to the fusion pipeline**
   Mitigation: The timeout only applies when the transform is not immediately available. If the transform is in the buffer, the lookup is instant. The timeout is a worst-case wait.

## Alternative Approaches

1. **Use `tf2_ros.Buffer` with a larger cache duration**: Increase the TF buffer cache time (default is usually 10s) to accommodate larger clock skews. This is a band-aid but may help if the skew is small.
   - Trade-off: Uses more memory, doesn't fix the root cause.

2. **Header re-stamping**: Upon receiving a cloud from the Jetson, immediately re-stamp its header with the host clock time. Then all TF lookups use host time consistently.
   - Trade-off: Loses the original capture timestamp, introduces a small timing error proportional to network latency. Simple to implement.

3. **Message_filters ApproximateTimeSynchronizer with `require_both=True`**: Force the fusion node to wait for both clouds before processing, ensuring temporal alignment.
   - Trade-off: Reduces output rate to the slower camera's rate. Won't help if TF lookups still fail.
