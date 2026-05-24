# TF Chain Reliability Fix

## Objective

Fix the root causes of point cloud transform jumping and TF chain disconnection by making the host self-sufficient for the critical dynamic TF edges (`marker_map -> *_imu`), rather than depending on Jetson `/tf` delivery over DDS. This addresses both the frame-to-frame jitter (jumping) and the prolonged cold-start blackout (~80s of zero output).

## Root Cause (Definitive)

The dynamic TF chain `marker_map -> *_imu -> *_cam0` is published by OpenVINS on the **Jetson** via the `/tf` topic with **VOLATILE** durability. The host must receive these transforms over the Ethernet DDS link. This creates three failure modes:

1. **DDS discovery delay**: If the host starts before the Jetson's DDS is discovered, all `/tf` messages are missed. The host sees `"marker_map" does not exist` until discovery completes. This explains the 80s cold start in v5/v6.
2. **Transient DDS drops**: Even after discovery, CycloneDDS over unicast-only Ethernet can drop `/tf` messages. The host's TF buffer has no entry for `marker_map -> *_imu`, causing `"not part of the same tree"` errors.
3. **OpenVINS tracking loss**: When OpenVINS loses tracking (no marker visible), it stops publishing both odom and TF. The host has no fallback.

Meanwhile, the OpenVINS odometry messages (`/ov_msckf/odomimu` and `/ov_msckf_arm/odomimu`) arrive at ~200 Hz via a **separate topic** with BEST_EFFORT QoS. The `odom_to_pose_relay` already subscribes to `/ov_msckf_arm/odomimu` and publishes PoseStamped — but it does **not** publish TF.

**The fix**: Subscribe to both OpenVINS odom topics on the host and publish the `marker_map -> *_imu` TF transforms directly from the odometry data. This bypasses the DDS `/tf` delivery entirely for the most critical dynamic edge.

## Implementation Plan

### Phase 1: Host-Side Odometry-to-TF Relay

- [ ] **Create a new node `openvins_odom_tf_relay` in the `camera` package.** This node subscribes to both `/ov_msckf/odomimu` (head) and `/ov_msckf_arm/odomimu` (arm) and publishes the `marker_map -> {head,arm}_imu` TF transforms at odom rate (~200 Hz). Rationale: The odom message's `header.frame_id` is `"marker_map"` and `child_frame_id` is the IMU frame (`"head_imu"` or `"arm_imu"`), and the pose is `T(marker_map -> imu)`. This is exactly the TF transform we need. By publishing it locally on the host, we bypass the unreliable DDS `/tf` delivery from the Jetson.

- [ ] **Use BEST_EFFORT QoS for the odom subscriptions**, matching OpenVINS's publish QoS (as already done in `odom_to_pose_relay.py:47-51`). Rationale: QoS compatibility is required for DDS delivery. OpenVINS publishes odom with BEST_EFFORT; the subscriber must match.

- [ ] **Publish the TF transforms via `TransformBroadcaster` on `/tf`** with the host clock timestamp. Rationale: Using the host clock avoids the clock-skew issue that causes "extrapolation into the past" errors when the Jetson clock and host clock differ. The TF2 buffer on the host will see fresh, locally-timestamped transforms.

- [ ] **Handle the `imu -> cam0` extrinsic.** OpenVINS publishes `imu -> cam0` as a separate TF edge (controlled by its `publish_calibration_tf` parameter). This is a **static** extrinsic (the IMU-to-camera rigid mount). Two options:
  - **Option A (recommended)**: Add the `imu -> cam0` extrinsic as a parameter to the new node, loaded from the known D435i calibration values. Publish it as a static TF. This makes the host fully self-sufficient for `marker_map -> imu -> cam0`.
  - **Option B**: Continue relying on the Jetson's `/tf` for this edge, since it's static and the liveness mechanism can cover it.
  Rationale: Option A eliminates the last dependency on Jetson `/tf` for the dynamic chain. The `imu -> cam0` extrinsic is a hardware property that doesn't change.

- [ ] **Add configuration parameters:** `head_odom_topic` (default `/ov_msckf/odomimu`), `arm_odom_topic` (default `/ov_msckf_arm/odomimu`), `publish_imu_to_cam_tf` (bool, default True), `imu_to_cam_extrinsics` (loaded from config). Rationale: The extrinsics are hardware-specific and should be configurable.

### Phase 2: Increase Bridge Liveness Rate

- [ ] **Increase `liveness_rate_hz` from 2.0 to 10.0 in `config/prosthesis_config.yaml:174`.** Rationale: The bridge re-sends the static `cam0 -> link` edges. At 10 Hz, the TF buffer always has a fresh entry. Near-zero cost (2 small messages per tick). This complements Phase 1 by ensuring the bridge edges are always available even if the initial `/tf_static` latch was missed.

- [ ] **Also re-send the nominal static chain in the liveness tick** (currently only the 2 bridge edges are re-sent). Add an option to include the 20 nominal D435i static transforms in the liveness re-send. Rationale: If CycloneDDS `/tf_static` is unreliable, re-sending the full static chain on `/tf` at a low rate (e.g., 1 Hz) ensures late-joining nodes receive all transforms. This is belt-and-suspenders with the existing nominal static chain publish.

### Phase 3: Bbox Reporting Fixes

- [ ] **Fix `bbox_skipped` counter semantics** in `pointcloud_fusion_node.py`. Move the `bbox_skipped` increment from the top of `_bbox_fallback()` to after the cache check fails. Rationale: Currently, even successful cache hits are counted as "skipped," making diagnostics misleading.

- [ ] **Include cache hits in `_bbox_successes`** for the `_check_bbox_health()` metric. Rationale: Cache hits are functionally successful removals. Excluding them makes the health check report worse than reality.

- [ ] **Add a throttled INFO log for successful bbox removals** (every 50th success). Rationale: Currently only failures are logged, creating the impression that bbox removal never works even when it does.

### Phase 4: Fusion Node Resilience

- [ ] **Add a "last known good" transform cache per camera** in the fusion node. When the TF lookup at `pointcloud_fusion_node.py:422-427` succeeds, cache the resulting (R, t_vec) per frame_id. When it fails, attempt to use the cached transform if it's less than `transform_max_age_s` old (default 1.0s). Rationale: Even with Phase 1, there will be brief TF gaps (e.g., OpenVINS tracking loss, DDS hiccups). Serving a 200ms-stale transform is better than dropping the frame entirely.

- [ ] **Add `transform_max_age_s` parameter** (default 1.0). When the cached transform exceeds this age, log a warning and skip that camera's cloud for this cycle. Rationale: Very stale transforms could place the cloud far from reality. The age limit prevents serving garbage.

- [ ] **Publish a diagnostic counter `tf_cache_fallbacks`** in the stats line. Rationale: Allows monitoring how often the system is operating in degraded mode.

## What This Fixes

| Problem | Before | After Phase 1 | After Phase 4 |
|---------|--------|---------------|---------------|
| 80s cold start (no TF) | Zero output until DDS discovery | Immediate TF from odom (~200 Hz) | N/A |
| OpenVINS TF jitter | Raw Jetson `/tf` at varying rate | Host-published TF at steady ~200 Hz | Smoothed via last-good cache |
| "Extrapolation into past" | Jetson clock stamps vs host clock | Host clock stamps | N/A |
| Intermittent TF drops | Frame dropped entirely | Frame dropped | Cached transform used |
| Bbox removal 0% | No diagnostics on success | N/A | Accurate success reporting |

## Verification Criteria

### Phase 1 Verification (Odometry-to-TF Relay)

- [ ] **V1.1: Cold start produces output within 5 seconds.** Start the host pipeline (`make camera-test`), then start the Jetson. The first fused pointcloud should publish within 5 seconds of the Jetson's odom topics becoming available (measured by `last_publish_ago` in stats). Previously this took 80+ seconds.

- [ ] **V1.2: `marker_map` frame exists immediately when odom is flowing.** Run `ros2 topic echo /tf --once | grep marker_map` on the host. It should show a `marker_map -> *_imu` transform published by `openvins_odom_tf_relay`, not by the Jetson.

- [ ] **V1.3: TF chain resolves end-to-end.** Run `ros2 run tf2_ros tf2_echo marker_map arm_d435i_arm_depth_optical_frame`. It should resolve without "not part of the same tree" errors once both odom topics are active.

- [ ] **V1.4: No "extrapolation into the past" errors.** Run the pipeline for 60 seconds and grep the log for "extrapolation". Zero occurrences expected, since the host now publishes TF with its own clock.

- [ ] **V1.5: Odometry rate is ~200 Hz.** Run `ros2 topic hz /ov_msckf_arm/odomimu` and verify the relay is processing at the same rate. Check the relay's throttled log output for message count.

### Phase 2 Verification (Liveness Rate)

- [ ] **V2.1: Bridge edges available at 10 Hz.** Run `ros2 topic hz /tf` and filter for `cam0 -> *_link` transforms. Should show ~10 Hz (was 2 Hz).

- [ ] **V2.2: No TF gaps during normal operation.** Run the pipeline for 120 seconds. The `tf_fail` counter in the stats line should be zero or near-zero (was hundreds per interval in v5/v6).

### Phase 3 Verification (Bbox Reporting)

- [ ] **V3.1: `bbox_skipped` only counts actual skips.** When bbox removal works via cache, `bbox_skipped` should NOT be incremented. Verify by checking stats during a period where bbox TF is available via cache but not fresh lookup.

- [ ] **V3.2: Health check includes cache hits.** When bbox removal works only via cache, the health check should report >0% success rate (was 0% even with cache hits).

- [ ] **V3.3: Successful removals are logged.** During normal operation with working bbox TF, verify that occasional "bbox removed N points" INFO messages appear.

### Phase 4 Verification (Fusion Resilience)

- [ ] **V4.1: Cached transform used during brief TF gaps.** Simulate a TF gap (e.g., briefly block the Jetson Ethernet). The fusion node should log "using cached transform" and continue publishing. The `tf_cache_fallbacks` counter should increment.

- [ ] **V4.2: Stale cache triggers skip, not garbage.** After a prolonged TF gap (>1s), the fusion node should log a warning and skip that camera's cloud, not publish a badly placed cloud.

- [ ] **V4.3: End-to-end latency unchanged.** The cached transform lookup adds ~0.1ms (dictionary lookup + age check). Measure via stats `last_publish_ago` — should remain at ~0.0-0.1s.

### Integration Verification

- [ ] **IV.1: Full camera-test run shows stable output.** Run `make camera-test` for 5 minutes. Verify:
  - `published > 0` within 5s of Jetson startup
  - `tf_fail` counts near zero
  - `bbox_removed > 0` when TF is healthy
  - No "not part of the same tree" errors after initial startup
  - No "extrapolation into the past" errors

- [ ] **IV.2: RViz shows no visible jumping.** Observe the fused pointcloud in RViz during normal operation. The cloud should be stable with no visible frame-to-frame displacement.

## Potential Risks and Mitigations

1. **Odom-to-TF relay duplicates Jetson `/tf` transforms**
   Mitigation: TF2 handles duplicate transforms gracefully — the latest timestamp wins. Since the relay uses the host clock (newer than Jetson clock), its transforms will be preferred. No conflict.

2. **The `imu -> cam0` extrinsic may differ between OpenVINS config and D435i factory calibration**
   Mitigation: Use the same values that OpenVINS uses. These are set in the Jetson's OpenVINS config file. Verify by comparing the relay's published `imu -> cam0` transform against what OpenVINS publishes on the Jetson (visible via `ros2 run tf2_ros tf2_echo head_imu head_cam0`).

3. **200 Hz TF publishing adds CPU load**
   Mitigation: Each TF message is ~100 bytes. At 200 Hz for 2 cameras, that's ~40 KB/s — negligible. The `TransformBroadcaster` is highly optimized in ROS2.

4. **BEST_EFFORT QoS means some odom messages are dropped**
   Mitigation: At 200 Hz, losing a few messages is fine — the next one arrives 5ms later. The TF buffer will have a slightly stale transform for those 5ms, which is well within the 15 Hz fusion rate.

5. **The relay creates a dependency on odom topics being available**
   Mitigation: The relay should be a **supplement**, not a replacement. If odom is not available (Jetson not running), the existing Jetson `/tf` path still works. The relay simply provides a faster, more reliable path when odom IS available. The bridge node's liveness mechanism remains as fallback.

## Alternative Approaches

1. **Fix DDS configuration instead**: Tune CycloneDDS reliability settings, increase discovery timeouts, enable multicast. Trade-off: would fix the root DDS issue but requires Jetson-side changes and may not work on WSL2 + direct Ethernet.

2. **Host-side EMA smoothing only** (prior plan Phase 1): Add quaternion slerp smoothing to the fusion node. Trade-off: simpler but treats the symptom, not the cause. The TF chain is still unreliable.

3. **Increase OpenVINS bridge rate on Jetson**: Modify the Jetson-side OpenVINS to publish TF more aggressively. Trade-off: requires changes to the separate Jetson repo, and doesn't help with DDS discovery delays.

4. **NTP/PTP clock sync instead of Makefile timesync**: Use proper clock synchronization instead of the one-shot SSH-based sync. Trade-off: more robust but adds infrastructure complexity. The odom-to-TF relay eliminates the clock issue entirely by re-stamping with host clock.

## Files Modified

| File | Change |
|------|--------|
| `src/camera/camera/openvins_odom_tf_relay.py` | **New file** — the odom-to-TF relay node |
| `src/camera/setup.py` | Add new entry point for the relay node |
| `config/prosthesis_config.yaml` | Add relay parameters, increase liveness rate |
| `src/prosthesis_launch/launch/pipeline.launch.py` | Add relay node to launch |
| `src/pointcloud_fusion/pointcloud_fusion/pointcloud_fusion_node.py` | Add transform cache, fix bbox reporting |
