# Pointcloud Fusion Node — Revised TF Timing Diagnosis & Fix Plan

## Objective

Diagnose and resolve the TF timing issues preventing reliable pointcloud fusion from both cameras. The fusion node only publishes single-camera clouds intermittently, with both head and arm optical frames failing TF lookups despite the TF bridge running.

---

## Revised Root Cause Analysis

### New Evidence from Extended Logs

The second log batch reveals **three critical new findings** that change the diagnosis:

#### Finding 1: BOTH cameras fail TF, not just the arm

```
tf_fail={arm_d435i_arm_depth_optical_frame:250, head_d435i_head_depth_optical_frame:254}
```

The head camera has **254 TF failures** — nearly identical to the arm's 250. This means the problem is NOT specific to the arm fallback. It's a **systemic timing issue** affecting both cameras equally.

#### Finding 2: Stats are completely frozen

```
published=100 (dual=21, cam1_only=79) dist_removed=1289010 bbox_removed=0
```

These numbers are identical across every 10-second interval. The node published 100 clouds in the first ~70 seconds, then **stopped publishing entirely**. No new clouds are being produced at all. The `dual=21` and `cam1_only=79` are all from the initial startup period.

#### Finding 3: Arm bridge source oscillates between anchor and fallback

```
arm: publishing arm_cam0->arm_d435i_arm_link from fallback assuming arm_cam0 == arm_d435i_arm_depth_optical_frame
arm: publishing arm_cam0->arm_d435i_arm_link from anchor arm_d435i_arm_color_optical_frame_body_display as link
arm: publishing arm_cam0->arm_d435i_arm_link from fallback assuming arm_cam0 == arm_d435i_arm_depth_optical_frame
arm: publishing arm_cam0->arm_d435i_arm_link from anchor arm_d435i_arm_color_optical_frame_body_display as link
```

The arm anchor frame (`arm_d435i_arm_color_optical_frame_body_display`) appears and disappears intermittently. This is expected — it's published by the Jetson's aruco marker detection node and only exists when the marker is visible. The head anchor frame (`head_d435i_head_color_optical_frame_body_display`) has the same behavior but the head bridge resolves consistently (we never see it flip to fallback in the logs).

#### Finding 4: RViz confirms timestamp gap

```
Message Filter dropping message: frame 'arm_d435i_arm_depth_frame' at time 1779293854.474
for reason 'the timestamp on the message is earlier than all the data in the transform cache'
```

The point cloud's timestamp is **older** than the oldest transform in the TF cache. This is the classic symptom of **clock skew between machines**.

#### Finding 5: Missing `head_d435i_head_color_optical_frame_raw` in RViz

The user reports no TF between `head_d435i_head_color_optical_frame_raw` and `marker_map`. From `report.md:57`, this frame is:
```
marker_0 -> head_d435i_head_color_optical_frame_raw  (Jetson aruco_marker_pose, dynamic — only when marker visible)
```
This is an aruco marker detection frame — it only exists when the marker is visible. This is expected behavior, not a bug, but it confirms the aruco detection is intermittent.

### Root Cause: Clock Skew + Time-Zero TF Lookups

The architecture is:
- **Jetson (10.42.0.2)**: Runs OpenVINS, RealSense drivers, publishes point clouds and TF with **Jetson clock**
- **Host (10.42.0.1)**: Runs the fusion node and TF bridge, uses **host clock**

The TF bridge at `openvins_realsense_tf_bridge_node.py:339` stamps all transforms with `self.get_clock().now()` (host clock). Point clouds arrive from the Jetson with Jetson-clock timestamps in their headers.

**The fusion node at `pointcloud_fusion_node.py:354` uses `rclpy.time.Time()` (time zero) for TF lookups.** In TF2, time zero means "get the latest available transform." This works when:
1. The transform exists in the buffer (it was published)
2. The buffer hasn't expired it due to age

**The default `tf2_ros.Buffer()` cache duration is ~10 seconds.** Here's the critical sequence:

1. Bridge publishes transforms with host clock time T_host
2. Jetson publishes point clouds with Jetson clock time T_jetson
3. Fusion node receives point cloud with header stamp T_jetson
4. Fusion node does `can_transform(target, source, Time(), timeout=0.05s)` — asks for "latest"
5. If `T_host` and `T_jetson` are close enough, the latest transform covers the cloud's timestamp → works
6. If clocks drift apart by more than the buffer cache duration, or if the DDS delivery is delayed, the "latest" transform may not cover the cloud's time → fails

The frozen stats suggest that after initial startup, either:
- The TF buffer fills with host-timestamped transforms that are too far from the Jetson timestamps
- Or the point clouds stop arriving (DDS issue)

Given the RViz "timestamp earlier than all data" error, **clock skew is the primary suspect**.

### Why the Stats Freeze (Not Just Slow Down)

The `_timer_merge` at `pointcloud_fusion_node.py:322-339` checks cloud freshness using `self.get_clock().now()` vs the **arrival time** (wall clock). If clouds keep arriving, they'd be "fresh" and the timer would keep trying to process them. But the stats freeze at 100, meaning either:

1. **Clouds stopped arriving** — DDS delivery from the Jetson stopped (network issue, CycloneDDS peer discovery failure)
2. **The timer keeps running but `_process_clouds` always hits the TF failure path and returns early** — but then `published` wouldn't increment, and we'd see `tf_fail` growing. The tf_fail is frozen too, which means the timer isn't even calling `_process_clouds` anymore.

Wait — looking more carefully at the stats, `tf_fail` is also frozen at `{arm:250, head:254}`. The `_log_stats` method at line 532-544 does NOT reset stats — it's cumulative. So these are the totals from the entire run. If the timer were still running and failing, tf_fail would grow. Since it's frozen, **the timer callback is either not running, or both clouds have `age > cloud_max_age_s=0.5`**.

The most likely explanation: **the point clouds stopped arriving from the Jetson**. After the initial burst, CycloneDDS peer communication became unreliable. This is consistent with the report's note about DDS being "flaky in practice."

---

## Implementation Plan

### Phase 1: Diagnose Clock Skew (Do First)

- [ ] **1.1** Run a clock comparison test between Jetson and host. On both machines, run `date +%s%N` simultaneously and compare. If the difference is >100ms, clock skew is confirmed.
  - **Rationale**: The RViz "timestamp earlier than all data" error directly indicates clock skew. This must be quantified before deciding on a fix strategy.

- [ ] **1.2** While the pipeline is running, check if point clouds are still arriving:
  ```bash
  ros2 topic hz /head/d435i_head/depth/color/points
  ros2 topic hz /arm/d435i_arm/depth/color/points
  ```
  If these show 0 Hz, the DDS link has dropped and the frozen stats are due to no incoming data, not TF failures.
  - **Rationale**: Differentiates between "no data" and "data but TF fails." The fix strategy differs significantly.

- [ ] **1.3** Check TF availability at runtime:
  ```bash
  ros2 run tf2_ros tf2_echo marker_map head_d435i_head_depth_optical_frame
  ros2 run tf2_ros tf2_echo marker_map arm_d435i_arm_depth_optical_frame
  ```
  If these fail or show large timestamp gaps, the TF chain is broken.
  - **Rationale**: Confirms whether the TF chain is complete from the fusion node's perspective.

### Phase 2: Fix Clock Synchronization (Critical Path)

- [ ] **2.1** Install and configure chrony (NTP daemon) on both the Jetson and the host. Add the host as a peer for the Jetson and vice versa. This ensures both machines converge to the same clock.
  - **Rationale**: Clock skew is the root cause of the timestamp mismatch. NTP can typically keep machines within 1ms of each other on a LAN.

- [ ] **2.2** Alternatively, if NTP is not feasible, configure the Jetson's RealSense node to use `use_sim_time:=false` and ensure both machines use `system_time`. Verify that `ROS_DOMAIN_ID` and DDS config are identical on both sides.
  - **Rationale**: Eliminates any ROS2-level time source discrepancies.

### Phase 3: Fix Fusion Node TF Lookups

- [ ] **3.1** In `pointcloud_fusion_node.py` `_process_clouds` (lines 352-360), change the TF lookup to use the cloud's header timestamp instead of `rclpy.time.Time()`:
  - Replace `rclpy.time.Time()` with `rclpy.time.Time.from_msg(cloud.header.stamp)` in both `can_transform` and `lookup_transform` calls
  - **Rationale**: Using time-zero ("latest") is a workaround that works when clocks are synced. Using the cloud's actual timestamp is correct regardless of clock sync — TF2 will find the transform that was valid at the cloud's capture time. This is the proper ROS2 pattern.

- [ ] **3.2** Increase the TF buffer cache duration. At line 241, change `tf2_ros.Buffer()` to `tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=30.0))` or similar.
  - **Rationale**: The default Buffer cache is ~10s. If the Jetson's clock is behind the host's, the cloud timestamps could be "in the past" from the host's perspective. A larger cache gives more room for clock skew before transforms expire. This is a safety net, not the primary fix.

- [ ] **3.3** Increase the `can_transform` timeout from 0.05s to 0.2s at line 355. The multi-hop TF chain across two machines needs more tolerance.
  - **Rationale**: 50ms is very tight for a TF chain that spans: `marker_map -> *_cam0 -> *_link -> *_depth_frame -> *_depth_optical_frame`, with the first two hops coming from the Jetson over WiFi/Ethernet.

- [ ] **3.4** Pass the cloud's timestamp to `_get_frame_origin_in_target` (line 424) and `_transform_points_to_frame` (line 443) instead of using `rclpy.time.Time()`. Update both method signatures to accept a timestamp parameter.
  - **Rationale**: Consistency — all TF lookups in the pipeline should use the same timestamp source.

### Phase 4: Fix Fusion Node Freshness Detection

- [ ] **4.1** In `_cb_cam1` and `_cb_cam2` (lines 312-320), also store the cloud's header timestamp alongside the arrival wall-clock time. In `_timer_merge`, check freshness using BOTH:
  - Wall-clock age (current behavior: arrival time vs now)
  - Header stamp age (cloud's own timestamp vs now, to detect stale clouds from a stopped publisher)
  - **Rationale**: A cloud could arrive "fresh" by wall clock but have a header timestamp that is seconds old. The TF buffer may have already expired the transform for that old timestamp. Dual-checking catches this.

- [ ] **4.2** Clear the stored cloud reference after processing in `_timer_merge` to avoid re-processing the same cloud. Currently, the same cloud message can be processed multiple times across timer ticks as long as it's within the age window. After successful processing, set `self._cam1_cloud = None` / `self._cam2_cloud = None`.
  - **Rationale**: Prevents the stats from being misleading (same cloud counted multiple times) and ensures the node always tries to get fresh data.

### Phase 5: Fix Stats to Be Per-Interval (Diagnostic Improvement)

- [ ] **5.1** In `_log_stats` (lines 532-544), reset the stats counters after logging. Alternatively, track per-interval deltas. The current cumulative stats make it impossible to tell if the node is currently working or stalled.
  - **Rationale**: Frozen cumulative stats look the same as "working but no new data." Per-interval stats would immediately show when the node stops producing output.

- [ ] **5.2** Add a "last published ago" metric to the stats log showing how many seconds since the last successful publish. This makes stalls immediately obvious.
  - **Rationale**: A single number ("last published 45s ago") is more actionable than comparing cumulative counters.

### Phase 6: Fix TF Bridge Robustness

- [ ] **6.1** In `openvins_realsense_tf_bridge_node.py` `_publish_all` (lines 343-346), the code mutates `self._nominal_static_tfs` in-place by setting `tf_msg.header.stamp = stamp`. This means the list's transforms are constantly being mutated. While this works, it's fragile — if `sendTransform` internally references the objects later, there could be issues. Create fresh copies or at least document the mutation.
  - **Rationale**: Defensive coding. The in-place mutation is a subtle bug risk.

- [ ] **6.2** Add a latching mechanism to the bridge: when the anchor frame resolves successfully, cache the resulting matrix. If the anchor frame subsequently disappears (marker not visible), continue using the last-known-good matrix instead of falling back to the incorrect identity assumption. Log a warning that the transform is stale.
  - **Rationale**: The arm anchor frame oscillates between available and unavailable. Each time it disappears, the bridge falls back to the incorrect `arm_cam0 == arm_d435i_arm_depth_optical_frame` assumption, causing the TF chain to jump between correct and incorrect transforms. Holding the last good transform provides stability.

- [ ] **6.3** Investigate why the head anchor frame (`head_d435i_head_color_optical_frame_body_display`) is consistently available while the arm's is intermittent. Check if the head marker is more visible, or if the arm OpenVINS has a different configuration.
  - **Rationale**: If the arm's OpenVINS can be configured to be as reliable as the head's, the oscillation problem goes away.

### Phase 7: DDS Reliability (If Clouds Stop Arriving)

- [ ] **7.1** If `ros2 topic hz` (from Phase 1) shows clouds stopping, investigate CycloneDDS reliability. Consider adding `<MaxSamples>` and `<HistoryDepth>` tuning to `cyclonedds_peer.xml`, or switching to Fast-RTPS for comparison.
  - **Rationale**: The report explicitly notes that CycloneDDS peer-mode across Ethernet is "flaky in practice." If data delivery is unreliable, no amount of TF fix will help.

- [ ] **7.2** Add a heartbeat/health-check topic from the Jetson that the host monitors. If the heartbeat stops, log an explicit error rather than silently stalling.
  - **Rationale**: Silent failures are the hardest to debug. A heartbeat makes DDS failures immediately visible.

---

## Verification Criteria

- [ ] `ros2 topic hz /fused_pointcloud` shows consistent ~15 Hz output
- [ ] Fusion stats show `dual=` count growing steadily (both cameras contributing)
- [ ] `tf_fail` counts remain at 0 or near-0 after initial warm-up
- [ ] No RViz "timestamp earlier than all data in transform cache" warnings
- [ ] Both camera views visible in the fused pointcloud in RViz
- [ ] Stats log shows per-interval numbers that change each period
- [ ] `ros2 run tf2_ros tf2_echo marker_map arm_d435i_arm_depth_optical_frame` works consistently

## Potential Risks and Mitigations

1. **Clock sync via NTP may not be possible on the isolated WiFi network**
   Mitigation: Use the header re-stamping approach (Phase 3.1 with cloud timestamps) which doesn't require clock sync. The larger TF buffer cache (Phase 3.2) provides additional tolerance.

2. **Using cloud timestamps for TF lookups may fail if the Jetson clock is far behind**
   Mitigation: The larger buffer cache (Phase 3.2) accommodates moderate skew. For extreme skew, re-stamp cloud headers on arrival.

3. **Caching the last-good bridge transform (Phase 6.2) could serve stale data if the arm moves significantly**
   Mitigation: Add a staleness timeout (e.g., 2 seconds). If the anchor hasn't been seen for longer than that, fall back to the identity assumption with a clear warning. Log the staleness continuously.

4. **Clearing cloud references after processing (Phase 4.2) could reduce output rate if clouds arrive slower than the timer rate**
   Mitigation: This is actually the desired behavior — don't re-publish the same data. The timer will pick up the next fresh cloud when it arrives.

## Alternative Approaches

1. **Header re-stamping on arrival**: In `_cb_cam1`/`_cb_cam2`, replace `msg.header.stamp` with `self.get_clock().now().to_msg()`. This makes all timestamps consistent with the host clock. Simple and effective, but loses original capture time.
   - Trade-off: Introduces network-latency jitter into the timestamp. For fusion purposes (not SLAM), this is usually acceptable.

2. **Single-machine architecture**: Move the fusion node to the Jetson itself, eliminating the cross-machine DDS and clock issues entirely.
   - Trade-off: Requires Jetson to have enough compute for fusion. Major architectural change.

3. **Use `use_sim_time` with a shared time source**: Configure both machines to use a network-synced simulated time source.
   - Trade-off: Adds complexity, requires a time server. Standard ROS2 approach for multi-robot systems.
