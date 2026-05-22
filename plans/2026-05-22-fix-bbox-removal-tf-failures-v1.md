# Fix: Bbox Removal Failing Due to Intermittent TF Lookup Failures

## Objective

Fix the intermittent "Cannot transform to palm_frame" and "Cannot transform to d435i_arm_bottom_screw_frame_8_cm_cam_mount" warnings in the pointcloud fusion node that cause the hand/arm to not be filtered out of the fused cloud. The root cause is a timing/race condition between TF availability and the fusion node's bbox removal step, combined with a design fragility where bbox removal silently skips when TFs are unavailable.

---

## Root Cause Analysis

### TF Tree Connectivity (Pipeline Mode)

The pruning box frames depend on this chain:

```
marker_map -> arm_imu -> arm_cam0 -> arm_d435i_arm_link -> d435i_arm_bottom_screw_frame_8_cm_cam_mount -> palm_frame
                                                  ^
                                                  |
                                         OpenVINS bridge publishes this edge
```

- **OpenVINS bridge** publishes `arm_cam0 -> arm_d435i_arm_link` at **2 Hz liveness** (`openvins_realsense_tf_bridge_node.py:416-417`)
- **Camera mount TF publisher** publishes `arm_d435i_arm_link -> screw_frame -> palm_frame` at **1 Hz liveness** (`publish_camera_mounts.py:196`)

### Why Bbox TF Lookups Fail Intermittently

The fusion node runs at **15 Hz** (`fallback_merge_rate_hz=15.0`). The bbox removal step at `pointcloud_fusion_node.py:482-498` calls:

```python
xyz_box = _transform_points_to_frame(
    xyz_all, self._tf_buffer, frame,
    self._target_frame, rclpy.time.Time(),  # <-- rclpy.time.Time() = "latest available"
)
```

This calls `tf_buffer.lookup_transform(target_frame=palm_frame, source_frame=marker_map, stamp=rclpy.time.Time())`.

**Three failure modes observed in the log:**

1. **Disconnected TF tree** (lines 48-51, 52-54 of log): During startup, OpenVINS head bridge hasn't resolved yet, and even the arm chain can be disconnected briefly. The camera mount TF publisher only publishes at 1 Hz, and the OpenVINS bridge at 2 Hz — there are windows where TFs expire from the buffer.

2. **Extrapolation into the past** (lines 330, 389, 549, 602, 678): The liveness ticks re-stamp TFs with the current time. If a processing cycle's cloud was captured before the latest liveness tick, TF2 rejects the lookup because it would require extrapolation into the past. This is especially likely when the fusion node processes a cloud with `rclpy.time.Time()` (latest) while the TF buffer only has data starting from a slightly later timestamp.

3. **TF buffer expiry**: TF2 has a default buffer duration (typically 10s). The combination of low liveness rates (1-2 Hz) and the fusion node processing in a background daemon thread means TF updates from the executor can be starved while the thread is running, causing TFs to appear stale or missing.

### Key Insight: The Fusion Node Uses `rclpy.time.Time()` (Zero) for Bbox Lookup

At `pointcloud_fusion_node.py:485`, the stamp is `rclpy.time.Time()` which means "get the latest available transform." However, the cloud processing runs in a **daemon thread** (`pointcloud_fusion_node.py:385-386`), which means the executor is NOT blocked and can continue updating the TF buffer. But there's still a race: if the camera mount TF publisher hasn't sent its 1 Hz liveness tick yet, or the OpenVINS bridge hasn't sent its 2 Hz liveness tick, the TF buffer simply won't have the transform at that instant.

### Why bbox_removed Is Almost Always 0

Looking at the stats lines in the log:
- `bbox_removed=0` appears in the vast majority of 10-second stat intervals
- `bbox_removed=377607` appears once (line 56) when TFs were briefly available
- `bbox_removed=141` (line 86), `bbox_removed=148` (line 125), `bbox_removed=11824` (line 308), `bbox_removed=488` (line 392), `bbox_removed=8742` (line 527), `bbox_removed=11096` (line 428), `bbox_removed=94` (line 540)

The bbox removal works only sporadically when all TFs happen to be available simultaneously.

---

## Implementation Plan

### Phase 1: Make Bbox Removal Robust Against TF Timing Gaps (Primary Fix)

- [x] **1.1. Add TF lookup retry with short timeout in `_transform_points_to_frame`** — Instead of using `rclpy.time.Time()` (instantaneous lookup), use a small timeout (e.g., 100-200ms) via `tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.2))`. This gives the TF buffer a window to receive the next liveness tick from the camera mount publisher (1 Hz = up to 1s wait) or the OpenVINS bridge (2 Hz = up to 500ms wait). A 200ms timeout is short enough to not degrade the 15 Hz processing rate significantly (each cycle gets ~66ms budget; a 200ms wait would only be hit when the TF is genuinely unavailable, not on every cycle). Rationale: This is the simplest targeted fix for the intermittent lookup failures.

- [x] **1.2. Cache the last successful bbox transform and use it as fallback** — Add a `_pruning_box_transforms` dict to `PointCloudFusionNode` that caches the most recent successful `lookup_transform` result for each pruning box frame. When a fresh TF lookup fails, reuse the cached transform (with a staleness check — e.g., reject if older than 2 seconds). The camera mount TFs are **quasi-static** (they change only when the robot arm moves, and the screw-to-palm relationship is rigid), so a slightly stale transform is far better than no filtering at all. Rationale: Even with retry timeouts, there will be brief gaps; caching ensures bbox removal always runs when it physically should.

- [x] **1.3. Log a more actionable warning when bbox removal is skipped** — Change the warning at `pointcloud_fusion_node.py:495-498` to include the TF chain diagnosis (e.g., which specific link is missing in the chain from `marker_map` to the pruning box frame). This makes future debugging much faster. Also add a counter metric `bbox_skipped` alongside `bbox_removed` in the stats to track how often filtering is being bypassed.

### Phase 2: Increase TF Liveness Rate for Pruning Box Frames

- [x] **2.1. Increase camera mount TF publisher liveness rate from 1 Hz to 5-10 Hz** — In `publish_camera_mounts.py:196`, the liveness timer is set to 1.0 seconds. Change this to 0.2s (5 Hz) or 0.1s (10 Hz). Since the camera mount TFs are static (only 8 transforms for a single mount), the bandwidth cost is negligible (8 transforms * ~200 bytes each * 10 Hz = ~16 KB/s). Rationale: The fusion node processes at 15 Hz; having TFs updated at 1 Hz means 93% of processing cycles see a "stale" TF entry. 5-10 Hz ensures TFs are fresh for most cycles.

- [x] **2.2. Consider publishing camera mount TFs as static transforms via `/tf_static` with `/tf` liveness at higher rate** — The camera mount TFs are truly static within a session (screw-to-palm doesn't change). The current code already publishes to both `/tf_static` and `/tf` at startup (`publish_camera_mounts.py:238-239`), and the liveness timer re-sends on `/tf` at 1 Hz. The fix is just increasing the liveness rate as described in 2.1. No architectural change needed.

### Phase 3: Fix the Extrapolation-Into-The-Past Failure Mode

- [x] **3.1. Use a recent-but-not-zero timestamp for bbox TF lookups instead of `rclpy.time.Time()`** — The current code uses `rclpy.time.Time()` which means "latest available." When processing runs in a background thread while TF liveness ticks update the buffer with future timestamps, the TF buffer can return extrapolation errors. Instead, use a timestamp that's slightly in the past (e.g., `self.get_clock().now() - rclpy.duration.Duration(seconds=0.1)`) to give the TF buffer a safe margin. Rationale: The "extrapolation into the past" errors (log lines 330, 389, 549, 602, 678) show that the cloud timestamp is older than the earliest TF data — using a slightly older lookup timestamp avoids this.

- [x] **3.2. Alternatively, pass the cloud's original stamp to the bbox lookup** — Superseded by 3.1 which uses `now() - 100ms` offset, which is more robust than passing the cloud stamp. — The cloud already has a stamp from step 6 (`pointcloud_fusion_node.py:519`: `header.stamp = self.get_clock().now().to_msg()`). But the bbox removal happens BEFORE the stamp is overwritten (it uses the raw `xyz_all` which is already in `marker_map` frame). The issue is that `rclpy.time.Time()` should work for "latest" lookups — the extrapolation error occurs when the TF buffer has data starting AFTER the requested time. Using the current time minus a small offset is more robust.

### Phase 4: Harden the Design — Never Silently Skip Bbox Removal

- [x] **4.1. Add a configurable `bbox_fallback_mode` parameter** — When TF lookup fails for a pruning box, instead of silently skipping, offer configurable behavior:
  - `"skip"` (current behavior) — log warning and skip
  - `"cache"` — use last known good transform (requires 1.2)
  - `"conservative"` — skip publishing the fused cloud entirely if bbox removal can't run (safest: avoids ever publishing a cloud with the hand in it)

  Default to `"cache"` since it provides the best balance of availability and correctness.

- [x] **4.2. Add a health metric and alert when bbox removal success rate drops below threshold** — Track the ratio of successful bbox removals to total attempts. If the success rate drops below 90% over a 30-second window, log an ERROR (not just WARN) indicating that the fused cloud quality is degraded. This makes the issue visible in test runs.

---

## Verification Criteria

- [ ] **V1**: No "Cannot transform to palm_frame for bbox removal — skipping" warnings in a 5-minute `camera-test` run when both OpenVINS chains are healthy (i.e., after the initial startup period)
- [ ] **V2**: `bbox_removed` count in stats is consistently > 0 in every 10-second stats interval (not just sporadically)
- [ ] **V3**: The ratio of `bbox_removed` to total points should be consistent across intervals (not wildly varying between 0 and hundreds of thousands)
- [ ] **V4**: No "extrapolation into the past" errors for bbox-related TF lookups
- [ ] **V5**: Fused cloud published at 15 Hz with bbox removal running on every cycle (not just when TFs happen to be available)

---

## Potential Risks and Mitigations

1. **Stale cached transforms cause incorrect bbox removal** — If the arm moves significantly between the cached transform and the current position, points that should be kept might be removed (or vice versa). Mitigation: Set a staleness limit (2 seconds) on cached transforms; if exceeded, fall back to skip mode and log an error. Since the screw-to-palm relationship is rigid, only gross arm motion (via the OpenVINS bridge edge) could make the cached transform inaccurate — and the distance filter (step 3) provides an additional safety net.

2. **TF lookup timeout slows down the processing pipeline** — Adding a 200ms timeout to `_transform_points_to_frame` could in theory slow processing. Mitigation: The timeout is only hit when the TF is genuinely unavailable (rare case). In the normal case, the lookup succeeds immediately. Also, the timeout is per-bbox-box, and there are only 2 boxes, so worst case adds ~400ms to a cycle — but this only happens during TF gaps, not continuously.

3. **Higher liveness rate increases DDS traffic** — Going from 1 Hz to 10 Hz for camera mount TFs means 10x more TF messages. Mitigation: 8 transforms at 10 Hz is ~16 KB/s, which is negligible compared to the pointcloud data (each cloud is ~2-5 MB). The OpenVINS bridge already runs at 2 Hz liveness for its 2 bridge edges.

4. **Daemon thread processing may still race with TF buffer updates** — The fusion node processes clouds in a daemon thread, but the TF buffer is updated by the executor thread. Mitigation: Python's GIL ensures thread-safe access to the TF buffer. The real issue is timing, not thread safety — the retry/timeout and caching approaches address this.

---

## Alternative Approaches

1. **Move bbox removal to use the same transform as the cloud transform (step 1)** — Instead of looking up `marker_map -> palm_frame` separately for each pruning box, precompute the transform chain once when the cloud is received and reuse it. This would mean transforming the pruning box AABB corners into `marker_map` frame once, then applying them. However, this requires handling orientation (the AABB in palm_frame may not be axis-aligned in marker_map after rotation), so it would need an OBB check instead of AABB, which is more complex.

2. **Use a dedicated TF listener with a longer buffer duration** — Increase the TF buffer cache time from the default (10s) to, say, 30s. This wouldn't fix disconnected-tree or extrapolation errors, but would make the system more tolerant of brief liveness gaps. Simple but insufficient as a standalone fix.

3. **Merge camera mount TF publisher into the OpenVINS bridge node** — Having a single node publish all the TFs (bridge edges + mount TFs) at a consistent 2-10 Hz rate would eliminate the desynchronization between the two publishers. This is a more invasive change but would simplify the TF publishing architecture. The downside is tighter coupling between the bridge and mount configuration.

4. **Precompute static pruning box transforms at startup** — Since the `screw_frame -> palm_frame` and `screw_frame -> bbcam1/bbcam2` relationships are defined in `camera_mounts.yaml` and are truly static, the fusion node could compute the `marker_map -> palm_frame` transform from the yaml data + the live OpenVINS bridge edge, rather than relying on the camera_mount_tf_publisher at all. This would eliminate the dependency on a second node's TF updates entirely. This is the most robust approach but requires significant refactoring.
