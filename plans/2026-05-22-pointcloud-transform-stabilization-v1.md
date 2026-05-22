# Point Cloud Transform Jump Stabilization

## Objective

Investigate and plan a lightweight stabilization strategy for the fused point cloud output to eliminate visible "jumping" caused by TF transform instability. The solution must not add significant latency to the pipeline (currently ~15 Hz fusion rate) and should work within the existing `pointcloud_fusion_node.py` architecture.

## Root Cause Analysis

### Evidence from Test Logs (v5, v6)

The test logs reveal **three distinct but interacting problems**:

1. **OpenVINS TF chain intermittently disconnected** — The bridge between `marker_map` and camera optical frames relies on OpenVINS tracking. When tracking is lost or delayed, the entire TF chain breaks. Logs show frequent "not part of the same tree" errors and "extrapolation into the past" failures (`camera-test-log-v6.txt:89`, `camera-test-log-v6.txt:233`, `camera-test-log-v6.txt:304`).

2. **Bbox removal 0% success rate throughout** — `bbox_removed=0` and `bbox_cache_hits=0` persist across the entire v6 test session. The `palm_frame` and `d435i_arm_bottom_screw_frame_8_cm_cam_mount` transforms are never available (`camera-test-log-v6.txt:80-81`, `camera-test-log-v6.txt:84`, `camera-test-log-v6.txt:102`). This means hand/arm points are never filtered out.

3. **TF timestamp gaps cause stale transforms** — The OpenVINS bridge publishes at only ~2 Hz (liveness mode), while the point cloud fusion runs at 15 Hz. The 100ms lookup offset at `pointcloud_fusion_node.py:511` is insufficient to bridge the gap when OpenVINS updates arrive with >100ms spacing. This causes the "extrapolation into the past" errors.

### Architecture Context

The TF chain is: `marker_map -> *_imu -> *_cam0 -> *_d435i_*_link -> *_depth_frame -> *_depth_optical_frame`. The critical dynamic edges (`marker_map -> *_cam0`) come from OpenVINS running on the Jetson, bridged to the host at ~2 Hz. Any latency, jitter, or dropout in this chain causes the fused cloud to jump or fail entirely.

### Why the Cloud "Jumps"

When the TF lookup at `pointcloud_fusion_node.py:423-426` succeeds but with a slightly different transform than the previous frame (due to OpenVINS pose estimation noise, interpolation artifacts, or stale data), the entire point cloud shifts in world frame. Since the fusion node uses `rclpy.time.Time()` (latest available) for the main cloud transform (`pointcloud_fusion_node.py:425`), each frame may use a slightly different transform, causing visible oscillation.

## Implementation Plan

### Phase 1: Transform Smoothing (Low risk, immediate impact)

- [ ] **Add an Exponential Moving Average (EMA) smoother for the main cloud TF lookup.** Instead of using the raw TF result from `lookup_transform()` at `pointcloud_fusion_node.py:423`, cache the previous transform (R, t_vec) and blend with the new one using a configurable alpha (e.g., 0.3). This is a single matrix interpolation per frame — negligible cost. Rationale: The OpenVINS bridge publishes at ~2 Hz but the fusion runs at 15 Hz, so between updates the same (possibly stale) transform is used. When a new transform arrives, it may differ from the previous one, causing a visible jump. EMA smooths these transitions.

- [ ] **Implement dual-quaternion rotation interpolation for the EMA.** Use scipy or a manual implementation to slerp between the previous and current rotation, avoiding gimbal lock issues. The translation component uses standard linear interpolation: `t_new = alpha * t_raw + (1 - alpha) * t_prev`. Rationale: Direct matrix blending can produce non-orthogonal rotation matrices; quaternion slerp preserves rotation validity.

- [ ] **Add a `transform_smoothing_alpha` parameter (default 0.5).** Higher values = more responsive but more jittery; lower = smoother but more lag. Allow runtime tuning. Rationale: The optimal alpha depends on the OpenVINS update rate and scene dynamics — it needs to be tunable per deployment.

- [ ] **Add a `transform_max_jump_m` parameter (default 0.05).** If the raw transform jumps more than this threshold (measured as translation delta), clamp the update to the max jump distance in the direction of the new transform. Rationale: Protects against gross tracking errors or relocalization events that would cause a huge cloud displacement.

### Phase 2: Point Cloud Cache / Temporal Fusion (Medium risk, significant improvement)

- [ ] **Add a small ring buffer of recent fused point clouds (2-3 frames).** Store the last N published clouds in world frame. When a new cloud is produced, blend it with the cached clouds using a configurable weight. Rationale: Averaging multiple frames reduces noise and smooths out single-frame transform artifacts. The cost is keeping 2-3 extra point clouds in memory (~few MB each at typical D435 resolution).

- [ ] **Implement confidence-weighted temporal blending.** Assign confidence to each cached frame based on: (a) TF chain health (was the transform fresh or stale?), (b) cloud age, (c) number of points. Newer frames with healthy TF get higher weight. Rationale: Frames produced during TF outages are less reliable and should contribute less to the output.

- [ ] **Add `temporal_blend_frames` parameter (default 1, max 3).** When set to 1, no temporal blending (backward compatible). When set to 2-3, enables the ring buffer. Rationale: Allows gradual rollout and A/B testing without risk.

### Phase 3: TF Health-Aware Publish Gating (Low risk, prevents bad output)

- [ ] **Track per-frame TF freshness in `_process_clouds`.** Record whether the transform used was from a fresh lookup vs. a stale/repeated one. Publish this as a metadata field or use it to gate publishing. Rationale: Currently the node publishes even when using a stale transform, which causes jumps. Skipping frames with stale transforms would be cleaner than publishing jittery output.

- [ ] **Add a `transform_max_age_s` parameter (default 0.3).** If the most recent successful transform for a camera frame is older than this threshold, skip that camera's cloud for this cycle rather than transforming with a stale transform. Rationale: The current approach uses whatever transform is available, regardless of age. An age limit prevents large jumps from accumulated drift.

- [ ] **Implement "last good cloud" caching.** When TF is healthy and the output looks good, cache the full published cloud. When TF degrades (failures, stale data), republish the cached cloud instead of producing a bad one. Rationale: A slightly stale but correct cloud is better than a fresh but incorrectly transformed one. The downstream segmentation and grasp planning can handle a ~200ms stale cloud but not a wildly displaced one.

### Phase 4: IPC (Inter-Process Communication) Optimization (Lower priority, architectural)

- [ ] **Evaluate replacing the timer-based merge with shared memory for point cloud transfer.** The current pipeline serializes PointCloud2 messages through ROS2 DDS, which adds serialization/deserialization overhead. Using `ros2_shared_memory` or numpy shared memory between the camera driver and fusion node could reduce latency by ~5-15ms per frame. Rationale: The Jetson-to-host network transit already introduces significant latency (logs show cloud stamps "seconds behind the host clock" per `pointcloud_fusion_node.py:560-561`). Reducing in-host overhead helps but won't fix the core TF jitter issue.

- [ ] **This is deferred to a future phase** because it requires changes to the camera driver layer and ROS2 QoS configuration, which is a larger architectural change. The smoothing approaches in Phases 1-3 address the visible jumping without this infrastructure change.

## Recommended Implementation Order

**Start with Phase 1 (transform smoothing)** — it is the simplest change (modify only `_process_clouds` in `pointcloud_fusion_node.py`), adds negligible latency (~0.1ms for a quaternion slerp), and directly addresses the observed jumping. If the EMA smoothing is insufficient, proceed to Phase 3 (TF health gating) which prevents bad frames from being published at all. Phase 2 (temporal blending) is a more aggressive approach that can be added if Phases 1+3 don't provide enough stability.

## Verification Criteria

- [ ] Fused point cloud in RViz shows no visible jumping during normal operation (OpenVINS tracking stable)
- [ ] Fused point cloud shows reduced jumping during OpenVINS tracking transitions (loss/recovery)
- [ ] `bbox_removed > 0` in stats output (hand removal working when TF is healthy)
- [ ] End-to-end latency does not increase by more than 5ms (measured via `/fused_pointcloud` header stamp delta)
- [ ] Published cloud rate remains at ~15 Hz (no stalls from smoothing logic)
- [ ] No new TF failures introduced by the smoothing (transform remains valid)

## Potential Risks and Mitigations

1. **EMA introduces lag in dynamic scenes**
   Mitigation: The `transform_smoothing_alpha` parameter allows tuning. Set default to 0.5 (balanced). The max-jump clamp prevents large lag accumulation. During fast arm movement, the user may prefer some jitter over lag — make it configurable.

2. **Quaternion slerp direction ambiguity**
   Mitigation: Always normalize quaternions and ensure dot product is positive before slerp. Use `scipy.spatial.transform.Slerp` or a proven implementation rather than hand-rolling.

3. **Temporal blending increases memory usage**
   Mitigation: Limit ring buffer to 3 frames max. At typical D435 resolution (~90K points * 16 bytes = ~1.4 MB per cloud), 3 frames is ~4.2 MB — negligible on the Jetson/host.

4. **"Last good cloud" caching masks real tracking failures**
   Mitigation: Log a warning when republishing cached clouds. Add a max cache age (e.g., 0.5s) after which the node stops publishing rather than serving very stale data.

5. **Transform smoothing interacts badly with bbox removal**
   Mitigation: Apply smoothing only to the main cloud transform (Step 1 in pipeline). The bbox transforms should use the raw (unsmoothed) TF lookup since they need to track the actual arm position for accurate hand removal.

## Alternative Approaches

1. **OpenVINS-side smoothing (increase bridge rate from 2 Hz to 15 Hz):** Instead of smoothing on the fusion side, increase the TF bridge publish rate so there are no gaps. This would be the cleanest fix but requires modifying the Jetson-side OpenVINS configuration and may not be feasible due to network bandwidth constraints.

2. **Static TF fallback with periodic calibration:** When OpenVINS tracking is lost, fall back to a static transform (last known good) rather than trying to use whatever fragmentary TF data is available. This is essentially what Phase 3's "last good cloud" does but at the TF level rather than the cloud level.

3. **Marker-based relocalization triggering:** When a large transform jump is detected, request a new ArUco marker detection to re-anchor the TF tree. This addresses the root cause (OpenVINS drift) but requires changes to the Jetson-side pipeline and adds complexity.
