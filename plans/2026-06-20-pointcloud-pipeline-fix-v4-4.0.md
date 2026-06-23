# Pointcloud Pipeline — Focused Resolution Plan (v4)

## Objective

Resolve why `/jetson/*/points` and `/fused_pointcloud` produce zero messages.
This plan focuses on four targeted changes: (1) adding camera_info visibility,
(2) widening the Jetson token gate and fixing the no-op `depth.hz`, (3) removing
the token gate for camera_info entirely, and (4) fixing TF configuration for
legacy mode. Restamping approaches are deferred to a future round.

---

## Background: Why Pointclouds Are Zero

The C++ `depth_image_proc::PointCloudXyzrgbNode` uses a 4-way
`ApproximateTimeSynchronizer` (depth image + depth camera_info + RGB image + RGB
camera_info, `slop=0.15`). The logs confirm depth_raw and image_raw arrive at
healthy rates (3.3 Hz and 4.6 Hz respectively), but points=0 Hz with
`[SYNC_DROP]`. Camera_info is the prime suspect but is completely unmonitored.

On the Jetson side, the relay's token gate (`_stamp_matches_token`, 40ms window)
drops depth and camera_info frames whose hardware timestamps don't match the
approved image token within 40ms. This starves depth to 2-3 Hz and may
silently drop all camera_info.

---

## Implementation Plan

### Phase 1: Add Camera_info Monitoring (Host Side)

**Rationale:** Camera_info is the most likely missing input to the synchronizer,
but `pipeline_diagnostics_node` does not monitor `/local/*/camera_info` at all.
We need visibility before and after the other fixes to confirm the root cause
and verify the resolution.

**Files to modify:**
- `src/camera/camera/pipeline_diagnostics_node.py`

- [ ] **1.1. Add camera_info topic parameters to `DEFAULT_PARAMS`.** After
  line 202 (`"arm_image_raw_topic": "/local/arm/image_raw",`), add two new
  default parameters:
  ```
  "head_camera_info_raw_topic": "/local/head/camera_info",
  "arm_camera_info_raw_topic": "/local/arm/camera_info",
  ```

- [ ] **1.2. Subscribe to camera_info topics.** In the `pc_chain_topics` list
  (~line 322-331), add the two new topics. The message type determination at
  ~line 336 currently checks `topic.endswith("/compressed")` for CompressedImage
  vs Image. Camera_info topics end with `/camera_info`, so add a branch:
  if the topic ends with `/camera_info`, use `CameraInfo`; else if it ends with
  `/compressed`, use `CompressedImage`; else use `Image`. Import `CameraInfo`
  from `sensor_msgs.msg` alongside the existing `CompressedImage` import at
  ~line 321.

- [ ] **1.3. Add `camera_info` to the `_pc_topics` dictionary.** In both the
  `"head"` and `"arm"` sub-dicts (~lines 342-357), add:
  `"camera_info": str(p("head_camera_info_raw_topic")),` (and arm equivalent).

- [ ] **1.4. Add `ci_raw=` field and `CI_MISSING` stage to the `[DIAG-PC]`
  output.** In `_periodic_summary` (~lines 558-598):
  - After line 563 (`image_r = ...`), add:
    `ci_r = self._topic_stats.get(tops["camera_info"])`
  - After line 568 (`ir_hz = ...`), add:
    `ci_hz = ci_r.rate_hz(self._rate_window) if ci_r else 0.0`
  - Add a new stage check before the existing `SYNC_DROP` check (~line 577):
    If `dr_hz > 0.1 and ir_hz > 0.1 and ci_hz < 0.1`, set stage to `CI_MISSING`.
    Else if `pt_hz < 0.1 and (dr_hz > 0.1 or ir_hz > 0.1)`, keep `SYNC_DROP`.
  - Update the output line (~line 580-584) to include `ci_raw={ci_hz:.1f}Hz`:
    `f"  {side}: depth_c=... ci_raw={ci_hz:.1f}Hz -> ..."`
  - Add a one-shot warning for `CI_MISSING` (similar to the existing
    `SYNC_DROP` warning at ~line 586-592):
    `"PC CI MISSING: {side} has depth={dr_hz}Hz image={ir_hz}Hz but
    camera_info=0Hz — check Jetson relay token gate or camera_info_bridge"`

### Phase 2: Widen Token Gate + Fix depth.hz (Jetson Side)

**Rationale:** The 40ms token window at `jetson_relay.py:769` is too tight,
starving depth to 2-3 Hz. The `depth.hz` parameter (set to 15 via `relay_hz`)
is read but never applied to any RateGate — it's a no-op. Three inline comments
incorrectly state "10ms".

**Files to modify:**
- `../multiview_prosthesis-jetson_docker/docker_ws/multi_cam_localization/sensor_fusion_bringup/scripts/jetson_relay.py`

- [ ] **2.1. Make the token window configurable.** Add a ROS parameter
  `depth.token_window_ms` (default: 100.0) declared in `__init__`. Replace the
  hardcoded `0.04` at line 769 of `_stamp_matches_token` with
  `self._token_window_s` (converted from ms to seconds in `__init__`).
  A 100ms window accommodates the observed 15-80ms depth-color gap while still
  rejecting frames from different exposure cycles.

- [ ] **2.2. Apply `depth.hz` to a real RateGate.** In the gate registry
  creation (~line 495-500), add:
  `self._gates[f"depth_{cam}"] = RateGate(self._depth_hz)`
  In `_on_depth` (~line 705-722), add the rate check BEFORE the token check:
  `if not self._gates[f"depth_{camera}"].should_publish(): return`
  This caps depth at the configured Hz (15) as a ceiling, while the token gate
  ensures synchronization below that ceiling.

- [ ] **2.3. Fix the stale comments.** Update the three inline comments at
  lines ~493-494, ~710, and ~733-734 that incorrectly state "10ms" to reflect
  the actual configurable threshold (e.g., "within the configurable
  token_window_ms threshold (default 100ms)").

- [ ] **2.4. Update `_log_config` label for depth.** At ~line 811-812, remove
  the misleading "(raw passthrough)" label since depth is now RateGate-limited.
  Change to something like: `f"depth: enabled={...} hz={...} (rate+token gated)"`.

- [ ] **2.5. Add DEBUG logging for token misses.** In
  `_stamp_matches_token`, when the function returns False, add:
  `self.get_logger().debug(f"token miss: {camera} delta={abs(stamp_sec-token_sec)*1000:.1f}ms window={self._token_window_s*1000:.0f}ms")`
  This allows diagnosing the actual timestamp offset at runtime without code
  changes.

### Phase 3: Remove Token Gate for Camera_info (Jetson Side)

**Rationale:** Camera intrinsics are constant for a given resolution — there is
no need to synchronize camera_info timestamps with image frames. The token gate
may be silently dropping all camera_info if the RealSense driver stamps it with
software time rather than hardware time (which would be offset by the full
~884ms clock skew). The 30 Hz RateGate already in place is sufficient to prevent
flooding.

**Files to modify:**
- `../multiview_prosthesis-jetson_docker/docker_ws/multi_cam_localization/sensor_fusion_bringup/scripts/jetson_relay.py`

- [ ] **3.1. Bypass the token gate for camera_info in `_on_ci`.** At ~line
  731-750, remove or comment out the `_stamp_matches_token` check for
  camera_info. Keep the RateGate check (`ci_{camera}` gate at 30 Hz) as the
  sole rate limiter. The callback should become:
  1. RateGate check (30 Hz cap)
  2. Scale intrinsics if `depth_ds > 1`
  3. Publish

  Add a comment explaining the rationale: camera intrinsics are constant, so
  timestamp synchronization is unnecessary — the RateGate alone prevents
  flooding, and the host-side synchronizer will match camera_info to the nearest
  depth/RGB pair within the slop window.

### Phase 4: Fix TF Configuration for Legacy Mode (Host Side)

**Rationale:** `use_corrected_tf: true` in legacy mode causes every odom message
to attempt a TF lookup for `*_imu_openvins_corrected` frames that only exist
when GTSAM is running. Every lookup fails (0.1s timeout each), wasting
computation and adding latency. The outlier suppression thresholds are also too
tight for the operator's confirmed movement range of up to 1.5m.

**Files to modify:**
- `src/prosthesis_launch/launch/pipeline.launch.py`
- `config/prosthesis_config.yaml`

- [ ] **4.1. Make `use_corrected_tf` conditional on `fusion_mode`.** In
  `pipeline.launch.py`, in the `_launch_setup` function where the
  `openvins_odom_tf_relay` node is created (~line 329-336), modify the
  parameters to override `use_corrected_tf` based on `fusion_mode`. The node
  currently gets its params via `_node_params(config, "openvins_odom_tf_relay")`.
  After that call, merge in the override:

  ```python
  relay_params = _node_params(config, "openvins_odom_tf_relay")
  if fusion_mode == "legacy":
      relay_params["use_corrected_tf"] = False
  ```

  This ensures the corrected TF lookup is only attempted when GTSAM is actually
  running (`gtsam_only`, `tsdf_grasp` modes). In legacy mode, the relay goes
  straight to raw odom publishing with no wasted lookups.

- [ ] **4.2. Relax the outlier suppression thresholds.** In
  `config/prosthesis_config.yaml:317-318`, change:
  - `max_pose_norm_m`: from `1.0` to `1.5`
  - `max_pose_jump_m`: from `0.20` to `0.30`

  The operator confirmed cameras move up to 1.5m in real test conditions
  (`docs/CURRENT_ISSUES.md:104`). The current 1.0m norm threshold is too close
  to legitimate movement, causing the relay to freeze the TF edge during normal
  operation. The 0.3m jump threshold still blocks catastrophic VIO divergence
  (which produces jumps of 0.5-1.0m+) while accommodating real camera motion.

### Phase 5: End-to-End Verification

- [ ] **5.1. Deploy and run.** Deploy the Jetson changes to the Jetson,
  rebuild the host workspace (`colcon build --packages-select camera
  prosthesis_launch`), and run the full pipeline for 60+ seconds with
  `fusion_mode:=legacy` and `debug_monitor:=true`.

- [ ] **5.2. Check `[DIAG-PC]` blocks.** Both head and arm should show:
  - `ci_raw=` with a non-zero Hz value (confirming camera_info flows)
  - `points=` with a non-zero Hz value (confirming synchronizer fires)
  - Stage `[OK]` (not `SYNC_DROP` or `CI_MISSING`)

- [ ] **5.3. Check `/fused_pointcloud`.** The fusion node stall diagnostic
  should show `published > 0`. The bag should record non-zero messages on
  `/fused_pointcloud`.

- [ ] **5.4. Check depth throughput.** `/local/*/depth_raw` should show >5 Hz
  (up from 2-3 Hz) with the widened token gate and depth RateGate at 15 Hz.

- [ ] **5.5. Check TF chain health.** `[DIAG-CHAIN]` blocks should show
  `marker_map → *_depth_optical_frame` as CONNECTED for the majority of the
  run after OpenVINS initialization. The `use_corrected_tf=false` change should
  eliminate the 0.1s timeout per message in the relay.

- [ ] **5.6. If SYNC_DROP persists after confirming ci_raw > 0 Hz,** the issue
  is timestamp mismatch between depth and RGB on the host side. At that point,
  consider increasing the synchronizer `slop` further (from 0.15 to 0.25) in
  `pipeline.launch.py:432,450`, or revisit the restamping approaches deferred
  from this round.

---

## Verification Criteria

- [ ] `/local/head/camera_info` and `/local/arm/camera_info` show >0 Hz in
  `[DIAG-PC]` diagnostic blocks
- [ ] `/jetson/head/points` and `/jetson/arm/points` show >0 Hz
- [ ] `/fused_pointcloud` shows >0 Hz (currently ABSENT — 0 messages)
- [ ] `/local/*/depth_raw` shows >5 Hz (currently 2-3 Hz)
- [ ] TF chain `marker_map → *_depth_optical_frame` CONNECTED for >50% of run
  after OpenVINS initialization
- [ ] No `SYNC_DROP` or `CI_MISSING` stage labels in `[DIAG-PC]` after startup
- [ ] No corrected-TF lookup timeouts in `openvins_odom_tf_relay` logs (legacy
  mode)

---

## Potential Risks and Mitigations

1. **Widening the token gate to 100ms may match wrong frame pairs**
   Mitigation: At 3-15 Hz with a 100ms window, the correct depth-color pair is
   always the closest match. The RealSense D435i's depth and color sensors are
   hardware-synchronized, so the actual offset is deterministic. A 100ms window
   is 4-6x the expected 15-23ms offset, providing margin without risking
   cross-cycle mismatches.

2. **Removing the token gate for camera_info may deliver stale intrinsics**
   Mitigation: Camera intrinsics are constant for a fixed resolution. The
   RateGate at 30 Hz ensures fresh delivery. The host synchronizer matches
   camera_info to the nearest depth/RGB pair within the 150ms slop — the exact
   camera_info timestamp is irrelevant for the pointcloud computation.

3. **Relaxing `max_pose_norm_m` to 1.5 may allow noisier TF during VIO
   instability**
   Mitigation: The threshold still blocks catastrophic divergence (VIO
   divergence produces norms of 10-19718m, far above 1.5m). The 0.3m jump
   threshold is the primary defense against sudden divergence. The trade-off
   is more TF availability during legitimate large movements at the cost of
   marginally less aggressive suppression during transient instability.

4. **If SYNC_DROP persists even with camera_info flowing and depth at 5+ Hz**
   Mitigation: Phase 5.6 provides a clear escalation path — increase slop to
   0.25s, or revisit restamping (deferred from this round) as the next step.

---

## Alternative Approaches (Deferred)

1. **Restamp all `/local/*` topics to host clock (Phase 3 Approaches A/B from
   v3):** Eliminates all timestamp-matching issues in one stroke but slightly
   reduces TF lookup precision. Deferred per operator request — revisit if
   SYNC_DROP persists after this round.

2. **Freeze bridge liveness timestamps when upstream is stale:** Would prevent
   the TF chain disconnection caused by the bridge publishing at current time
   while the relay edge is frozen. Deferred — the `use_corrected_tf=false` and
   threshold relaxation changes should reduce the frequency of upstream freezes.

---

## Dependency Graph

```
Phase 1 (CI monitoring) ──────────────────────────────────────────┐
                                                                   │
Phase 2 (token gate + depth.hz) ───► depth at 5-15 Hz ───────────►│
                                                                   │
Phase 3 (remove CI token gate) ───► camera_info flows freely ───►│
                                                                   │
Phase 4 (TF config) ───► TF chain connected ─────────────────────►│
                                                                   ▼
                                                     Fused pointcloud working
```

**Execution order:** Phases 1-4 can be developed in parallel. Deploy all
together, then run Phase 5 verification.

**Jetson changes:** Phases 2 and 3 (single file: `jetson_relay.py`)
**Host changes:** Phases 1 and 4 (two files: `pipeline_diagnostics_node.py`,
`pipeline.launch.py` + `prosthesis_config.yaml`)
