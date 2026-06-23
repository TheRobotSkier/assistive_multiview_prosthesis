# Pointcloud Pipeline — Root Cause Analysis and Resolution Plan (v3)

## Objective

Diagnose and resolve why `/jetson/*/points` and `/fused_pointcloud` produce zero
messages despite depth and RGB data arriving successfully on the host. This plan
builds on the v2.0 plan (which fixed the ArUco crash, camera_info RateGate, and
QoS), identifies what remains broken, and provides a step-by-step resolution path.

---

## Root Cause Analysis

### The Failure Chain (Confirmed from Logs + Code)

```
Jetson RealSense → jetson_relay token gate (40ms) → depth starved to 2-3 Hz
                                                      camera_info possibly dropped
                                                          ↓
Host decompress_bridge → /local/*/depth_raw (3 Hz) ✓
                       → /local/*/image_raw (4.5 Hz) ✓
                       → /local/*/camera_info (?? Hz) ← UNMONITORED
                                                          ↓
depth_image_proc::PointCloudXyzrgbNode (C++ ApproximateTimeSynchronizer)
    slop=0.15, queue_size=25
    Requires: depth + rgb + camera_info timestamps within 150ms
    Result: 0% match → SYNC_DROP → 0 points
                                                          ↓
pointcloud_fusion_node → receives 0 clouds → 0 fused output
```

### Three Confirmed Problems

#### Problem 1: Camera_info Is a Complete Blind Spot (PRIMARY SUSPECT)

The `pipeline_diagnostics_node` monitors depth_compressed, image_compressed,
depth_raw, image_raw, and points — but **does NOT monitor `/local/*/camera_info`**
(`src/camera/camera/pipeline_diagnostics_node.py:322-331`). The `[DIAG-PC]` block
has no `ci_raw=` field. We have zero visibility into whether camera_info is
arriving on the host side.

The C++ `ApproximateTimeSynchronizer` requires **four** inputs: depth image +
depth camera_info + RGB image + RGB camera_info. If camera_info is absent or its
timestamps don't match, the synchronizer produces nothing — exactly the observed
SYNC_DROP.

**Evidence:** The bag shows `/jetson/*/camera_info` arriving at ~7 Hz on the wire,
but we cannot confirm it reaches `/local/*/camera_info` or that its timestamps
match the depth/RGB frames.

**Likely root cause:** The Jetson relay's token gate
(`_stamp_matches_token`, 40ms window at `jetson_relay.py:769`) may be silently
dropping camera_info if the RealSense camera_info timestamp differs from the
image timestamp by more than 40ms. Some RealSense driver versions publish
camera_info with software (ROS) time rather than hardware (ASIC) time, which
would be offset by the full clock skew (~884ms).

#### Problem 2: Token Gate Starves Depth to 2-3 Hz

The `_stamp_matches_token` function (`jetson_relay.py:752-769`) uses a **40ms**
window to match depth and camera_info timestamps against the approved image
token. However:

- The `depth.hz` parameter (set to 15 via `relay_hz`) is **never applied to any
  RateGate** — it's a complete no-op (`jetson_relay.py:811-812` labels it "raw
  passthrough" which is misleading).
- The actual depth throughput is entirely determined by how often a depth frame's
  hardware timestamp falls within 40ms of an approved image token.
- At the observed rates (depth 2-3 Hz vs image 4.5 Hz), the token gate is
  dropping 50-65% of depth frames.
- Three inline comments (`jetson_relay.py:493-494, 710, 733-734`) incorrectly
  state "10ms" when the actual threshold is 40ms — suggesting the threshold was
  widened but insufficiently.

#### Problem 3: TF Chain Disconnected for Entire Run

The analysis shows `marker_map → *_depth_optical_frame` DISCONNECTED for all 18
diagnostic blocks. Two contributing factors:

1. **`use_corrected_tf: true` in legacy mode** (`config/prosthesis_config.yaml:325`):
   The relay attempts to look up `*_imu_openvins_corrected` frames on every
   message, but these are only published by the GTSAM tracker (not running in
   legacy mode). Every lookup silently fails and falls through to raw odom.

2. **VIO jumps trigger outlier suppression**: The relay rejects poses with
   jumps > 0.2m (`max_pose_jump_m`), freezing the `marker_map → *_imu` edge.
   When this edge is frozen but downstream edges (`*_cam0 → *_link`) continue at
   current time, TF2 cannot compose the multi-edge chain — hence DISCONNECTED
   for depth_optical_frame while *_imu shows CONNECTED (single-edge, stale cache).

The fusion node's TF wait gate did open at 28s (line 549 of host log), so TF is
a secondary issue — the primary blocker is that no clouds exist to fuse.

---

## Implementation Plan

### Phase 1: Add Camera_info Monitoring (Host Side) — CRITICAL DIAGNOSTIC

**Rationale:** We cannot fix what we cannot see. Camera_info is the most likely
missing input to the synchronizer, but it's completely unmonitored. This is a
zero-risk diagnostic addition.

- [ ] **1.1. Add `/local/*/camera_info` to the pipeline_diagnostics_node's
  monitored topics.** In `src/camera/camera/pipeline_diagnostics_node.py`, add
  `head_camera_info_raw_topic` and `arm_camera_info_raw_topic` to the
  `pc_chain_topics` list (~line 322-331), and add a `ci_raw` field to the
  `_pc_topics` dictionary (~line 342-357). Subscribe to these topics with
  `CameraInfo` message type and BEST_EFFORT QoS (matching the camera_info_bridge
  output which is RELIABLE — either QoS will work for monitoring).

- [ ] **1.2. Add `ci_raw=` field to the `[DIAG-PC]` output block.** In the
  `_periodic_summary` method (~line 560-584), add the camera_info rate alongside
  depth_raw and image_raw. Update the SYNC_DROP condition: if depth_raw and
  image_raw are present but ci_raw is <0.1 Hz, label the stage as `CI_MISSING`
  instead of `SYNC_DROP` to distinguish the two failure modes.

- [ ] **1.3. Add a one-shot warning for camera_info absence.** When depth and
  RGB are flowing but camera_info is <0.1 Hz, log:
  `"PC CI MISSING: {side} has depth={dr_hz}Hz image={ir_hz}Hz but
  camera_info=0Hz — check Jetson relay token gate or camera_info_bridge"`.

### Phase 2: Widen the Token Gate and Fix depth.hz (Jetson Side)

**Rationale:** The 40ms token window is too tight for the actual RealSense
depth-color timestamp offset, starving depth to 2-3 Hz. The `depth.hz` parameter
is a misleading no-op that should either be applied or documented.

- [ ] **2.1. Make the token window configurable.** In `jetson_relay.py`, add a
  parameter `depth.token_window_ms` (default: 100.0) and use it in
  `_stamp_matches_token` instead of the hardcoded `0.04`. A 100ms window
  accommodates the observed 15-80ms depth-color gap while still rejecting
  frames from different exposure cycles. This is a safer first step than
  removing the token gate entirely.

- [ ] **2.2. Apply `depth.hz` to a real RateGate.** In `_setup_depth` (or
  wherever depth subscriptions are created), add
  `self._gates[f"depth_{cam}"] = RateGate(self._depth_hz)` and check it in
  `_on_depth` BEFORE the token check. This ensures depth is rate-limited to
  the configured Hz (15 Hz) as a ceiling, while the token gate ensures
  synchronization. Remove the misleading "(raw passthrough)" label in
  `_log_config`.

- [ ] **2.3. Fix the stale comments.** Update the three inline comments at
  `jetson_relay.py:493-494, 710, 733-734` that incorrectly state "10ms" to
  reflect the actual configurable threshold.

- [ ] **2.4. Add debug logging for token misses.** In `_stamp_matches_token`,
  when the function returns False, log the actual stamp difference at DEBUG
  level: `self.get_logger().debug(f"token miss: {camera} stamp_delta={delta*1000:.1f}ms")`.
  This allows diagnosing the actual timestamp offset without code changes.

### Phase 3: Fix Camera_info Timestamp Handling (Host or Jetson Side)

**Rationale:** If Phase 1 confirms camera_info is the missing input, the issue
is almost certainly timestamp-related. Camera intrinsics are constant for a
given resolution — the exact timestamp is irrelevant for the pointcloud
computation. The safest fix is to ensure camera_info always has a usable
timestamp.

**Approach A (Preferred — Host Side, Zero Jetson Changes):**

- [ ] **3.1. Restamp camera_info on the camera_info_bridge.** Modify
  `src/camera/camera/camera_info_bridge.py` to optionally overwrite the output
  timestamp with the most recent depth or RGB frame timestamp. Add a parameter
  `restamp_to_match` (default: True). When True, the bridge caches the most
  recent timestamp seen on any `/local/*/depth_raw` or `/local/*/image_raw`
  topic (via a shared parameter or a separate subscription) and stamps the
  camera_info output with that timestamp. This ensures the synchronizer always
  sees a camera_info with a matching timestamp.

  **Simpler alternative:** Since the bridge is a simple pass-through, just
  overwrite `msg.header.stamp` with `self.get_clock().now()` when
  `restamp_to_match` is True. This puts camera_info in the host clock domain.
  The synchronizer will then need depth and RGB to also be in the host clock
  domain — which they are NOT (they carry Jetson hardware timestamps). So this
  only works if depth and RGB are also restamped.

**Approach B (More Robust — Restamp All Three on Host):**

- [ ] **3.2. Add a `restamp` parameter to `decompress_bridge.py`.** When True,
  overwrite `out.header.stamp` with `self.get_clock().now().to_msg()` before
  publishing. Do the same in `camera_info_bridge.py`. Enable `restamp=True`
  for all six bridges (4 decompress + 2 camera_info) in the launch file.

  **Trade-off:** All messages on `/local/*` will carry host arrival-time
  stamps. The synchronizer will match perfectly since all inputs share the
  same clock domain. The fusion node's clock-skew auto-fallback
  (`pointcloud_fusion_node.py:452-467`) will activate, using arrival time for
  cloud age and TF lookups. TF lookup precision is slightly reduced (the cloud
  timestamp won't exactly match the TF timestamp), but the fusion node already
  handles this via its `cloud_max_age_s` tolerance.

  **This is the fastest path to visible pointclouds.** It eliminates all
  timestamp-matching issues in one stroke. The v2.0 plan rejected restamping
  to preserve TF precision, but that argument is moot when zero pointclouds
  are produced. Once pointclouds are flowing, the restamp can be selectively
  disabled to test whether original timestamps work with the widened token gate.

**Approach C (Jetson Side — Remove Token Gate for Camera_info):**

- [ ] **3.3. Bypass the token gate for camera_info.** In `jetson_relay.py:_on_ci`,
  remove or bypass the `_stamp_matches_token` check. Camera intrinsics are
  constant — there is no need to synchronize camera_info with image frames.
  The RateGate (30 Hz) is sufficient to prevent flooding. Forward every
  camera_info that passes the RateGate, regardless of timestamp.

### Phase 4: Fix TF Configuration for Legacy Mode (Host Side)

**Rationale:** The `use_corrected_tf: true` setting causes wasted computation
and subtle failures in legacy mode where GTSAM is not running. The TF chain
disconnections also prevent the fusion node from processing clouds even if they
were produced.

- [ ] **4.1. Set `use_corrected_tf: false` in legacy fusion mode.** In
  `config/prosthesis_config.yaml:325`, change `use_corrected_tf: true` to
  `false`. When GTSAM is not running (legacy mode), the corrected frame lookup
  always fails and silently falls through to raw odom. Setting it to false
  eliminates the wasted lookup on every message and makes the relay's behavior
  explicit.

  Alternatively, make this conditional on `fusion_mode` in the launch file:
  if `fusion_mode == 'legacy'`, pass `use_corrected_tf:=false`.

- [ ] **4.2. Relax the outlier suppression thresholds.** In
  `config/prosthesis_config.yaml:317-318`, increase `max_pose_norm_m` from 1.0
  to 1.5 and `max_pose_jump_m` from 0.20 to 0.30. The current thresholds are
  too tight for legitimate camera movement (cameras move up to 1.5m per the
  operator in `docs/CURRENT_ISSUES.md:104`). With the ArUco fix deployed,
  VIO should be more stable, but the thresholds should still accommodate
  normal movement without freezing the TF chain.

- [ ] **4.3. Consider freezing bridge liveness timestamps when upstream is
  stale.** When `marker_map → *_imu` stops flowing (relay suppression), the
  `openvins_realsense_tf_bridge_node`'s liveness timer continues publishing
  `*_cam0 → *_link` at current time. This prevents TF2 from composing the
  multi-edge chain at the old timestamp. Consider having the bridge detect
  upstream staleness (no `marker_map → *_imu` transform for >2s) and freeze
  its liveness timestamp to match the last known upstream timestamp.

### Phase 5: End-to-End Verification

- [ ] **5.1. Run the pipeline with all fixes for 60+ seconds.** Use
  `debug_monitor:=true` on the Jetson and `fusion_mode:=legacy` on the host.
  Record a bag and host/jetson logs.

- [ ] **5.2. Check the `[DIAG-PC]` blocks.** Both head and arm should show
  non-zero `points` Hz and stage `OK` (not `SYNC_DROP` or `CI_MISSING`).
  Camera_info rate (`ci_raw=`) should be >0 Hz.

- [ ] **5.3. Check `/fused_pointcloud`.** The fusion node should report
  `published > 0` in its stall diagnostic. The bag should show non-zero
  messages on `/fused_pointcloud`.

- [ ] **5.4. Check TF chain health.** The `[DIAG-CHAIN]` blocks should show
  `marker_map → *_depth_optical_frame` as CONNECTED for the majority of the
  run after OpenVINS initialization.

- [ ] **5.5. Check depth throughput.** With the widened token gate (100ms)
  and depth RateGate at 15 Hz, depth should arrive at 5-15 Hz (limited by
  the actual RealSense frame rate and network capacity).

---

## Verification Criteria

- [ ] `/local/head/camera_info` and `/local/arm/camera_info` show >0 Hz in the
  `[DIAG-PC]` diagnostic block
- [ ] `/jetson/head/points` and `/jetson/arm/points` show >0 Hz (currently 0.0 Hz
  with `[SYNC_DROP]`)
- [ ] `/fused_pointcloud` shows >0 Hz (currently ABSENT — 0 messages)
- [ ] Depth arrives at >5 Hz on `/local/*/depth_raw` (currently 2-3 Hz)
- [ ] TF chain `marker_map → *_depth_optical_frame` stays CONNECTED for >50% of
  the run after initialization
- [ ] No `SYNC_DROP` or `CI_MISSING` labels in `[DIAG-PC]` blocks after startup

---

## Potential Risks and Mitigations

1. **Restamping breaks TF-based cloud transformation**
   Mitigation: The fusion node has a clock-skew auto-fallback
   (`pointcloud_fusion_node.py:452-467`) that switches to arrival-time stamping
   when skew >1.0s. With restamping, this fallback activates automatically.
   Cloud positions may be slightly less precise (using arrival time instead of
   capture time for TF interpolation), but this is far better than zero clouds.
   Once clouds are flowing, selectively disable restamping to test whether
   original timestamps work with the widened token gate.

2. **Widening the token gate may match wrong frame pairs**
   Mitigation: At 3-15 Hz with a 100ms window, the correct depth-color pair is
   always the closest match. The RealSense D435i's depth and color sensors are
   hardware-synchronized, so the actual offset is deterministic (not random).
   A 100ms window is 4-6x the expected offset, providing margin without risking
   cross-cycle mismatches.

3. **Removing the token gate for camera_info may increase wire traffic**
   Mitigation: The RateGate at 30 Hz is still active. Camera_info messages are
   0.4KB each — at 30 Hz that's 12KB/s, negligible compared to depth+RGB at
   ~300KB/s.

4. **Relaxing outlier suppression thresholds may allow noisier TF**
   Mitigation: The thresholds still block catastrophic divergence (1.5m norm
   and 0.3m jumps are well below the 100m+ divergence values). The trade-off
   is more TF availability at the cost of slightly noisier transforms during
   transient VIO instability.

---

## Alternative Approaches

1. **Generate pointclouds on the Jetson side (revert to old architecture):**
   Re-enable the pointcloud publishers in `jetson_relay.py` and the RealSense
   pointcloud filter. This eliminates the host-side synchronizer entirely but
   increases wire bandwidth by ~30x (raw pointcloud vs compressed depth+RGB).
   Not recommended given the Cat5e bandwidth constraints.

2. **Use `depth_image_proc` with exact sync instead of approximate:** This would
   require perfectly matched timestamps, which is even harder to achieve. Not
   recommended — approximate sync with sufficient slop is the right approach.

3. **Replace the C++ `depth_image_proc` with a Python pointcloud generator:**
   A custom Python node could subscribe to depth, RGB, and camera_info
   independently and generate pointclouds without a strict synchronizer — using
   the most recent camera_info regardless of timestamp. This eliminates the
   synchronizer dependency entirely but adds computational overhead and
   maintenance burden.

4. **Use `depthimage_to_laaserscan` or a depth-only pointcloud:** Skip the RGB
   component and generate pointclouds from depth + camera_info only (2-way sync
   instead of 4-way). This reduces the synchronization burden but loses color
   information needed for segmentation.

---

## Recommended Execution Order

```
Phase 1 (camera_info monitoring) ──► Run pipeline ──► Confirm CI_MISSING
         │
         ├── Phase 2 (widen token gate) ──► Run pipeline ──► Check depth rate
         │
         ├── Phase 3 Approach B (restamp all) ──► Run pipeline ──► Confirm points > 0
         │                                                        (FASTEST PATH)
         │
         ├── Phase 4 (TF config fixes) ──► Run pipeline ──► Confirm fused > 0
         │
         └── Phase 5 (end-to-end verification)
```

**Fastest path to visible pointclouds:** Phase 1 + Phase 3 Approach B (restamp
all bridges) + Phase 4.1 (use_corrected_tf=false). This can be done entirely on
the host side with zero Jetson changes.

**Most robust long-term fix:** Phase 1 + Phase 2 (token gate fix) + Phase 3
Approach C (remove token gate for camera_info) + Phase 4. This requires Jetson
changes but preserves original timestamps and fixes the root cause.

---

## Dependency Graph

```
Phase 1 (monitoring) ──────────────────────────────────────────────────┐
                                                                       │
Phase 2 (token gate) ───► depth at 5-15 Hz ──────────────────────────►│
                                                                       │
Phase 3 (camera_info fix) ───► synchronizer fires ───► points > 0 ──►│
                                                                       │
Phase 4 (TF config) ───► TF chain connected ─────────────────────────►│
                                                                       ▼
                                                         Fused pointcloud working
```

Phase 1 is zero-risk and provides immediate diagnostic value.
Phase 3 Approach B is the fastest unblock (host-only changes).
Phase 2 and Phase 4 are improvements that increase robustness.
