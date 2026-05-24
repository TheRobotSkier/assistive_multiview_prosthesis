# Investigation Report: v9 Log Analysis — Point Cloud Jumping & Instability

## Objective

Analyze the v9 camera test log to identify all issues causing point cloud jumping, instability, and drift beyond what is expected from OpenVINS VIO updates.

---

## Summary of Findings

The v9 log confirms that the two startup crash fixes (from v7) are resolved. All 12 nodes start cleanly. However, **three distinct bugs** remain, two of which are directly causal to the instability and jumping you observe. Additionally, a **severe OpenVINS drift event** was captured that explains the massive coordinate explosion in the twist propagation logs.

---

## Finding 1 (CRITICAL): Bbox removal 0% success rate — hand/arm never filtered

**Evidence:** Every stats line from line 68 to line 1991 shows `bbox_removed=0, bbox_cache_hits=0`. Every 30 seconds the health checker reports 0% success. Every 5 seconds a "no cached transform available" warning fires.

**Root cause:** The `_lookup_bbox_transform()` method at `src/pointcloud_fusion/pointcloud_fusion/pointcloud_fusion_node.py:585-591` uses:
- `stamp = now - 100ms` (a specific past timestamp)
- `timeout = Duration(seconds=0)` (zero timeout, non-blocking)

This lookup requires the full TF chain `marker_map -> arm_imu -> arm_cam0 -> arm_d435i_arm_link -> screw_frame -> palm_frame` to be resolvable at that exact timestamp. The chain involves edges from three independent publishers at different rates (200 Hz, 10 Hz, 10 Hz). With zero timeout, TF2 cannot wait for interpolation data to arrive, so the lookup **always fails**. Since it always fails, the cache is never populated, and the fallback always hits "no cached transform available".

**Contrast with what works:** The TF diagnostics (`tf_pipeline_diagnostics.py:69-78`) uses `can_transform()` with `Time()` (latest) and a **1-second timeout** — this always succeeds. The main cloud transform at line 422-425 uses `rclpy.time.Time()` (latest) with the **default infinite timeout** — this also succeeds. Only the bbox lookup uses a specific past time with zero timeout.

**Impact on jumping:** The hand and arm geometry are **never removed** from the fused cloud. This means:
1. The arm/hand points move with every VIO update, creating visible jitter
2. The twist propagation node receives these moving points and interprets them as object motion
3. The "jumping" you see is partially the unfiltered arm/hand geometry shifting in the cloud

**Fix:** Change `_lookup_bbox_transform()` to use `rclpy.time.Time()` (latest) with a small non-zero timeout (e.g., 50ms). This matches the approach used by the main cloud transform and the diagnostics node.

---

## Finding 2 (CRITICAL): OpenVINS drift event — massive coordinate explosion

**Evidence:** The twist propagation poses show a clear drift event:
- Lines 152-161 (~T+80s): Normal poses around `(-0.16, -0.16, 0.25)` — realistic coordinates
- Lines 244-246 (~T+88s): Sudden jump to `(-0.241, 0.422, 0.306)` — Y coordinate flips from negative to positive
- Lines 247-251 (~T+89s): Rapid acceleration to `(-0.316, 0.768, 0.312)` — Y doubles
- Lines 618-620 (~T+114s): Explosion to `(26.543, 21.920, 1.557)` with covariance trace 251.72 exceeding max 100.0
- Lines 624-627: `(29.782, 23.526, 1.944)` — coordinates grow by ~2m/s
- Lines 1915-1920 (~T+309s): `(698.085, 796.486, 99.652)` — hundreds of meters
- Lines 1980-1990 (~T+315s): `(730.012, 818.998, 107.859)` — still drifting

The drift rate is approximately **2-3 m/s** in X and Y, consistent with an OpenVINS tracking failure where the VIO accumulates velocity bias.

**Impact on jumping:** This is the **primary cause** of the "jumping around and things drift away" behavior. The fused point cloud is transformed using the OpenVINS pose, so when OpenVINS drifts, the entire cloud shifts by meters per second. The bbox removal failure (Finding 1) means the hand/arm points in the cloud amplify this visual effect.

**Root cause:** OpenVINS tracking quality degradation. This is not a host-side software bug — it's a VIO algorithm issue on the Jetson. Possible causes:
1. Insufficient visual features (textureless scene, motion blur)
2. IMU bias drift not being corrected
3. Camera calibration or time synchronization issues between IMU and camera
4. The Jetson's OpenVINS instance losing tracking and not recovering

**Note:** This is separate from the host-side TF issues. The host correctly relays whatever OpenVINS sends. The problem is that what OpenVINS sends is wrong.

---

## Finding 3 (HIGH): Intermittent TF extrapolation errors cause dropped frames

**Evidence:**
- Line 134: `head_d435i_head_depth_optical_frame: Lookup would require extrapolation into the past. Requested time 1779624873.732594 but the earliest data is at time 1779624873.857778` (125ms gap)
- Line 141: `head_d435i_head_depth_optical_frame: Requested time 1779624879.436010 but the earliest data is at time 1779624880.592106` (1.16s gap)
- Line 547: `head_d435i_head_depth_optical_frame: Requested time 1779624925.894115 but the earliest data is at time 1779624926.019933` (125ms gap)
- Line 817: `head_d435i_head_depth_optical_frame: Requested time 1779624942.287782 but the earliest data is at time 1779624942.551378` (263ms gap)
- Line 1053: `arm_d435i_arm_depth_optical_frame: Requested time 1779624959.690215 but the earliest data is at time 1779624959.947280` (257ms gap)

These errors occur because the main cloud transform at `pointcloud_fusion_node.py:422-425` uses `rclpy.time.Time()` (which TF2 interprets as "latest"). But the cloud's original timestamp (from the Jetson) is older than the earliest TF data in the host's buffer. The `openvins_odom_tf_relay` stamps TFs with the **host clock** via `get_clock().now()`, while the cloud is stamped with the **Jetson clock**. There is a clock skew between the two machines.

When TF2 processes `rclpy.time.Time()`, it resolves to the latest transform. But when the cloud's frame_id is used in the lookup, TF2 needs to find the transform at a time compatible with the cloud's stamp. The error messages show the "requested time" is the cloud's stamp (from the Jetson clock), and the "earliest data" is from the host clock — the Jetson clock is behind the host clock by 125ms to 1.16s.

**Impact:** Frames are dropped when the clock skew exceeds the TF buffer's interpolation window. The stats show `tf_fail` counts of 1-50 per 10-second window, meaning some frames are lost but most get through.

**Fix:** The `openvins_odom_tf_relay` should stamp its TFs with the **Jetson's odom timestamp** (from the incoming odom message) rather than the host clock. This ensures the TF timestamps are in the same time domain as the cloud timestamps. Alternatively, the fusion node could use a larger TF buffer duration.

---

## Finding 4 (MEDIUM): Distance filter intermittently fails

**Evidence:** Lines 81, 88, 124, 142, 151, 561, 838, 1055 show:
```
Cannot look up arm_d435i_arm_depth_frame in marker_map for distance filter — skipping
```

This uses `_get_frame_origin_in_target()` at `pointcloud_fusion_node.py:689-700` with `rclpy.time.Time()` (latest, no timeout specified = infinite). It should generally work, but fails intermittently. This is likely the same clock skew issue — when the TF buffer has been recently cleared (e.g., after an OpenVINS reconnection), the latest transform may not yet be available.

**Impact:** When the distance filter fails, points beyond 2m from the arm are not filtered. This increases noise in the cloud but doesn't directly cause jumping.

---

## Finding 5 (INFO): All previous fixes confirmed resolved

- **No node crashes** — all 12 nodes start and run for the entire session (2004 lines, ~5 minutes)
- **`openvins_odom_tf_relay`** starts successfully at line 16, no crash
- **`segmentation_ros2_node`** starts successfully at line 17, initializes with `roi_radius=0.3m` (line 53)
- **TF diagnostics** consistently reports `All chains healthy` from line 136 onward (after OpenVINS connects)
- **Dual-camera fusion** works — stats show `dual` counts increasing (e.g., line 148: `dual=26`, line 372: `dual=36`, line 491: `dual=37`)

---

## Finding 6 (INFO): Twist propagation shows the drift clearly

The twist propagation node's poses are derived from the fused cloud's frame (marker_map). The progression of poses tells the story:
- T+80s: Normal `(-0.16, -0.16, 0.25)` — the prosthesis is near the origin
- T+88s: Jump to `(-0.24, 0.42, 0.31)` — OpenVINS starts drifting
- T+114s: `(26.5, 21.9, 1.6)` — massive drift, covariance exceeds max
- T+309s: `(698, 796, 100)` — hundreds of meters from origin

This confirms the drift is in the OpenVINS output, not in the TF relay or fusion pipeline.

---

## Priority-Ordered Issue List

| Priority | Issue | Type | Impact on Jumping |
|----------|-------|------|-------------------|
| 1 | OpenVINS drift event | VIO algorithm | Primary cause — entire cloud shifts by meters |
| 2 | Bbox removal always fails | Host software bug | Amplifies visual jitter from unfiltered arm/hand |
| 3 | Clock skew TF errors | Host software bug | Drops frames, causes intermittent gaps |
| 4 | Distance filter intermittent | Host software bug | Minor — increases noise |

---

## Recommended Actions

### Fix 1: Bbox lookup timeout (host-side, high confidence fix)
Change `_lookup_bbox_transform()` at `pointcloud_fusion_node.py:585-591` to use `rclpy.time.Time()` with a 50ms timeout instead of `now - 100ms` with 0 timeout.

### Fix 2: Clock skew mitigation (host-side, medium confidence fix)
In `openvins_odom_tf_relay.py`, use the odom message's timestamp instead of `get_clock().now()` for TF stamps. This aligns the TF time domain with the cloud time domain.

### Fix 3: OpenVINS drift (Jetson-side, requires investigation)
The OpenVINS drift is the primary cause of the "jumping around and drifting away" behavior. This requires investigation on the Jetson side:
- Check OpenVINS logs for tracking quality metrics
- Verify IMU-camera time synchronization
- Check if the drift correlates with specific motions or scenes
- Consider adding a drift detection mechanism that flags when the VIO pose exceeds reasonable bounds

---

## Verification Criteria

- [ ] Bbox removal success rate > 90% in v10 log
- [ ] No TF extrapolation errors in v10 log
- [ ] Twist propagation poses remain within 1m of origin during stable tracking
- [ ] `bbox_removed > 0` in stats lines
