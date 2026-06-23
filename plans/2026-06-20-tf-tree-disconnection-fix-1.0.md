# TF Tree Disconnection Fix — Enable Corrected-TF Path in Legacy Mode

## Objective

Fix the TF tree disconnection that prevents `pointcloud_fusion_node` from transforming point clouds into `marker_map`. The root cause is a two-layer bug: (1) the launch file incorrectly disables the corrected-TF path in legacy mode based on a wrong assumption about who publishes the corrected frames, and (2) the relay's raw VIO outlier suppression blocks the corrected-TF code path. The fix enables the ArUco-corrected TF frames (already published by the Jetson-side `aruco_marker_pose_node.py`) to flow through the relay, keeping the `marker_map → *_imu` edge fresh.

## Root Cause Analysis

### The Problem Chain

```
VIO odom jumps 0.83–0.89m per message (ArUco reanchor instability)
  → openvins_odom_tf_relay outlier suppression (max_pose_jump_m=0.30) rejects ~99% of messages
  → only 15–16 TF updates published in 70 seconds
  → marker_map → *_imu edge goes STALE (age=49.5s, evicted from 10s TF cache)
  → pointcloud_fusion_node TF lookups all fail ("extrapolation into past/future")
  → /fused_pointcloud publishes 0 messages
  → entire downstream pipeline stalls (segmentation, twist propagation, etc.)
```

### Two Bugs

**Bug 1 — Wrong assumption in launch file (`pipeline.launch.py:312-318`)**

The launch file forces `use_corrected_tf=False` in legacy mode with this comment:
> "use_corrected_tf is forced to False — otherwise the relay wastes a 0.1s TF lookup on every odom message for a frame (*_imu_openvins_corrected) that only GTSAM publishes."

This is **factually wrong**. The `*_openvins_corrected` frames are published by the Jetson-side `aruco_marker_pose_node.py` (confirmed in `multiview_prosthesis-jetson_docker/docker_ws/multi_cam_localization/sensor_fusion_bringup/scripts/aruco_marker_pose_node.py:2077-2098`), NOT by the GTSAM tracker. The corrected frames are available in ALL modes, including legacy. The DIAG-TF output proves it: `marker_map → arm_imu_openvins_corrected: updates=1199, age=0.0s` — fresh and active.

**Bug 2 — Outlier suppression blocks corrected-TF path (`openvins_odom_tf_relay.py:411-432`)**

Even if `use_corrected_tf=True` were set, it wouldn't help because of code ordering. In `_on_odom()`:
1. Init guard (lines 367–409) — OK
2. **Outlier suppression (lines 411–432) — `return`s early on raw VIO jump** ← BLOCKS EVERYTHING BELOW
3. Rate limit check (lines 440–450)
4. Corrected-TF path (lines 452–479) — **never reached when VIO is jumping**
5. Raw odom TF broadcast (lines 481–512)

The corrected-TF path uses a completely different data source (ArUco-corrected TF from the Jetson's TF buffer), so the raw VIO outlier check is irrelevant to it. But because the `return` at line 432 exits the entire function, the corrected-TF path is starved along with the raw path.

### Evidence from Logs

- `host-log-20260620_160849.txt:34`: `use_corrected_tf=False` (forced by launch)
- `host-log-20260620_160849.txt:3449`: `marker_map → arm_imu: updates=16, age=49.5s [STALE]`
- `host-log-20260620_160849.txt:3450`: `marker_map → arm_imu_openvins_corrected: updates=1199, age=0.0s [JUMPING]`
- `host-log-20260620_160849.txt:3426`: `published=0, received(cam1=346, cam2=349), tf_fail={...:150, ...:149}`
- `host-log-20260620_160849.txt:483`: `head odom pose jumped 0.849m (> 0.3m) — suppressing TF`

## Implementation Plan

- [x] **Task 1. Remove `use_corrected_tf=False` override in legacy mode (`pipeline.launch.py:317-318`)**

  Delete the two-line override that forces `use_corrected_tf=False` when `fusion_mode=='legacy'`. The config already has `use_corrected_tf: true` at `config/prosthesis_config.yaml:325`, so removing the override lets the config value take effect.

  Specifically, remove:
  ```python
  if fusion_mode == "legacy":
      relay_params["use_corrected_tf"] = False
  ```

  The `relay_params` dictionary will then pass through the config value unchanged.

- [x] **Task 2. Fix the misleading comment in `pipeline.launch.py:312-315`**

  Update the comment block above the relay params to correctly state that `*_openvins_corrected` frames are published by the Jetson-side `aruco_marker_pose_node.py`, not by the GTSAM tracker. The corrected frames are available in all fusion modes.

- [x] **Task 3. Restructure `_on_odom()` to move corrected-TF path before outlier suppression (`openvins_odom_tf_relay.py`)**

  Move the rate-limit check (currently lines 440–450) and the corrected-TF republishing block (currently lines 452–479) to **before** the outlier suppression (currently lines 411–432). The new order in `_on_odom()` should be:

  1. **Init guard** (lines 367–409) — unchanged
  2. **Rate-limit check** (moved from 440–450) — compute `tf_allowed`
  3. **Corrected-TF republishing** (moved from 452–479) — if `tf_allowed and self._use_corrected_tf`, look up corrected frame and rebroadcast as raw `*_imu`. If successful, update rate-limit timestamp and skip to self-calibration/logging at the end.
  4. **Outlier suppression** (lines 411–432, now only reached if corrected wasn't published) — check `pos_norm` and `jump` against thresholds; `return` on failure as before.
  5. **Update last position** (lines 434–438) — unchanged
  6. **Raw odom TF broadcast** (lines 481–512) — unchanged, only reached if outlier check passed
  7. **Self-calibration** (lines 522–526) — unchanged
  8. **Throttled logging** (lines 528–539) — unchanged

  Key detail: when the corrected-TF path succeeds (`published_corrected=True`), update the rate-limit timestamp (`setattr(self, last_tf_attr, now_ns)`) and skip the outlier suppression + raw TF sections entirely. This can be done with an early `return` after the self-calibration and logging sections, or by restructuring with a conditional block.

  The simplest restructuring approach:
  - After the init guard, compute `tf_allowed` and attempt corrected-TF.
  - If `published_corrected`: run self-calibration + logging, then `return`.
  - If not: fall through to outlier suppression → raw TF → self-calibration + logging (existing flow).

- [x] **Task 4. Update the relay startup log to reflect corrected-TF availability**

  The startup WARN log at `openvins_odom_tf_relay.py:328-337` already prints `use_corrected_tf={self._use_corrected_tf}`. No change needed here — it will now correctly show `True` in legacy mode.

- [x] **Task 5. Update the docstring for `use_corrected_tf` parameter (`openvins_odom_tf_relay.py:166-173`)**

  The current docstring says "Only takes effect when publish_dynamic_tf is also True." This is incorrect — the code at line 462 checks `self._use_corrected_tf` independently of `self._publish_dynamic_tf` (the raw path at line 490 has its own `self._publish_dynamic_tf` check). Update the docstring to clarify that corrected-TF works independently of `publish_dynamic_tf`.

## Implementation Status

All five tasks implemented and verified:
- `py_compile` passes on both modified files
- `flake8` reports no new warnings in modified ranges
- Control flow of `_on_odom()` verified end-to-end: init guard → rate-limit → corrected-TF → (raw path gated) → self-calibration + logging

## Verification Criteria

These must be confirmed by running the pipeline (require hardware/Jetson):

- [ ] After restart, the relay startup log shows `use_corrected_tf=True` in legacy mode
- [ ] The `[DIAG-TF]` block shows `marker_map → head_imu` and `marker_map → arm_imu` with `age < 1.0s` (no longer STALE)
- [ ] The `pointcloud_fusion_node` logs show `published > 0` in its stats line (no longer `published=0`)
- [ ] `/fused_pointcloud` topic has messages in the bag (no longer ABSENT)
- [ ] The TF wait gate opens and stays open (no repeated "first cloud batch had no successful transforms" messages)
- [ ] The `tf_pipeline_diagnostics` node reports `OpenVINS(head): OK | OpenVINS(arm): OK` (no longer MISSING)

## Potential Risks and Mitigations

1. **Corrected frames jump 0.87–0.89m**
   The ArUco-corrected TF still has spatial jitter from reanchor events. The fused point cloud will be noisier than ideal. This is acceptable — a noisy cloud is strictly better than no cloud. The deeper VIO/ArUco stability issue should be addressed separately at the Jetson level (chi2 gate tuning, reanchor cooldown, etc.).

2. **0.1s TF lookup per odom message**
   The corrected-TF path does a `lookup_transform` with 0.1s timeout. At ~30 Hz odom rate, this could add latency if the frame is unavailable. However, once the ArUco node is publishing (which it is — 1199 updates in 70s ≈ 17 Hz), the lookup returns immediately. The rate limiter (100 Hz) further bounds the actual lookup frequency. Risk is negligible.

3. **Two publishers on `marker_map → *_imu`**
   If the Jetson's `tf_throttle` is also forwarding `head_imu` / `arm_imu` frames (it's configured to at `dynamic_id2_arm_update_live.launch.py:387-391`), there could be TF conflicts with the relay's rebroadcast. However, the relay stamps with the host clock and the Jetson stamps with the Jetson clock — with chrony sync (drift=0.0ms), these are the same domain. The relay's 100 Hz rate will dominate the Jetson's throttled 50 Hz. If conflicts arise, the Jetson's `tf_throttle` can be configured to drop `head_imu` / `arm_imu` from its forward list, keeping only `*_openvins_corrected`.

4. **`use_header_stamp_age=True` in fusion node**
   The fusion node looks up TF at the cloud's capture time (header stamp). The relay stamps corrected TFs with the host clock at publish time. With chrony sync, the cloud stamp (Jetson system clock) and the TF stamp (host clock) are in the same domain. The 0.15s `transform_tolerance_s` covers small timing differences. The 0.421s transport latency on `/jetson/head/points` means the cloud arrives late, but its stamp is from capture time — which is when the TF should also be available (since the ArUco node publishes at higher rate than the cloud). This should work correctly.

## Alternative Approaches

1. **Increase `max_pose_jump_m` to 1.0m**: Would let more raw VIO TF updates through, but the raw VIO is fundamentally unstable (jumping 0.83m). The fused cloud would be placed at incorrect positions. Using corrected TF is strictly better because the ArUco correction at least anchors to the physical marker map.

2. **Disable outlier suppression entirely (`max_pose_jump_m=999`)**: Same problem as above — raw VIO positions are wrong, just less aggressively wrong. No spatial anchoring to the real world.

3. **Run GTSAM tracker in non-legacy mode**: Would provide smoothed TFs, but requires GTSAM to be functional and adds significant complexity. The corrected-TF approach achieves the same goal (stable, anchored TFs) with a simpler change.

4. **Have the fusion node look up `*_openvins_corrected` frames directly**: Would require changing the fusion node's target frame or adding frame remapping. More invasive and couples the fusion node to the corrected-frame naming convention. The relay's rebroadcast approach (corrected → raw name) is cleaner because it's transparent to all downstream consumers.
