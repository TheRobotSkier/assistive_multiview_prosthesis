# Pointcloud Fusion — TF Timing & Frame Selection Deep Analysis

**Date:** 2026-05-20  
**Version:** v2 (refined after code re-read)  
**Status:** Investigation Complete

---

## The Core Problem (Re-stated Clearly)

The fusion node needs to transform point clouds from each camera's `*_depth_optical_frame` into `marker_map`. The TF chain has 5 hops:

```
marker_map → head_imu → head_cam0 → head_d435i_head_link → head_d435i_head_depth_frame → head_d435i_head_depth_optical_frame
                         ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
                         THIS IS THE ONLY EDGE THE HOST PUBLISHES (the bridge)
```

Everything except `head_cam0 → head_d435i_head_link` comes from the Jetson. This single edge is the **only fragile link** in the entire chain. If it breaks, the whole tree disconnects.

## How the Bridge Computes That Edge

The bridge at `openvins_realsense_tf_bridge_node.py:403-433` tries two paths:

### Primary Path (anchor)
```
_lookup_matrix(head_cam0, head_d435i_head_color_optical_frame_body_display)
```
This looks up the transform from `head_cam0` to the `body_display` frame. OpenVINS publishes `body_display` as a child of `head_cam0` — it represents where the camera body is in the VIO frame. Since `anchor_frame_mode = "link"`, the bridge uses this transform **directly** as `head_cam0 → head_d435i_head_link`.

**This only works when the aruco marker is visible**, because `body_display` is published by the Jetson's `aruco_marker_pose_node.py` and only exists when the marker is detected.

### Fallback Path
```
_lookup_matrix(head_d435i_head_depth_optical_frame, head_d435i_head_link)
```
This looks up the RealSense depth-optical-to-link extrinsic (a static transform). The bridge then **assumes `head_cam0` is identical to `head_d435i_head_depth_optical_frame`** and uses the extrinsic as the bridge transform.

**This is geometrically wrong.** `head_cam0` is the OpenVINS camera frame (optical convention: Z forward, Y down) while `head_d435i_head_depth_optical_frame` is the RealSense depth optical frame. They are different physical sensors with different positions and orientations. The fallback produces a transform that's rotated and translated incorrectly.

## Why the Fusion is Intermittent

The log shows a clear pattern:

1. **Bridge gets anchor → correct transform → fusion works** (`dual=29, cam1_only=12`)
2. **Marker occluded → bridge falls back → wrong transform → only one camera fuses** (the other's TF chain is broken)
3. **Both bridges lose anchor → both fallbacks wrong → complete stall** (`published=0` for 85+ seconds)
4. **Cache expires (30s) → no transform at all → stall continues**

The cache fix (30s) helps during brief marker occlusions but can't survive extended ones. And the fallback actively makes things worse by publishing a wrong transform.

## The "Behind" Markers in RViz

These are the `*_color_optical_frame_raw`, `*_from_marker`, and `*_body_display` frames from `report.md:50-58`. They're published by the Jetson's aruco marker detection node and represent **raw, unfiltered single-frame detections**. They lag behind the VIO estimate because they're not integrated with IMU data. They're diagnostic frames, not used by the fusion pipeline.

## Is `head_cam0` the Right Frame?

**Yes.** Here's the mapping:

| Frame | Source | Quality |
|---|---|---|
| `head_cam0` | OpenVINS MSCKF VIO | **Best real-time estimate.** Integrates IMU + visual features at ~200 Hz. Smooth, drift-corrected. |
| `*_body_display` | aruco marker detection | Noisy, intermittent. Only exists when marker visible. Used by the bridge as a calibration reference, not as the primary pose. |
| `*_from_marker` | aruco marker detection | Same as above, different convention. |
| `*_raw` | aruco marker detection | Raw detection under `marker_0`. No filtering. |

The VIO estimate (`head_cam0`) is the correct frame for real-time fusion. The aruco frames are only useful for the bridge to calibrate the extrinsic offset between the OpenVINS camera frame and the RealSense body frame.

## Root Cause Summary

The bridge depends on the aruco `body_display` frame to compute a **static extrinsic** (the physical mounting offset between two cameras). This extrinsic never changes — it's a property of the physical camera mount. But the bridge treats it as a dynamic quantity that needs to be re-computed every tick, and falls back to a wrong value when the marker is lost.

**The fix is to compute the extrinsic once and lock it permanently.**

---

## Implementation Plan

- [ ] **Task 1: Lock the bridge extrinsic permanently after first successful anchor resolution.**  
  In `_resolve_bridge_transform`, add a `_extrinsic_locked` dict per camera. Once a transform is successfully resolved from the anchor path, store it in `_extrinsic_locked` and never discard it. On subsequent ticks, always try the live anchor first (to refine the estimate), but if the live lookup fails, use the locked extrinsic with no time limit. This eliminates the staleness limit entirely for the primary path.

- [ ] **Task 2: Disable the fallback path by default.**  
  The fallback at `openvins_realsense_tf_bridge_node.py:426-433` produces a geometrically wrong transform. Set `fallback_to_optical_assumption: false` in `config/prosthesis_config.yaml:166`. The locked extrinsic from Task 1 makes the fallback unnecessary — once the extrinsic is locked, the bridge always has a valid transform.

- [ ] **Task 3: Add a startup log showing the locked extrinsic values.**  
  When the extrinsic is first locked, log the translation (xyz) and rotation (rpy) so you can verify it's reasonable. This helps catch calibration issues.

- [ ] **Task 4: (Optional) Subscribe to odom topics as an alternative extrinsic source.**  
  The Jetson publishes `/ov_msckf/odomimu` and `/ov_msckf_arm/odomimu` at 200 Hz. These contain the full VIO pose in `marker_map`. The bridge could use these to compute the bridge transform without depending on the aruco anchor at all. This is a more robust approach but requires more code changes. Consider this if the locked extrinsic approach doesn't provide sufficient accuracy.

- [ ] **Task 5: Rebuild and test with the Jetson running.**  
  After the changes, restart the pipeline and verify: (a) the bridge logs "locked extrinsic" once per camera, (b) `dual` fusion is consistent (not intermittent), (c) the fusion doesn't stall when markers are occluded.

## Verification Criteria

- Bridge logs show extrinsic locked exactly once per camera within the first 30 seconds
- No "fallback assuming" messages in the bridge log after initial lock
- Fusion stats show `dual` count consistently >50% of total publishes per 10s interval
- No stalls (`published=0` intervals) during normal operation
- `last_publish_ago` stays below 1 second continuously

## Potential Risks and Mitigations

1. **Extrinsic drifts over time due to camera physical movement**  
   Mitigation: The bridge still tries the live anchor path every tick. If a new anchor is detected, it updates the locked extrinsic. The lock only prevents falling back to the wrong identity transform.

2. **Initial extrinsic is computed from a noisy anchor detection**  
   Mitigation: Could average the first N anchor detections before locking. For now, a single detection should be sufficient since the aruco marker provides a good estimate.

3. **Disabling fallback means no transform until first anchor detection**  
   Mitigation: This is acceptable — the first anchor detection typically happens within 5-10 seconds of startup. The fusion node's stall diagnostic will clearly show that the bridge is waiting for the anchor.
