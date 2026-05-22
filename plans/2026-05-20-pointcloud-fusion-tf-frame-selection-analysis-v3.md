# Pointcloud Fusion — TF Timing & Frame Selection Deep Analysis

**Date:** 2026-05-20  
**Version:** v3 (refined with rate analysis and static publishing strategy)  
**Status:** Investigation Complete

---

## The Core Problem

The fusion node needs a 5-hop TF chain to transform point clouds into `marker_map`:

```
marker_map → head_imu → head_cam0 → head_d435i_head_link → head_d435i_head_depth_frame → head_d435i_head_depth_optical_frame
             200Hz VIO   200Hz VIO   15Hz BRIDGE            STATIC                        STATIC
             MOVES       MOVES       RIGID MOUNT             NEVER CHANGES                 NEVER CHANGES
```

The bridge edge `head_cam0 → head_d435i_head_link` is the **only edge published by the host**. It is a rigid physical mounting offset between the OpenVINS tracking camera and the RealSense body — it never changes when the robot moves.

When the robot moves, the first two edges (`marker_map → head_imu → head_cam0`) update at 200 Hz from OpenVINS on the Jetson. The bridge edge is static.

The problem: the bridge treats this static extrinsic as dynamic, re-computing it every tick from an intermittent aruco anchor frame, and falling back to a geometrically wrong identity assumption when the anchor disappears.

## Rate Analysis

| Edge | Source | Rate | Changes when robot moves? |
|---|---|---|---|
| `marker_map → head_imu` | OpenVINS VIO (Jetson) | ~200 Hz | YES — this is the VIO pose |
| `head_imu → head_cam0` | OpenVINS VIO (Jetson) | ~200 Hz | YES — this is the VIO pose |
| `head_cam0 → head_d435i_head_link` | HOST bridge | 15 Hz | **NO — rigid mount** |
| `head_d435i_head_link → *_depth_frame` | RealSense static (Jetson) | once | NO |
| `*_depth_frame → *_depth_optical_frame` | RealSense static (Jetson) | once | NO |

**15 Hz is overkill for a static transform.** Once the extrinsic is locked, it should be published on `/tf_static` (latched, once) instead of re-published on `/tf` at 15 Hz.

The dynamic publishing at 15 Hz is only useful during the **initial calibration phase** — before the extrinsic is locked, the bridge keeps trying to refine its estimate from the aruco anchor. Once locked, the 15 Hz timer becomes unnecessary overhead for this edge.

## How the Bridge Currently Works (and Why It's Wrong)

The bridge at `openvins_realsense_tf_bridge_node.py:403-433`:

1. **Primary path**: Look up `head_cam0 → body_display`. If found, use as bridge transform. **Correct, but only available when aruco marker is visible.**

2. **Fallback**: Look up `depth_optical_frame → link`, assume `head_cam0 == depth_optical_frame`. **Geometrically wrong** — different sensors, different positions, different orientations.

3. **Cache**: Store last-good transform for up to 30 seconds. **Too short** — this is a static quantity.

## The "Behind" Markers in RViz

The `*_color_optical_frame_raw`, `*_from_marker`, and `*_body_display` frames are raw aruco detections from the Jetson. They lag behind the VIO estimate because they're unfiltered single-frame detections. Not used by the fusion pipeline. Red herring.

## Frame Selection

`head_cam0` (OpenVINS VIO output) is the correct frame. It integrates IMU + visual features at 200 Hz. The `body_display` frames are only useful for the bridge to calibrate the static extrinsic offset.

---

## Implementation Plan

- [ ] **Task 1: Lock the bridge extrinsic permanently after first successful anchor resolution.**  
  Add `_extrinsic_locked: dict[str, np.ndarray]` per camera. Once a transform is successfully resolved from the anchor path, store it in `_extrinsic_locked` and never discard it. On subsequent ticks, always try the live anchor first (to refine), but if the live lookup fails, use the locked extrinsic with **no time limit**.

- [ ] **Task 2: Switch to `/tf_static` publishing once the extrinsic is locked.**  
  After the extrinsic is locked, publish the bridge edge on `/tf_static` (latched) via the existing `_static_tf_broadcaster`. Stop re-publishing it on `/tf` at 15 Hz. This eliminates the timer overhead and ensures the transform never expires from the TF buffer. The 15 Hz timer can continue for other purposes (nominal static chain re-broadcast, refinement attempts) but the locked edge goes out once and is done.

- [ ] **Task 3: Disable the wrong fallback path.**  
  Set `fallback_to_optical_assumption: false` in `config/prosthesis_config.yaml:166`. The locked extrinsic makes the fallback unnecessary. The fallback produces a geometrically wrong transform — having it active is worse than having no transform at all.

- [ ] **Task 4: Log the locked extrinsic values for verification.**  
  When the extrinsic is first locked, log the translation (xyz) and rotation (rpy) so you can verify it's reasonable. Also log when the refinement updates the locked value (from a new anchor detection).

- [ ] **Task 5: (Optional) Subscribe to odom topics as a more robust calibration source.**  
  The Jetson publishes `/ov_msckf/odomimu` and `/ov_msckf_arm/odomimu` at 200 Hz. The bridge could use these to compute the extrinsic without depending on the aruco anchor. Consider this if the locked extrinsic approach doesn't provide sufficient accuracy.

- [ ] **Task 6: Rebuild and test with the Jetson running.**  
  Verify: (a) bridge logs "locked extrinsic" once per camera, (b) `dual` fusion is consistent, (c) no stalls during marker occlusion.

## Verification Criteria

- Bridge logs show extrinsic locked exactly once per camera within the first 30 seconds
- No "fallback assuming" messages in the bridge log after initial lock
- Fusion stats show `dual` count consistently >50% of total publishes per 10s interval
- No stalls (`published=0` intervals) during normal operation
- `last_publish_ago` stays below 1 second continuously

## Potential Risks and Mitigations

1. **Extrinsic drifts due to camera physical movement**  
   Mitigation: The bridge still tries the live anchor every tick. If a new anchor is detected, it updates the locked value and re-publishes on `/tf_static`.

2. **Initial extrinsic computed from noisy anchor detection**  
   Mitigation: Could average the first N detections before locking. A single detection should be sufficient for aruco-based calibration.

3. **No transform until first anchor detection**  
   Mitigation: Acceptable — first detection typically within 5-10 seconds. The fusion node's stall diagnostic will clearly show the bridge is waiting.
