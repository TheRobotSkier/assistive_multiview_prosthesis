# Pointcloud Fusion — TF Timing & Frame Selection Analysis

**Date:** 2026-05-20
**Status:** Investigation Complete

---

## The Core Problem

The fusion node transforms point clouds from each camera's `*_depth_optical_frame` into `marker_map`. The TF chain for each camera has **5 hops**, and the weakest link is the bridge edge published by the host:

```
marker_map -> head_imu -> head_cam0 -> head_d435i_head_link -> head_d435i_head_depth_frame -> head_d435i_head_depth_optical_frame
                         ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
                         HOST BRIDGE — published at 15 Hz, depends on OpenVINS anchor
```

The bridge edge `head_cam0 -> head_d435i_head_link` is the **only transform published by the host**. Everything else comes from the Jetson. This single edge is fragile because:

1. It depends on the `body_display` anchor frame from OpenVINS, which only exists when the aruco marker is visible
2. When the anchor disappears, the bridge falls back to a geometrically wrong identity assumption
3. Even with the cache fix (30s), the cached transform becomes stale as the camera moves

## The "Behind" Markers in RViz

You mentioned seeing markers that are "always behind." These are almost certainly the **`*_color_optical_frame_raw`** and **`*_color_optical_frame_from_marker`** frames. Looking at the TF tree in `report.md:50-58`:

```
marker_map
├── head_d435i_head_color_optical_frame_body_display    (only when marker visible)
├── head_d435i_head_color_optical_frame_from_marker     (only when marker visible)
├── head_imu_from_marker                                (only when marker visible)
└── marker_0
    └── head_d435i_head_color_optical_frame_raw          (only when marker visible)
```

These frames are published by the Jetson's `aruco_marker_pose_node.py`. They appear and disappear with marker visibility, and when visible, they represent the **raw aruco marker detection** — not the VIO-filtered estimate. They will lag behind the VIO pose because they're unfiltered single-frame detections.

**These frames are NOT used by the fusion pipeline.** They're diagnostic/visualization frames. The fusion node uses `head_d435i_head_depth_optical_frame` (from the RealSense static chain), which connects through `head_cam0` (the VIO estimate). The "behind" frames are a red herring for the fusion quality issue.

## The Real Question: Is `head_cam0` the Right Frame?

You asked whether `head_cam0` or `arm_cam0` might provide better estimates. Let me map out exactly what each frame represents:

| Frame | Source | What it is | Update rate |
|---|---|---|---|
| `head_cam0` | OpenVINS MSCKF | VIO-estimated camera pose in `marker_map`. Integrates IMU + visual features. **Smooth, drift-corrected.** | ~200 Hz (odom rate) |
| `head_d435i_head_color_optical_frame_body_display` | aruco_marker_pose | Raw aruco marker detection. **Noisy, intermittent, laggy.** | Only when marker visible |
| `head_d435i_head_color_optical_frame_from_marker` | aruco_marker_pose | Same as above, different convention | Only when marker visible |
| `head_d435i_head_color_optical_frame_raw` | aruco_marker_pose | Raw detection under `marker_0` | Only when marker visible |

**`head_cam0` IS the correct frame.** It's the VIO output — the best real-time estimate of where the camera is in the world. The `body_display` frames are only used by the bridge as a calibration reference (to figure out the extrinsic offset between OpenVINS camera frame and RealSense link frame).

## Why the Fusion is Still Intermittent

Even with all the fixes, the fundamental issue is **TF chain fragility**. Here's what happens:

### The Bridge's Anchor Dependency

The bridge resolves `head_cam0 -> head_d435i_head_link` by:

1. **Primary path**: Look up `head_cam0 -> head_d435i_head_color_optical_frame_body_display`. If found, use that as the transform (mode `link`). This works because OpenVINS publishes the `body_display` frame as a child of `head_cam0` — it's the camera body pose in the VIO frame.

2. **Fallback path**: If `body_display` is not available, look up `head_d435i_head_depth_optical_frame -> head_d435i_head_link` and assume `head_cam0 == head_d435i_head_depth_optical_frame`. This is **geometrically wrong** because `head_cam0` (OpenVINS camera frame) and `head_d435i_head_depth_optical_frame` (RealSense optical frame) have different coordinate conventions (optical vs. ROS).

### The Timing Gap

The bridge runs at 15 Hz and publishes with the **host clock** (`get_clock().now()`). The OpenVINS VIO data arrives from the Jetson at ~200 Hz. But the `body_display` frame only appears when the aruco marker is detected — which can be intermittent.

When the marker is visible:
- `body_display` exists → bridge computes correct transform → TF chain is good
- Fusion works, both clouds fuse correctly

When the marker is lost:
- `body_display` disappears → bridge uses cache (up to 30s) → TF chain degrades
- After 30s, bridge falls back to wrong identity → TF chain is broken
- Fusion shows only one camera or nothing

### Why Dual Fusion is Rare

Even when both TF chains are valid, the timer-based merge at 15 Hz only produces a `dual` fusion when **both clouds are fresh** (arrived within the last 0.5s). The clouds arrive at ~4.5 Hz each, independently. The probability of both being fresh at any timer tick is roughly:

- Head cloud fresh: 0.5s window / 0.22s period ≈ 100% (almost always fresh)
- Arm cloud fresh: same ≈ 100%
- Both fresh simultaneously: high, BUT only if both TF chains resolve

The limiting factor is not cloud timing — it's **TF chain availability**. The stats show `dual` counts of 20-30 per 10s interval when things are working, which is about right (15 Hz timer × ~2 cameras = ~30 dual attempts, with some TF failures).

## Recommendations (Priority Order)

### 1. Fix the bridge to NOT depend on the aruco anchor for the primary path

**This is the highest-impact change.** The bridge should compute `head_cam0 -> head_d435i_head_link` directly from the VIO TF tree, without needing the `body_display` frame.

The OpenVINS tree publishes: `marker_map -> head_imu -> head_cam0`
The RealSense tree publishes: `head_d435i_head_link -> head_d435i_head_depth_frame -> head_d435i_head_depth_optical_frame`

The bridge needs to connect these. The correct approach:

- Look up `head_cam0 -> head_d435i_head_depth_optical_frame` directly. This is the full extrinsic chain from OpenVINS camera frame to RealSense depth optical frame. It's available as soon as both trees are connected (i.e., always, once OpenVINS starts).
- Compose: `T(head_cam0->link) = T(head_cam0->depth_optical) @ T(depth_optical->depth_frame)^{-1} @ T(depth_frame->link)^{-1}`

But wait — `head_cam0` and `head_d435i_head_depth_optical_frame` are in **different trees** until the bridge connects them. That's the chicken-and-egg problem.

The actual solution is simpler: **use the OpenVINS odom topic**. The odom message contains the pose of `head_cam0` in `marker_map` directly, stamped with the Jetson clock. The bridge could subscribe to `/ov_msckf/odomimu` and extract the pose, then compose it with the known static extrinsic to get `head_cam0 -> head_d435i_head_link`.

### 2. Use the odom topic for the bridge transform

Subscribe to `/ov_msckf/odomimu` (head) and `/ov_msckf_arm/odomimu` (arm). These contain:
- `header.frame_id = "marker_map"`
- `child_frame_id = "head_cam0"` (or `arm_cam0`)
- Full pose (position + orientation) at 200 Hz

The bridge can extract the pose from the odom message and compose it with the static RealSense extrinsic to produce `head_cam0 -> head_d435i_head_link`. This eliminates the dependency on the aruco `body_display` frame entirely.

### 3. Alternatively: make the bridge subscribe to the OpenVINS TF directly

The Jetson already publishes `marker_map -> head_imu -> head_cam0` at 200 Hz on `/tf`. The bridge already receives this. The problem is that `head_cam0 -> head_d435i_head_link` requires knowing the extrinsic between the OpenVINS camera frame and the RealSense link frame.

This extrinsic IS what the `body_display` frame encodes. When OpenVINS publishes `head_cam0 -> head_d435i_head_color_optical_frame_body_display`, it's saying "here's where the camera body is relative to the VIO estimate." The bridge uses this as the link transform.

**The key insight**: this extrinsic is **static** — it's the physical mounting offset between the VIO camera coordinate frame and the RealSense camera body. It only needs to be computed ONCE (when the marker is first detected) and then cached forever. The current 30-second cache limit is too conservative for a static quantity.

### 4. Simplest immediate fix: increase cache to effectively infinite for the extrinsic

The extrinsic between `head_cam0` and `head_d435i_head_link` is a **static property of the physical camera mount**. It should be computed once and never expire. The only reason it changes is if the camera physically moves on the robot.

Change `_matrix_staleness_limit_s` from 30.0 to something very large (e.g., 3600.0 = 1 hour), or better yet, make the cache permanent once a good anchor-based transform has been obtained.

### 5. Verify the fallback path is actually wrong

The fallback assumes `head_cam0 == head_d435i_head_depth_optical_frame`. If this assumption is close enough (the OpenVINS camera frame and RealSense depth optical frame are nearly coincident), the fallback might produce acceptable results. The coordinate convention difference (optical Y-down vs. ROS Y-up) would cause a 180° rotation error, which is definitely NOT acceptable. But if the `anchor_frame_mode` is `link`, the bridge uses the anchor transform directly without the optical rotation — so the fallback is the one that's wrong.

---

## Summary

| Question | Answer |
|---|---|
| Are the TFs coming at the wrong time? | No — the TFs arrive at good rates (200 Hz VIO, 15 Hz bridge). The problem is the bridge losing its anchor and falling back to a wrong transform. |
| Are we using the wrong frames? | No — `head_cam0` (VIO estimate) is the correct frame. The `body_display`/`_raw`/`_from_marker` frames are aruco diagnostics, not suitable for fusion. |
| What are the "behind" markers? | Likely `*_color_optical_frame_raw` and `*_from_marker` — raw aruco detections that lag behind the VIO estimate. Not used by the fusion pipeline. |
| Why is dual fusion intermittent? | The bridge's anchor frame (`body_display`) appears/disappears with marker visibility. When lost, the TF chain breaks after the cache expires. |
| What would fix it permanently? | (a) Make the extrinsic cache permanent (it's a static property), or (b) subscribe to the odom topic to compute the bridge transform without depending on the aruco anchor. |

## Implementation Plan

- [ ] **Task 1**: Make the bridge extrinsic cache permanent once computed from a valid anchor. Add a `_extrinsic_locked` flag per camera that, once set, never expires. The bridge still tries to update from the live anchor, but if the anchor disappears, it uses the locked extrinsic forever.
- [ ] **Task 2**: Add a diagnostic log that reports the extrinsic matrix once locked, so you can verify it's reasonable.
- [ ] **Task 3**: Consider subscribing to the odom topics (`/ov_msckf/odomimu`, `/ov_msckf_arm/odomimu`) as an alternative source for the bridge transform. This would eliminate the anchor dependency entirely.
- [ ] **Task 4**: Rebuild and test with the Jetson running. Verify that `dual` fusion is consistent and doesn't stall when markers are temporarily occluded.
