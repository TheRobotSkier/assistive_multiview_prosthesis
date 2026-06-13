# Localization Rework — V6: Evidence-Adjusted 4GB-Safe Architecture

**Date:** 2026-06-13
**Status:** V5 with recorded-data evidence integrated. Key assumptions re-graded based on rosbag inspection.
**Supersedes:** V5 plan (`plans/2026-06-12-localization-rework-plan-v5.md`)

---

## 0. What Changed From V5

V5 was a desk-review of the codebase. V6 incorporates **empirical evidence from the two recorded rosbags** (`rosbags/rosbag2_2026_05_21-16_37_45` and `rosbags/rosbag2_2026_05_21-16_48_20`). The bags are from an **earlier system revision** and recorded with a **deliberately selective topic set**, so their evidence is indicative, not definitive. Where the bags contradict V5 assumptions, V6 treats the assumption as **unverified** and promotes the fallback to a first-class design requirement.

| # | V5 stance | Rosbag evidence | V6 stance |
|---|-----------|-----------------|-----------|
| 1 | Organized clouds assumed (`height==480`), §5.5 | **All 4 clouds are `height==1` (unorganized)** in both bags | **Unverified.** Fallback path is now a mandatory design branch, not an afterthought. Verification gate is critical-path. |
| 2 | Keyframe ~5.5 MB (organized 640×480), §10 | Actual ~3.2 MB/keyframe (unorganized, ~113k pts) | Memory budget **relaxed to ~320 MB** for 100 keyframes. |
| 3 | Images assumed 30 Hz, §3.1 | Recorded at **6 Hz** | SIFT "process every 6th-10th frame" is wrong; node should process **every** synced pair. |
| 4 | Head + arm odom both assumed available | Only `/ov_msckf_arm/odomimu` present; head pose via TF only | Head odom topic **unverified on current system**. TF-derived head pose is a viable fallback. |
| 5 | ArUco topics assumed flowing over DDS | **Absent** from both bags | **Unverified.** Cannot confirm §2.1/§5.4 ArUco factor path without a fresh recording. |

**Important caveat:** The bags predate the current system and were recorded with a curated topic list. The absence of a topic in a bag does **not** mean it is absent on the live system — it may simply not have been recorded. Conversely, the unorganized-cloud finding is structural (a property of how the driver/pipeline publishes), so it is more likely to persist. Treat #1 as a strong signal and #4/#5 as "needs live confirmation."

### 0.1 New Action Items Introduced by V6

1. **Record a fresh comprehensive bag** capturing the full topic set in a single session (see §12).
2. **Treat the cloud as unorganized by default** in all code; organized structure is an optimization, not an assumption.
3. **Add a runtime cloud-organization probe** to the keyframe buffer that logs `height` at startup and switches strategy automatically.

---

## 1. Hardware Reality

*(Unchanged from V5.)*

| Resource | Available | Constraint |
|----------|-----------|------------|
| GPU VRAM | 4096 MiB | ~1.5 GB for OS/display, leaving ~2.5 GB usable |
| GPU compute | GTX 3050-class mobile | Continuous 15 Hz deep learning saturates the card |
| CPU | Laptop-class (8+ cores) | GTSAM, SIFT, Open3D all CPU-friendly |
| RAM | 16+ GB | Keyframe buffer ~320 MB at max capacity (see §9.2, corrected) |
| Network | USB gadget Ethernet (10.42.0.x) | RGB images at 640x480 already flow; JPEG compression optional |

### 1.1 VRAM Budget (Post-InterObject3D Removal)

*(Unchanged from V5.)* By replacing InterObject3D/MinkowskiEngine with MobileSAM:

| Component | VRAM | When |
|-----------|------|------|
| OS/display/WM | ~1.5 GB | Always |
| MobileSAM (image encoder) | ~300 MB | At grasp time only |
| SuperPoint + LightGlue | ~500 MB | Optional, 3-5 Hz steady state |
| **Total (with SuperPoint)** | ~2.3 GB | Fits in 2.5 GB budget |
| **Total (SIFT-only)** | ~1.8 GB | Ample headroom |

SuperPoint at 3 Hz is viable on 4GB AFTER removing InterObject3D. But SIFT at 3-5 Hz on CPU costs zero VRAM and should be tried first.

---

## 2. Verified Jetson-Side Topics and Messages

The Jetson runs `aruco_marker_pose_node.py` (in `docker_ws/multi_cam_localization/sensor_fusion_bringup/scripts/`). It publishes observation messages that flow over CycloneDDS to the laptop. These are **already available** — no new Jetson code needed.

> **V6 evidence note:** None of the ArUco observation topics below appear in either recorded bag. This is consistent with a selective recording, not proof of absence on the live system. **Live verification (§6.0) is mandatory before relying on these.**

### 2.1 ArUco Observation Topics (From Config)

From `head_aruco_map.yaml` and `arm_aruco_map.yaml`:

| Topic | Message Type | Source Node | Description |
|-------|-------------|-------------|-------------|
| `/head/marker_pose/observation` | `sensor_fusion_msgs/MarkerPoseObservation` | `aruco_marker_pose_node` (head) | Fixed world marker pose + covariance in `marker_map` |
| `/head/marker_pose/dynamic_observation` | `sensor_fusion_msgs/DynamicMarkerObservation` | `aruco_marker_pose_node` (head) | Dynamic arm marker pose + covariance in head camera frame |
| `/arm/marker_pose/observation` | `sensor_fusion_msgs/MarkerPoseObservation` | `aruco_marker_pose_node` (arm) | Fixed world marker pose + covariance in `marker_map` |
| `/arm/marker_pose/dynamic_arm_pose_observation` | `sensor_fusion_msgs/DynamicArmPoseObservation` | `dynamic_arm_pose_measurement_node` | Head-derived arm IMU pose + covariance in `marker_map` |

### 2.2 Message Definitions (from `sensor_fusion_msgs/msg/`)

*(Unchanged from V5.)*

**MarkerPoseObservation.msg** — contains:
- `header`, `marker_id`, `marker_frame`, `target_frame`
- `geometry_msgs/PoseWithCovariance pose` — T_map_imu with 6x6 covariance
- `bool hard_gate_passed`, `bool stable`, `int32 stable_frames`, `float64 stability_factor`
- `float64 reprojection_error_px`, `distance_m`, `view_angle_deg`, `area_px2`
- `float64 side_mean_px`, `side_min_px`, `geometry_score`, `covariance_sigma_px`

**DynamicMarkerObservation.msg** — contains:
- `header`, `marker_id`, `marker_frame`, `camera_frame`
- `geometry_msgs/PoseWithCovariance pose` — T_cam_marker with covariance
- Quality metrics same as above

**DynamicArmPoseObservation.msg** — contains:
- `header`, `marker_id`, `source_camera_frame`, `marker_frame`, `target_frame`
- `geometry_msgs/PoseWithCovariance pose` — T_map_armimu with covariance
- Head pose match metadata (`head_pose_match_dt_s`, `head_pose_match_mode`, etc.)

### 2.3 Dependency: `sensor_fusion_msgs` on Laptop

*(Unchanged from V5.)* The laptop workspace (`src/`) does not contain `sensor_fusion_msgs`. Recommendation: copy the package into the laptop workspace.

### 2.4 DDS Bridge Verification

*(Unchanged from V5, but elevated to a hard gate — see §6.0.)* The topics cross the DDS bridge because both machines run CycloneDDS with peer-to-peer discovery. **Action item:** Verify with `ros2 topic list` on the laptop during a live session that the observation topics are visible. If not, check CycloneDDS config (`config/cyclonedds_peer.xml`).

---

## 3. Architecture Overview

```
┌──────────────────────────────────────────────────────────────────────────────┐
│                              JETSON (unchanged)                               │
│                                                                              │
│  realsense2 → images + points + IMU + camera_info                            │
│  OpenVINS × 2 → /ov_msckf/odomimu, /ov_msckf_arm/odomimu                   │
│  aruco_marker_pose_node (head) → /head/marker_pose/observation              │
│                               → /head/marker_pose/dynamic_observation        │
│  aruco_marker_pose_node (arm)  → /arm/marker_pose/observation               │
│  dynamic_arm_pose_measurement  → /arm/marker_pose/dynamic_arm_pose_obs       │
│                                                                              │
│  All cross DDS bridge (CycloneDDS, host network mode) → laptop               │
└──────────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌──────────────────────────────────────────────────────────────────────────────┐
│                                 LAPTOP                                        │
│                                                                              │
│  ┌──────────────────────┐                                                     │
│  │ gtsam_trajectory_binder│ (NEW, CPU-only)                                   │
│  │                       │                                                   │
│  │  Inputs:              │                                                   │
│  │   /ov_msckf/odomimu   │  (head odom, decimated)                           │
│  │   /ov_msckf_arm/odom  │  (arm odom, decimated)                            │
│  │   /head/marker_pose/  │                                                   │
│  │     observation       │  (MarkerPoseObservation → PriorFactor)            │
│  │   /head/marker_pose/  │                                                   │
│  │     dynamic_observation│ (DynamicMarkerObservation → BetweenFactor)       │
│  │   /vis/head_arm_pose  │  ← optional (SIFT or SuperPoint)                 │
│  │                       │                                                   │
│  │  FixedLagSmoother:    │                                                   │
│  │   BetweenFactor(odom) │                                                   │
│  │   PriorFactor(ArUco)  │                                                   │
│  │   BetweenFactor(dyn.) │                                                   │
│  │   BetweenFactor(vis.) │  ← SIFT or SuperPoint                            │
│  │   RangeFactor(1.0m)   │                                                   │
│  │                       │                                                   │
│  │  Output: /gtsam/head_pose, /gtsam/arm_pose                               │
│  └──────────┬───────────┘                                                    │
│             │                                                                 │
│  ┌──────────▼───────────┐   ┌──────────────────────────┐                      │
│  │ keyframe_buffer      │   │ cross_camera_features    │ (NEW)                │
│  │  (NEW)               │   │  (NEW)                    │                      │
│  │                      │   │                           │                      │
│  │  Stores per camera:  │   │  Mode A (default): SIFT   │                      │
│  │   point cloud        │   │    CPU                    │                      │
│  │   RGB image          │   │    ApproxTime sync        │                      │
│  │   camera intrinsics  │   │                           │                      │
│  │   GTSAM-optimized    │   │  Mode B (upgrade):        │                      │
│  │     pose             │   │    SuperPoint + LightGlue │                      │
│  │                      │   │    GPU                    │                      │
│  │  UNORGANIZED-safe:   │   │                           │                      │
│  │   auto-detect height │   │  Both publish:            │                      │
│  │   project-to-2D      │   │    /vis/head_arm_pose     │                      │
│  │   fallback built-in  │   │    (PoseWithCovariance)   │                      │
│  └──────────┬───────────┘   └──────────────────────────┘                      │
│             │               ┌──────────────────────────┐                      │
│  ┌──────────▼───────────┐   │ segmentation_node        │ (MODIFIED)           │
│  │ tsdf_grasp_fusion    │   │                           │                      │
│  │  (NEW)               │   │  Replaces InterObject3D   │                      │
│  │                      │   │  with MobileSAM           │                      │
│  │  Trigger: service    │   │                           │                      │
│  │   call from twist    │   │  Runs at GRASP TIME only  │                      │
│  │   propagation        │   │                           │                      │
│  └──────────────────────┘   └──────────────────────────┘                      │
└──────────────────────────────────────────────────────────────────────────────┘
```

*(Architecture is unchanged from V5. The keyframe buffer box now explicitly notes the unorganized-safe design — see §5.5.)*

---

## 4. Key Architectural Decisions

### 4.1 SAM-First, Fuse-Second (Segment Each Keyframe Before TSDF)

*(Unchanged from V5.)*

Rationale: SAM runs on native camera images where it works best. Dilated masks (10-15 px) create a safety buffer against tracking errors. Multi-view consensus in TSDF naturally suppresses background.

### 4.2 MobileSAM Replaces InterObject3D

*(Unchanged from V5.)*

### 4.3 Segment at Grasp Time, Not Continuously

*(Unchanged from V5.)*

### 4.4 Hit Point Shifted Inward 1-2 cm

*(Unchanged from V5.)*

### 4.5 Cross-Camera Features: SIFT First, SuperPoint Optional

*(Unchanged from V5, with rate correction — see §5.10.)*

**Default (zero VRAM):** SIFT or AKAZE on CPU, running on keyframe RGB images.
**Upgrade (if SIFT match quality is poor):** SuperPoint + LightGlue on GPU.

---

## 5. Critical Integration Points (Updated in V6)

### 5.1 Pipeline Manager Integration

*(Unchanged from V5.)* The TSDF fusion node must publish to `/segmentation/object_cloud` (the topic `pipeline_manager_node.py:243` subscribes to).

### 5.2 Twist Propagation Flow Change

*(Unchanged from V5.)* New flow is TSDF service-driven; `twist_propagation_node.py` gains ~60 lines.

### 5.3 GTSAM Smoother Choice: FixedLagSmoother vs. Raw iSAM2

*(Unchanged from V5.)* Use `gtsam.FixedLagSmoother` with 15-second lag.

### 5.4 ArUco Factor Integration Details

*(Unchanged from V5.)*

> **V6 evidence note:** Arm odometry message structure is **confirmed** by the bags: `/ov_msckf_arm/odomimu` carries `frame=marker_map`, `child=arm_imu`, with a populated 6×6 pose covariance diagonal (~3.5e-5 to ~6.5e-5). This validates the GTSAM odometry between-factor and the noise-model-from-covariance approach. **Head odom (`/ov_msckf/odomimu`) was not recorded** — its structure is assumed identical; confirm live.

### 5.5 Organized Point Cloud Handling — REWORKED IN V6

**V5 assumed** `height == 480` (organized) with a fallback if `height == 1`.

**V6 finding:** Both recorded bags show `height == 1` (unorganized) on all four point cloud streams:
- Head cloud: `height=1`, `width≈113000`, `point_step=20`, fields `[x,y,z,rgb]`
- Arm cloud: `height=1`, `width≈117000`, `point_step=20`, fields `[x,y,z,rgb]`

**V6 design:** Treat the cloud as **unorganized by default**. The keyframe buffer and TSDF fusion node must work correctly with `height == 1`. If an organized cloud is detected at runtime (`height > 1`), use the faster direct-index path as an optimization.

**Runtime auto-detection (mandatory in keyframe buffer):**
```python
class KeyframeBufferNode(Node):
    def __init__(self):
        ...
        self._cloud_organized = None  # None = not yet detected

    def _on_cloud(self, msg, camera_id):
        # Detect organization once, then cache
        if self._cloud_organized is None:
            self._cloud_organized = (msg.height > 1)
            self.get_logger().info(
                f"{camera_id} cloud: height={msg.height} -> "
                f"{'ORGANIZED' if self._cloud_organized else 'UNORGANIZED'}"
            )
        ...
```

**Unorganized-cloud masking strategy (the default path):**
When the cloud is unorganized, there is no pixel↔3D correspondence. To apply a SAM mask to an unorganized cloud, project each 3D point into the camera image using `K` and `pose`, then test the projected pixel against the mask:

```python
def mask_unorganized_cloud(cloud_xyz, mask, K, pose):
    """
    cloud_xyz: (N, 3) unorganized points in marker_map frame
    mask: (H, W) binary SAM mask
    K: (3, 3) intrinsics
    pose: (4, 4) T_map_camera
    Returns: (N,) boolean keep array
    """
    H, W = mask.shape
    # Transform points into camera frame
    pose_inv = np.linalg.inv(pose)
    p_cam = (pose_inv @ np.hstack([cloud_xyz, np.ones((len(cloud_xyz), 1))]).T).T[:, :3]
    # Keep only points in front of camera
    in_front = p_cam[:, 2] > 0
    # Project to pixel coordinates
    uv = (K @ p_cam[in_front].T).T
    uv = uv[:, :2] / uv[:, 2:3]
    u = uv[:, 0].astype(np.int32)
    v = uv[:, 1].astype(np.int32)
    # Bounds check
    valid = (u >= 0) & (u < W) & (v >= 0) & (v < H)
    keep = np.zeros(len(cloud_xyz), dtype=bool)
    idx_in_front = np.where(in_front)[0]
    keep[idx_in_front[valid]] = mask[v[valid], u[valid]]
    return keep
```

This is vectorized (no per-point Python loop) and runs in a few ms for ~115k points.

**Organized-cloud fast path (optimization, if detected):**
```python
def mask_organized_cloud(cloud_xyz, mask):
    """cloud_xyz: (H, W, 3) organized. mask: (H, W)."""
    return cloud_xyz[mask]  # direct pixel indexing
```

**TSDF depth-image construction must also handle unorganized clouds.** Open3D's `ScalableTSDFVolume.integrate()` requires an RGBD image (a dense depth grid). With an unorganized cloud, build the depth image by rasterizing projected points:

```python
def build_depth_image(cloud_xyz, K, pose, H, W, mask=None):
    """Rasterize unorganized cloud into a depth image."""
    pose_inv = np.linalg.inv(pose)
    p_cam = (pose_inv @ np.hstack([cloud_xyz, np.ones((len(cloud_xyz), 1))]).T).T[:, :3]
    in_front = p_cam[:, 2] > 0
    if mask is not None:
        # pre-filter by mask to reduce work
        pass
    uv = (K @ p_cam[in_front].T).T
    uv = uv[:, :2] / uv[:, 2:3]
    u = np.clip(uv[:, 0].astype(np.int32), 0, W - 1)
    v = np.clip(uv[:, 1].astype(np.int32), 0, H - 1)
    depth = np.zeros((H, W), dtype=np.float32)
    # z-buffer: keep nearest point per pixel
    z = p_cam[in_front, 2]
    order = np.argsort(-z)  # far-to-near so near overwrites
    depth[v[order], u[order]] = z[order]
    return depth
```

### 5.6 TSDF Integration Detail: RGBD Image Construction

*(Updated in V6 to be unorganized-safe — see §5.5 above. The V5 per-pixel loop assumed organized clouds and is replaced by the vectorized rasterization in §5.5.)*

### 5.7 SAM Cold Start Latency

*(Unchanged from V5.)* Pre-load the MobileSAM model at node startup.

### 5.8 Cross-Camera Feature Synchronization

*(Unchanged from V5.)* Use `message_filters.ApproximateTimeSynchronizer` with 50ms slop.

### 5.9 Segmentation Bridge End State

*(Unchanged from V5.)*

### 5.10 Image Rate Correction — NEW IN V6

**V5 assumed** RGB images at 30 Hz and advised "process every 6th-10th frame pair."

**V6 finding:** The recorded bags show RGB images at **~6 Hz** (head: 5.98 Hz, arm: 6.15 Hz). Point clouds in the same bag run at ~2 Hz.

**V6 design:** The SIFT node should process **every** synchronized frame pair — there is no surplus to decimate. Set `process_rate_hz: 5.0` and do not skip frames. If the live system publishes faster, reintroduce decimation. Log the actual input rate at startup so the operator knows.

> **Caveat:** Image rate is a driver/recording setting and may differ on the current system. Treat 6 Hz as the observed floor; verify live.

---

## 6. Implementation Plan

### Phase 1: Foundation (Week 1)

#### 6.0 Pre-requisites and Live Verification (Day 1, Morning) — EXPANDED IN V6

This phase now includes **mandatory live verification gates** because the recorded bags cannot confirm several critical topics.

1. **Copy `sensor_fusion_msgs` to laptop workspace:**
   ```
   cp -r ../worktrees/.../docker_ws/multi_cam_localization/sensor_fusion_msgs/ src/sensor_fusion_msgs/
   colcon build --packages-select sensor_fusion_msgs
   ```

2. **Verify ArUco topics on laptop (CRITICAL — unverified by bags):**
   ```bash
   ros2 topic list | grep marker_pose
   ros2 topic echo /head/marker_pose/observation --once
   ros2 topic echo /arm/marker_pose/dynamic_arm_pose_observation --once
   ```
   If absent, this blocks the entire ArUco factor path (§5.4). Debug CycloneDDS peers before proceeding.

3. **Verify BOTH odometry topics (head odom unverified by bags):**
   ```bash
   ros2 topic echo /ov_msckf/odomimu --field header --once      # HEAD — verify!
   ros2 topic echo /ov_msckf_arm/odomimu --field header --once  # arm — confirmed structure
   ```

4. **Verify point cloud organization (CRITICAL — bags show unorganized):**
   ```bash
   ros2 topic echo /head/d435i_head/depth/color/points --field height --once
   ros2 topic echo /arm/d435i_arm/depth/color/points --field height --once
   ```
   - If `height == 1`: unorganized path is the default (expected based on bags). Proceed with §5.5 unorganized design.
   - If `height == 480`: organized fast-path is available as an optimization.

5. **Record a fresh comprehensive bag** (see §12 for the full topic list). This becomes the new reference dataset for all downstream integration testing.

6. **Install GTSAM Python bindings:**
   ```bash
   pip install gtsam
   ```

#### 6.1 Kinematic Distance Check in Twist Propagation (Day 1)

*(Unchanged from V5.)* ~30 lines in `twist_propagation_node.py`.

#### 6.2 Keyframe Buffer Node (Day 2-5)

**Package:** `src/keyframe_buffer/`

```
src/keyframe_buffer/
├── keyframe_buffer/
│   ├── __init__.py
│   ├── keyframe_buffer_node.py    # Main node (~400 lines, +50 for org detection)
│   ├── keyframe.py               # Keyframe dataclass
│   └── cloud_utils.py            # NEW: unorganized/organized cloud handling
├── test/
│   ├── test_keyframe_buffer.py
│   └── test_cloud_utils.py       # NEW: test both organized + unorganized paths
├── setup.py
└── package.xml
```

**Storage per keyframe (V6 — unorganized-aware):**
```python
@dataclass
class Keyframe:
    timestamp: float
    camera_id: str              # "head" or "arm"
    cloud_xyz: np.ndarray       # (N, 3) — unorganized by default
    cloud_rgb: np.ndarray       # (N, 3) per-point RGB
    image: np.ndarray           # (H, W, 3) RGB image for SAM
    K: np.ndarray               # (3, 3) camera intrinsics
    pose: np.ndarray            # (4, 4) T_marker_map→camera from GTSAM
    organized: bool             # True if cloud is (H, W, 3), False if (N, 3)
```

**Subscriptions (INDEPENDENT per camera — NOT synchronized):**

> **CRITICAL DESIGN RULE (unchanged):** The keyframe buffer MUST subscribe to raw, un-synchronized individual camera topics independently. Do NOT use `ApproximateTimeSynchronizer`. During rapid arm movement, temporal offset between cameras can exceed 50ms. If gated by a synchronizer, it silently drops keyframes during the most critical phase of a reach.

- `/head/d435i_head/depth/color/points` — **independent subscription**
- `/arm/d435i_arm/depth/color/points` — **independent subscription**
- `/head/d435i_head/color/image_raw` — **independent subscription**
- `/arm/d435i_arm/color/image_raw` — **independent subscription**
- `/head/d435i_head/color/camera_info`
- `/arm/d435i_arm/color/camera_info`
- `/gtsam/head_pose`
- `/gtsam/arm_pose`

**Spatial gate (per camera, independent):** unchanged from V5 (10cm / 15°).

**Eviction:** Max 50 keyframes per camera, oldest first.

**Service:** `GetKeyframesInROI(center, radius) → list of Keyframe`.

#### 6.3 GTSAM Trajectory Binder (Day 3-7)

*(Unchanged from V5.)* Four factor types: odometry between-factors, ArUco prior-factors, visual between-factors, kinematic range factor. FixedLagSmoother with 15s lag.

> **V6 note:** If head odom (`/ov_msckf/odomimu`) is unavailable on the live system, the head branch of the graph can be driven by TF-derived pose (`marker_map → head_imu`) instead. Add a parameter `head_pose_source: "odom" | "tf"` and a TF-listener fallback. The bags confirm `marker_map → head_imu` is published dynamically at high rate.

### Phase 2: Segmentation Pivot (Week 2)

#### 6.4 MobileSAM Inference Server (Day 1-3)

*(Unchanged from V5.)* New Dockerfile for Python 3.10+, PyTorch 2.x, MobileSAM. Takes RGB image + 2D click, returns 2D binary mask.

#### 6.5 TSDF Grasp Fusion Node (Day 3-5)

**Package:** `src/tsdf_fusion/` — structure unchanged from V5.

**Pipeline on trigger (V6 — unorganized-safe):**

```python
def on_grasp_trigger(request):
    hit_point_3d = np.array([request.hit_point.x, request.hit_point.y, request.hit_point.z])
    roi_radius = request.roi_radius or 0.15

    # 1. Shift hit point inward along camera ray
    hit_internal = shift_inward(hit_point_3d, camera_ray, offset=0.015)

    # 2. Query keyframe buffer for frames near ROI
    keyframes = keyframe_buffer.get_in_roi(hit_internal, roi_radius * 1.5)

    # 3. Initialize TSDF volume
    volume = o3d.pipelines.integration.ScalableTSDFVolume(
        voxel_length=0.005, sdf_trunc=0.02,
        color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8)

    # 4. For each keyframe:
    for kf in keyframes:
        H, W = kf.image.shape[:2]

        # a. Project 3D hit point to 2D
        uv = project_3d_to_2d(hit_internal, kf.K, kf.pose)

        # b. Run MobileSAM on the keyframe's RGB image
        mask = sam_segment(kf.image, uv, dilation_px=15)

        if kf.organized:
            # Fast path: direct pixel indexing
            masked_xyz = kf.cloud_xyz[mask]
            masked_rgb = kf.cloud_rgb[mask]
        else:
            # Default path (expected based on bags): project cloud to 2D
            keep = mask_unorganized_cloud(kf.cloud_xyz, mask, kf.K, kf.pose)
            masked_xyz = kf.cloud_xyz[keep]
            masked_rgb = kf.cloud_rgb[keep]

        # c. Build depth image by rasterizing masked points
        depth_image = build_depth_image(masked_xyz, kf.K, kf.pose, H, W)

        # d. Create RGBD image (RGB from keyframe, depth from rasterized cloud)
        rgb_image = kf.image.copy()
        o3d_rgb = o3d.geometry.Image(rgb_image.astype(np.uint8))
        o3d_depth = o3d.geometry.Image(depth_image)
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            o3d_rgb, o3d_depth, depth_scale=1.0, depth_trunc=0.5,
            convert_rgb_to_intensity=False)

        # e. Integrate into TSDF
        intrinsic = o3d.camera.PinholeCameraIntrinsic(
            W, H, float(kf.K[0, 0]), float(kf.K[1, 1]),
            float(kf.K[0, 2]), float(kf.K[1, 2]))
        volume.integrate(rgbd, intrinsic, np.linalg.inv(kf.pose))

    # 5-7. Extract, DBSCAN cleanup, publish (unchanged from V5)
    ...
```

### Phase 3: Cross-Camera Features (Week 2-3)

#### 6.6 SIFT Feature Node (Default, Day 1-3)

*(Largely unchanged from V5, with rate correction from §5.10.)* Process every synced frame pair (observed ~6 Hz, not 30 Hz). The Umeyama 3D-3D alignment requires depth lookups; with unorganized clouds, use the projection-based correspondence (project SIFT keypoint pixel → nearest 3D point) rather than direct `cloud[v,u]` indexing.

```python
def _on_synced_frames(self, head_img_msg, arm_img_msg,
                      head_cloud_msg, arm_cloud_msg):
    head_img = bridge.imgmsg_to_cv2(head_img_msg, "rgb8")
    arm_img = bridge.imgmsg_to_cv2(arm_img_msg, "rgb8")

    # Parse cloud (handles both organized and unorganized)
    head_cloud, head_org = parse_cloud(head_cloud_msg)
    arm_cloud, arm_org = parse_cloud(arm_cloud_msg)

    # Extract + match SIFT
    kp_head, des_head = self._sift.detectAndCompute(head_img, None)
    kp_arm, des_arm = self._sift.detectAndCompute(arm_img, None)
    matches = self._matcher.match(des_head, des_arm)
    matches = sorted(matches, key=lambda m: m.distance)[:100]

    # Depth lookup — organization-aware
    valid_matches = []
    for m in matches:
        u_h, v_h = int(kp_head[m.queryIdx].pt[0]), int(kp_head[m.queryIdx].pt[1])
        u_a, v_a = int(kp_arm[m.trainIdx].pt[0]), int(kp_arm[m.trainIdx].pt[1])
        p3d_h = lookup_depth(head_cloud, head_org, u_h, v_h, self._head_K)
        p3d_a = lookup_depth(arm_cloud, arm_org, u_a, v_a, self._arm_K)
        if p3d_h is not None and p3d_a is not None:
            valid_matches.append((p3d_h, p3d_a))

    if len(valid_matches) >= 5:
        T, covariance = umeyama(valid_matches)
        publish_pose(T, covariance)
```

#### 6.7 SuperPoint Upgrade Path (Optional)

*(Unchanged from V5.)*

### Phase 4: Integration and Tuning (Week 3-4)

#### 6.8 Launch File Updates

*(Unchanged from V5.)* Add gtsam_tracker, cross_camera_features, keyframe_buffer, conditional tsdf_fusion/segmentation_bridge.

#### 6.9 Configuration

**File:** `config/prosthesis_config.yaml` additions (V6 changes marked):

```yaml
gtsam_tracker:
  ros__parameters:
    head_odom_topic: "/ov_msckf/odomimu"
    arm_odom_topic: "/ov_msckf_arm/odomimu"
    head_pose_source: "odom"          # V6 NEW: "odom" or "tf" (tf fallback if head odom absent)
    aruco_marker_topic: "/head/marker_pose/observation"
    aruco_dynamic_topic: "/head/marker_pose/dynamic_observation"
    aruco_arm_pose_topic: "/arm/marker_pose/dynamic_arm_pose_observation"
    graph_rate_hz: 15.0
    smoother_lag_s: 15.0
    kinematic_range_m: 1.0
    kinematic_range_sigma_m: 0.05
    odom_translation_sigma_m: 0.01
    odom_rotation_sigma_rad: 0.01

cross_camera_features:
  ros__parameters:
    head_image_topic: "/head/d435i_head/color/image_raw"
    arm_image_topic: "/arm/d435i_arm/color/image_raw"
    head_cloud_topic: "/head/d435i_head/depth/color/points"
    arm_cloud_topic: "/arm/d435i_arm/depth/color/points"
    head_info_topic: "/head/d435i_head/color/camera_info"
    arm_info_topic: "/arm/d435i_arm/color/camera_info"
    process_rate_hz: 5.0              # V6: was implicit 30 Hz assumption; 6 Hz observed
    sync_slop_s: 0.05
    min_matches: 5
    backend: "sift"

keyframe_buffer:
  ros__parameters:
    head_image_topic: "/head/d435i_head/color/image_raw"
    arm_image_topic: "/arm/d435i_arm/color/image_raw"
    head_cloud_topic: "/head/d435i_head/depth/color/points"
    arm_cloud_topic: "/arm/d435i_arm/depth/color/points"
    head_info_topic: "/head/d435i_head/color/camera_info"
    arm_info_topic: "/arm/d435i_arm/color/camera_info"
    head_pose_topic: "/gtsam/head_pose"
    arm_pose_topic: "/gtsam/arm_pose"
    max_keyframes_per_camera: 50
    spatial_gate_translation_m: 0.10
    spatial_gate_rotation_deg: 15.0
    expect_organized_clouds: false    # V6 NEW: default false; auto-detected at runtime

tsdf_fusion:
  ros__parameters:
    keyframe_service: "/keyframe_buffer/get_in_roi"
    sam_inference_url: "http://127.0.0.1:5679"
    sam_weights_path: "/weights/mobile_sam.pt"
    roi_radius_m: 0.15
    hit_point_shift_m: 0.015
    mask_dilation_px: 15
    voxel_size_m: 0.005
    sdf_trunc_m: 0.02
    dbscan_eps_m: 0.02
    dbscan_min_points: 10
    output_topic: "/segmentation/object_cloud"

twist_propagation:
  ros__parameters:
    max_head_wrist_distance_m: 1.0
    enforce_kinematic_constraint: false
    use_tsdf_fusion: false
    tsdf_fusion_service: "/tsdf_fusion/trigger"
```

#### 6.10 Validation Gates (V6 — evidence-graded)

Each gate is now annotated with whether the recorded bags can pre-validate it.

| Gate | What to check | Tool | Bag-pre-validable? |
|------|--------------|------|-------------------|
| **ArUco topics visible** | `/head/marker_pose/observation` echoes on laptop | `ros2 topic echo` | No — absent from bags |
| **Head odom visible** | `/ov_msckf/odomimu` echoes on laptop | `ros2 topic echo` | No — absent from bags |
| **Organized clouds** | `height` on both camera point clouds | `ros2 topic echo ... --field height` | **Partially** — bags show `height==1`; confirm live |
| **Arm odom structure** | `/ov_msckf_arm/odomimu` has covariance | `ros2 topic echo` | **Yes** — confirmed `marker_map`/`arm_imu` + 6×6 cov |
| **GTSAM correctness** | Pose output matches raw odometry within 2cm | `ros2 topic echo` | No — needs head odom |
| **GTSAM drift suppression** | Arm pose stays within 1.0m of head over 60s | Record + plot | No — no simultaneous head+arm odom in any bag |
| **GTSAM marginalization** | Memory stable over 10+ minutes | Monitor rate | No |
| **SIFT match quality** | ≥ 5 valid matches with < 3cm RMS per pair | Node diagnostics | No — no synced image+odom+cloud bag |
| **Keyframe buffer latency** | < 5ms to insert a new keyframe | ROS timers | Yes (offline replay) |
| **Keyframe buffer memory** | < 350 MB at max capacity | Diagnostics topic | Yes (offline replay) |
| **SAM latency (burst)** | < 300ms for 20 keyframes | Timestamp tracking | Yes (offline replay with images) |
| **SAM cold start** | < 5s from node start to model ready | Startup log | Yes |
| **TSDF quality** | Fused cloud < 2cm RMS vs ground truth | Compare to reference | No — needs full topic set |
| **End-to-end hit-to-cloud** | < 500ms from hit to segmented cloud | Pipeline timestamps | No — needs full topic set |
| **Pipeline manager transition** | State machine reaches PRESHAPING | `ros2 topic echo /pipeline/state_name` | No |

---

## 7. What Changes vs. Current Code

*(Unchanged from V5.)*

### 7.1 New Packages

| Package | Lines (est.) | Purpose |
|---------|-------------|---------|
| `sensor_fusion_msgs` | ~50 (copied) | Message definitions for ArUco observations |
| `gtsam_tracker` | ~780 + ~280 tests | GTSAM factor graph + FixedLagSmoother (+TF fallback) |
| `keyframe_buffer` | ~450 + ~200 tests | Spatial-gated storage; unorganized-safe cloud handling |
| `tsdf_fusion` | ~500 + ~200 tests + ~20 srv | On-demand TSDF fusion; unorganized-safe rasterization |
| `cross_camera_features` | ~420 + ~150 tests | SIFT/SuperPoint; organization-aware depth lookup |

### 7.2 Modified Existing Files

*(Unchanged from V5.)*

### 7.3 Unchanged

*(Unchanged from V5.)*

---

## 8. Implementation Sequence

```
Week 1 (Phase 1):
  Mon AM: Pre-requisites + LIVE VERIFICATION GATES (§6.0) + record fresh bag (§12)
  Mon PM: Kinematic constraint in twist_propagation (30 lines)
  Tue-Thu: Keyframe buffer node + cloud_utils (unorganized-safe) + tests
  Wed-Fri: GTSAM Trajectory Binder (overlaps with keyframe buffer)
  Fri:   Integration test: GTSAM + keyframe buffer on FRESH recorded rosbag

Week 2 (Phase 2):
  Mon-Wed: MobileSAM inference server (replace InterObject3D)
  Wed-Fri: TSDF grasp fusion node (unorganized-safe rasterization)
  Fri:    Integration test: full SAM + TSDF pipeline on FRESH rosbag

Week 3 (Phase 3 + 4 start):
  Mon-Wed: SIFT feature node (process every frame, ~6 Hz)
  Thu:    Twist propagation TSDF service integration (+60 lines)
  Fri:    Launch file integration, end-to-end testing on FRESH rosbag

Week 4 (Phase 4 finish):
  Mon-Tue: Noise model tuning for GTSAM (with real data)
  Wed:    Dry-run on hardware (no grasp execution, just data flow)
  Thu:    Hardware deployment, latency measurement, tuning
  Fri:    Validation: full pipeline with real grasp execution
```

---

## 9. Risk Assessment (V6 — updated)

| Risk | Likelihood | Impact | Mitigation |
|------|-----------|--------|------------|
| **Clouds are unorganized (height==1)** | **High** (confirmed in bags) | Medium | **Designed for in V6.** Unorganized path is default; organized is optimization. See §5.5. |
| SIFT fails head-to-wrist viewpoint (too few matches) | Medium | High | Upgrade to SuperPoint at 3-5 Hz |
| MobileSAM latency exceeds budget (>300ms for 20 keyframes) | Low | Medium | Run SAM in-process; reduce keyframe count |
| GTSAM optimization takes > 67ms (15 Hz deadline missed) | Low | Medium | Reduce lag to 10s; decimate to 10 Hz |
| **ArUco topics not discoverable on laptop via DDS** | **Unknown** (not in bags) | **High** | Verify first (§6.0). If missing, configure CycloneDDS peers. ArUco is the primary drift-correction source. |
| **Head odom (`/ov_msckf/odomimu`) absent on live system** | **Unknown** (not in bags) | Medium | TF fallback (`marker_map → head_imu`) confirmed available in bags. Add `head_pose_source: "tf"` mode. |
| Keyframe buffer memory growth over long sessions | Low | Low | Hard limit 50/camera (~320 MB max); oldest-first eviction |
| `sensor_fusion_msgs` build fails on laptop | Low | Low | Pure CMake message generation |
| GTSAM Python bindings unavailable or incompatible | Low | Medium | Build from source; fallback to custom EKF |
| TSDF integration produces artifacts from mask dilation | Low | Low | DBSCAN cleanup; conservative dilation |
| Keyframe buffer drops frames during rapid arm movement | **AVOIDED** | N/A | Buffer subscribes to raw per-camera topics independently (no synchronizer) |
| **Stale-bag assumptions mislead development** | Medium | Medium | All bag-derived findings tagged "V6 evidence note." Fresh bag (§12) recorded before Week 1 coding. |

---

## 10. Keyframe Buffer Memory Budget (V6 — corrected with real data)

V5 estimated ~5.5 MB/keyframe assuming organized 640×480 clouds. The recorded bags show **unorganized** clouds with ~113,000–118,000 points per frame:

| Component | V5 estimate | V6 measured (from bags) |
|-----------|-------------|------------------------|
| Cloud XYZ | 640×480×3×4 = 3.7 MB | ~115,000 × 3 × 4 = **1.4 MB** |
| Cloud RGB | 640×480×3×1 = 0.9 MB | ~115,000 × 3 × 1 = **0.35 MB** |
| RGB image | 0.9 MB | 0.9 MB |
| **Total/keyframe** | **5.5 MB** | **~2.7 MB** |

At max capacity (50 keyframes × 2 cameras = 100 keyframes):
- **V6 total: ~270–330 MB** (vs V5's ~550 MB estimate)

This is comfortably within the 16 GB RAM budget. Add a ROS diagnostics publisher reporting current memory usage.

> **Note:** Point count varies per frame (~113k–118k observed). Use the measured maximum (~120k) for worst-case budgeting.

---

## 11. Summary of Architecture Evolution

| Version | Core Insight | Why Evolved |
|---------|-------------|-------------|
| V1 | TSDF + keyframe buffer can produce better fused clouds | Pose accuracy wasn't addressed |
| V2 | GTSAM needed for retroactive smoothing; defer visual features | Only ArUco for cross-camera — not continuous enough |
| V3 | RGB images available; SuperPoint feasible on dev GPU | Assumed 12GB VRAM; not deployment-realistic |
| V4 | 4GB constraint is real; SAM-first/fuse-second is correct; segment at grasp time not continuously | Matches hardware reality and architectural clarity |
| V5 | Integration gaps with pipeline_manager, twist_propagation, ArUco topics, Open3D API resolved; FixedLagSmoother; organized cloud verification added | Comprehensive review against actual codebase |
| **V6** | **Recorded-data evidence integrated. Clouds confirmed unorganized (height==1) — unorganized-safe design is now default, not fallback. Memory budget corrected (~320 MB). Image rate corrected (6 Hz). Head odom + ArUco topics flagged unverified. Fresh comprehensive bag required before coding.** | **Empirical validation against two recorded rosbags, with stale-bag caveats** |

---

## 12. Fresh Comprehensive Bag Recording Specification — NEW IN V6

The existing bags cannot validate the full pipeline (missing head odom, ArUco topics, and synchronized images+clouds+odom). **Before Week 1 coding, record a new bag** with the complete topic set in a single session.

**Required topics (record ALL in one bag):**

```
# Odometry (both cameras)
/ov_msckf/odomimu
/ov_msckf_arm/odomimu

# ArUco observations (all four)
/head/marker_pose/observation
/head/marker_pose/dynamic_observation
/arm/marker_pose/observation
/arm/marker_pose/dynamic_arm_pose_observation

# Head camera
/head/d435i_head/color/image_raw
/head/d435i_head/color/camera_info
/head/d435i_head/depth/color/points
/head/d435i_head/depth/image_rect_raw
/head/d435i_head/depth/camera_info
/head/d435i_head/imu

# Arm camera
/arm/d435i_arm/color/image_raw
/arm/d435i_arm/color/camera_info
/arm/d435i_arm/depth/color/points
/arm/d435i_arm/depth/image_rect_raw
/arm/d435i_arm/depth/camera_info
/arm/d435i_arm/imu

# TF
/tf
/tf_static
```

**Recording scenario:** A representative reach-to-grasp sequence including:
1. ~10s static (for drift baseline)
2. Slow arm movement toward an object (~10s)
3. A grasp attempt (~5s)
4. ~10s static post-grasp
5. At least one instance where an ArUco marker is visible to both cameras

**Target duration:** 60–90 seconds. This becomes the reference dataset for all Phase 1–4 integration tests.

```bash
# Example recording command (adjust topic list as needed)
ros2 bag record -o rosbags/reference_$(date +%Y_%m_%d-%H_%M_%S) \
  /ov_msckf/odomimu /ov_msckf_arm/odomimu \
  /head/marker_pose/observation /head/marker_pose/dynamic_observation \
  /arm/marker_pose/observation /arm/marker_pose/dynamic_arm_pose_observation \
  /head/d435i_head/color/image_raw /head/d435i_head/color/camera_info \
  /head/d435i_head/depth/color/points /head/d435i_head/imu \
  /arm/d435i_arm/color/image_raw /arm/d435i_arm/color/camera_info \
  /arm/d435i_arm/depth/color/points /arm/d435i_arm/imu \
  /tf /tf_static
```

---

## Appendix A: Recorded Bag Evidence Summary

For traceability, here is exactly what the two existing bags contain and what they confirm.

### Bag 1: `rosbag2_2026_05_21-16_37_45` (65.6s, 4.4 GB)

| Topic | Type | Rate | Notes |
|-------|------|------|-------|
| `/head/d435i_head/depth/color/points` | PointCloud2 | 14.8 Hz | **height=1** (unorganized), ~113k pts, fields `[x,y,z,rgb]` |
| `/arm/d435i_arm/depth/color/points` | PointCloud2 | 14.7 Hz | **height=1** (unorganized), ~118k pts |
| `/ov_msckf_arm/odomimu` | Odometry | 141.6 Hz | `frame=marker_map`, `child=arm_imu`, 6×6 cov populated |
| `/tf` | TFMessage | 706.9 Hz | Dynamic: `marker_map→head_imu`, `marker_map→arm_imu`, `marker_map→marker_0` |
| `/tf_static` | TFMessage | — | Camera mount chains (link→color/depth/gyro/accel frames) |

### Bag 2: `rosbag2_2026_05_21-16_48_20` (50.7s, 1.4 GB)

| Topic | Type | Rate | Notes |
|-------|------|------|-------|
| `/head/d435i_head/color/image_raw` | Image | 6.0 Hz | 640×480 rgb8 |
| `/arm/d435i_arm/color/image_raw` | Image | 6.2 Hz | 640×480 rgb8 |
| `/head/d435i_head/color/camera_info` | CameraInfo | 19.9 Hz | K confirmed, `plumb_bob` |
| `/arm/d435i_arm/color/camera_info` | CameraInfo | 19.7 Hz | K confirmed |
| `/head/d435i_head/depth/color/points` | PointCloud2 | 1.9 Hz | **height=1** (unorganized) |
| `/arm/d435i_arm/depth/color/points` | PointCloud2 | 2.0 Hz | **height=1** (unorganized) |
| `/head/d435i_head/imu` | Imu | 130.3 Hz | |
| `/arm/d435i_arm/imu` | Imu | 130.3 Hz | |
| `/head/d435i_head/depth/image_rect_raw` | Image | 5.7 Hz | |
| `/arm/d435i_arm/depth/image_rect_raw` | Image | 6.3 Hz | |
| `/tf` | TFMessage | 611.5 Hz | Same dynamic chains as Bag 1 |

**Bags are sequential (569s gap), zero temporal overlap — cannot be merged.**

### What the bags confirm
- Point clouds are **unorganized** (`height=1`) across both bags and both cameras.
- Arm odometry message format and covariance are valid for GTSAM between-factors.
- RGB images are 640×480 `rgb8` with valid intrinsics.
- `marker_map → head_imu` and `marker_map → arm_imu` TF transforms are published dynamically (head pose recoverable via TF).
- Keyframe memory per frame is ~2.7 MB (not 5.5 MB).

### What the bags do NOT confirm (due to selective recording / older system)
- Head odometry topic (`/ov_msckf/odomimu`) — not recorded.
- Any ArUco observation topic — not recorded.
- Synchronized images + clouds + odom in a single session — the two bags split these across separate recordings.
