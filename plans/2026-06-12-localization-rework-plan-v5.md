# Localization Rework — V5: Comprehensive 4GB-Safe Architecture

**Date:** 2026-06-12
**Status:** V4 with all review findings incorporated. Verified against Jetson worktree.
**Supersedes:** V4 plan (`plans/2026-06-12-localization-rework-plan-v4.md`)

---

## 0. What Survived From Earlier Plans

| From | Kept? | Why |
|------|-------|-----|
| V1 (original) TSDF + keyframe buffer | Yes | Core fusion mechanism |
| V2 GTSAM Trajectory Binder | Yes, modified | CPU-only GTSAM with between factors |
| V3 SuperPoint + LightGlue | Downgraded to optional | 4GB VRAM makes 15 Hz impossible; 3 Hz may be viable |
| V3 Visual feature between factors | Yes | SIFT/AKAZE on CPU as default; SuperPoint as upgrade |

## 1. Hardware Reality

| Resource | Available | Constraint |
|----------|-----------|------------|
| GPU VRAM | 4096 MiB | ~1.5 GB for OS/display, leaving ~2.5 GB usable |
| GPU compute | GTX 3050-class mobile | Continuous 15 Hz deep learning saturates the card |
| CPU | Laptop-class (8+ cores) | GTSAM, SIFT, Open3D all CPU-friendly |
| RAM | 16+ GB | Keyframe buffer ~530 MB at max capacity (see Section 9.2) |
| Network | USB gadget Ethernet (10.42.0.x) | RGB images at 640x480 already flow; JPEG compression optional |

### 1.1 VRAM Budget (Post-InterObject3D Removal)

By replacing InterObject3D/MinkowskiEngine with MobileSAM:

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

### 2.1 ArUco Observation Topics (Verified from Config)

From `head_aruco_map.yaml` and `arm_aruco_map.yaml`:

| Topic | Message Type | Source Node | Description |
|-------|-------------|-------------|-------------|
| `/head/marker_pose/observation` | `sensor_fusion_msgs/MarkerPoseObservation` | `aruco_marker_pose_node` (head) | Fixed world marker pose + covariance in `marker_map` |
| `/head/marker_pose/dynamic_observation` | `sensor_fusion_msgs/DynamicMarkerObservation` | `aruco_marker_pose_node` (head) | Dynamic arm marker pose + covariance in head camera frame |
| `/arm/marker_pose/observation` | `sensor_fusion_msgs/MarkerPoseObservation` | `aruco_marker_pose_node` (arm) | Fixed world marker pose + covariance in `marker_map` |
| `/arm/marker_pose/dynamic_arm_pose_observation` | `sensor_fusion_msgs/DynamicArmPoseObservation` | `dynamic_arm_pose_measurement_node` | Head-derived arm IMU pose + covariance in `marker_map` |

### 2.2 Message Definitions (from `sensor_fusion_msgs/msg/`)

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

**The `sensor_fusion_msgs` package is NOT currently built on the laptop.** The laptop workspace (`src/`) does not contain it. The GTSAM tracker needs to subscribe to these message types. Options:

1. **Copy `sensor_fusion_msgs` into the laptop workspace** — simplest, just copy the `msg/` directory, `CMakeLists.txt`, and `package.xml` from the Jetson worktree.
2. **Build a shared package** — create a git submodule or shared package.

**Recommendation:** Option 1 for now. Add `sensor_fusion_msgs` as a package in the laptop workspace. The message definitions are stable and unlikely to change.

### 2.4 DDS Bridge Verification

The topics cross the DDS bridge because:
- Both machines run CycloneDDS with peer-to-peer discovery over USB gadget Ethernet.
- The Jetson publishes these topics; the laptop already subscribes to `/ov_msckf/odomimu` and camera topics successfully over the same bridge.
- The `aruco_marker_pose_node` uses `QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)` for images and `QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=50)` for odometry — these should cross DDS without issue.
- **Action item:** Verify with `ros2 topic list` on the laptop during a live session that the observation topics are visible. If not, check CycloneDDS config (`config/cyclonedds_peer.xml`).

---

## 3. Architecture Overview

```
┌──────────────────────────────────────────────────────────────────────────────┐
│                              JETSON (unchanged)                               │
│                                                                              │
│  realsense2 → images + organized points + IMU + camera_info                  │
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
│  │   /ov_msckf/odomimu   │  (head odom, 200 Hz → decimated 15 Hz)           │
│  │   /ov_msckf_arm/odom  │  (arm odom, 200 Hz → decimated 15 Hz)            │
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
│  │  Output: /gtsam/head_pose, /gtsam/arm_pose (15 Hz)                        │
│  └──────────┬───────────┘                                                    │
│             │                                                                 │
│  ┌──────────▼───────────┐   ┌──────────────────────────┐                      │
│  │ keyframe_buffer      │   │ cross_camera_features    │ (NEW)                │
│  │  (NEW)               │   │  (NEW)                    │                      │
│  │                      │   │                           │                      │
│  │  Stores per camera:  │   │  Mode A (default): SIFT   │                      │
│  │   organized pcl      │   │    CPU, 3-5 Hz            │                      │
│  │   RGB image          │   │    ApproxTime sync        │                      │
│  │   camera intrinsics  │   │                           │                      │
│  │   GTSAM-optimized    │   │  Mode B (upgrade):        │                      │
│  │     pose             │   │    SuperPoint + LightGlue │                      │
│  │                      │   │    GPU, 3-5 Hz            │                      │
│  │  Spatial gate:       │   │                           │                      │
│  │   10cm / 15°         │   │  Both publish:            │                      │
│  │                      │   │    /vis/head_arm_pose     │                      │
│  │  Eviction:           │   │    (PoseWithCovariance)   │                      │
│  │   max 50 per camera  │   └──────────────────────────┘                      │
│  │   oldest first       │                                                     │
│  └──────────┬───────────┘   ┌──────────────────────────┐                      │
│             │               │ segmentation_node        │ (MODIFIED)           │
│  ┌──────────▼───────────┐   │                           │                      │
│  │ tsdf_grasp_fusion    │   │  Replaces InterObject3D   │                      │
│  │  (NEW)               │   │  with MobileSAM           │                      │
│  │                      │   │                           │                      │
│  │  Trigger: service    │   │  Runs at GRASP TIME only  │                      │
│  │   call from twist    │   │  Prompt: projected hit    │                      │
│  │   propagation        │   │    point on each keyframe │                      │
│  │                      │   │                           │                      │
│  │  Pipeline:           │   │  Dilation: 10-15 px       │                      │
│  │   1. Shift hit point │   │                           │                      │
│  │      inward 1-2 cm   │   │  Output: 2D mask per      │                      │
│  │   2. For each buffered│   │    keyframe image         │                      │
│  │      keyframe:       │   └──────────────────────────┘                      │
│  │      a. Project hit  │                                                     │
│  │         point to 2D  │   ┌──────────────────────────┐                      │
│  │      b. SAM(image,   │   │ openvins_odom_tf_relay   │ (EXISTING, unchanged) │
│  │         px) → mask   │   │  TF: raw odom → frames   │                      │
│  │      c. Dilate mask  │   └──────────────────────────┘                      │
│  │      d. Filter cloud │                                                     │
│  │      e. Integrate    │   ┌──────────────────────────┐                      │
│  │         into TSDF    │   │ twist_propagation        │ (EXISTING, +60 lines) │
│  │   3. Extract mesh    │   │  + kinematic distance    │                      │
│  │   4. DBSCAN cleanup  │   │  + TSDF service trigger  │                      │
│  │   5. Publish to      │   └──────────────────────────┘                      │
│  │      /segmentation/  │                                                     │
│  │      object_cloud    │   ┌──────────────────────────┐                      │
│  └──────────────────────┘   │ pipeline_manager         │ (EXISTING, unchanged) │
│                              │  subscribes to           │                      │
│                              │  /segmentation/          │                      │
│                              │  object_cloud            │                      │
│                              └──────────────────────────┘                      │
└──────────────────────────────────────────────────────────────────────────────┘
```

---

## 4. Key Architectural Decisions

### 4.1 SAM-First, Fuse-Second (Segment Each Keyframe Before TSDF)

**Rationale:**
- SAM runs on native camera images where it works best (no 3D→2D reprojection of fused geometry)
- Dilated masks (10-15 px) create a safety buffer against tracking errors
- Multi-view consensus in TSDF naturally suppresses background from any single view
- The views cross-erase each other's occlusion shadows (head carves what wrist can't see, and vice versa)

**Pipeline per keyframe at grasp time:**
```
3D hit point → project to 2D using that keyframe's K + pose
             → MobileSAM(RGB_image, click=projected_pixel) → binary mask
             → cv2.dilate(mask, 15px kernel)
             → filter organized point cloud: keep points where mask[pixel] == 1
             → integrate filtered cloud into Open3D ScalableTSDFVolume
```

**Why not segment at capture time:**
At capture time, we don't know what object the user will eventually grasp. Using image center or a generic prompt segments whatever happens to be there. Waiting until grasp time gives us the exact hit point as a SAM prompt — zero ambiguity about the target.

### 4.2 MobileSAM Replaces InterObject3D

**Why:**
- InterObject3D/MinkowskiEngine was already crashing (suspected CUDA OOM on 4GB)
- MinkowskiEngine locks us into Python 3.8 (fragile Docker dependency)
- MobileSAM uses ~300 MB VRAM vs MinkowskiEngine's likely 2+ GB
- 2D SAM is architecturally cleaner when we already have RGB images flowing
- The organized point cloud gives us pixel↔3D correspondence for free

**Impact on existing code:**
- `src/segmentation/nodes/inference_server.py` — replaced with MobileSAM Flask server (Python 3.10+)
- `src/segmentation/segmentation_bridge/segmentation_ros2_node.py` — retained for legacy click-driven path, bypassed by TSDF path
- `docker/Dockerfile.segmentation` — rewritten for Python 3.10+, PyTorch 2.x, no MinkowskiEngine
- `docker/docker-compose.segmentation.cuda.yml` — updated segmentation service definition

### 4.3 Segment at Grasp Time, Not Continuously

**Why:**
- Zero steady-state GPU usage — the GPU idles until a grasp triggers
- At grasp time, ~20 keyframes × ~10ms SAM inference = ~200ms total
- This is negligible compared to mechanical prosthesis latency
- The hit point provides the perfect SAM prompt — no ambiguity about what to segment

**What runs continuously vs. on-demand:**

| Component | Continuous? | Rate |
|-----------|------------|------|
| GTSAM Trajectory Binder | Yes | 15 Hz (CPU) |
| Cross-camera features (SIFT/SuperPoint) | Yes | 3-5 Hz |
| Keyframe buffer (cloud + image capture) | Yes | ~15 Hz ingestion, spatial-gated storage |
| MobileSAM | No (grasp time only) | Burst of ~20 frames |
| TSDF fusion | No (grasp time only) | Single invocation |

### 4.4 Hit Point Shifted Inward 1-2 cm

The twist propagation hit point sits on the outer surface of the object. For the ROI sphere to fully encompass the object:

```
P_internal = P_surface + d_normalized * 0.015  (1.5 cm inward along camera ray)
```

This ensures the 15 cm ROI sphere is centered within the object's volume rather than on its front face. The shift is small enough that it won't overshoot small objects but large enough that the sphere captures the full structural mass.

**Edge case — multi-camera shift direction:** The shift direction uses the camera→hit ray from whichever camera detected the hit. When projecting to other keyframes (e.g., head camera keyframes when the hit came from the arm camera), the shifted point may not be centered in the object from that camera's perspective. This is acceptable because:
1. The 15 cm ROI sphere is generous — it captures the object even with 1-2 cm offset.
2. SAM's dilation (15 px) adds further tolerance.
3. If needed, the shift can be computed per-keyframe using that keyframe's camera position.

### 4.5 Cross-Camera Features: SIFT First, SuperPoint Optional

**Default (zero VRAM):** SIFT or AKAZE on CPU at 3-5 Hz, running on keyframe RGB images.
- OpenCV's `cv2.SIFT_create()` is now free (patent expired)
- Handles ~30-40° viewpoint changes — marginal for head-to-wrist but may suffice
- Publishes `PoseWithCovarianceStamped` to `/vis/head_arm_pose`

**Upgrade (if SIFT match quality is poor):** SuperPoint + LightGlue on GPU at 3-5 Hz.
- Handles 60-70° viewpoint changes — robust to head-to-wrist differences
- Uses ~500 MB VRAM — viable after removing InterObject3D
- Same output topic — drop-in replacement for SIFT

**Both integrate identically into GTSAM:** The graph receives `BetweenFactor(head_i, arm_i, T_head_arm, noise)`. It doesn't care whether T_head_arm came from SIFT or SuperPoint.

---

## 5. Critical Integration Points (New in V5)

### 5.1 Pipeline Manager Integration

**Problem:** The TSDF fusion node must publish to `/segmentation/object_cloud` (the topic `pipeline_manager_node.py:243` subscribes to), NOT to a new topic. The pipeline manager's state machine (`IDLE → GRASPING → SEGMENTING → PRESHAPING → ...`) transitions on receiving a new cloud on that topic.

**Solution:** The TSDF fusion node publishes its output to `/segmentation/object_cloud` with the same `PointCloud2` message format. The pipeline manager detects the new cloud by timestamp comparison and proceeds normally.

**Topic mapping:**
```
tsdf_fusion config:
  output_topic: "/segmentation/object_cloud"   # MUST match pipeline_manager subscription
```

### 5.2 Twist Propagation Flow Change

**Current flow (click-driven):**
1. `twist_propagation` detects hit → publishes `PointStamped` to `/segmentation/click_positive`
2. `segmentation_ros2_node` receives click → calls InterObject3D → publishes `/segmentation/object_cloud`
3. `twist_propagation` detects new cloud stamp → transitions `WAITING_FOR_SEGMENTATION → IDLE`
4. `pipeline_manager` detects new cloud → calls preshaping service

**New flow (TSDF service-driven):**
1. `twist_propagation` detects hit → calls TSDF fusion service (blocking or async)
2. TSDF fusion queries keyframe buffer → runs SAM + TSDF → publishes `/segmentation/object_cloud`
3. TSDF fusion service returns success/failure to `twist_propagation`
4. `twist_propagation` transitions `WAITING_FOR_SEGMENTATION → IDLE` on service response
5. `pipeline_manager` detects new cloud → calls preshaping service (unchanged)

**Changes to `twist_propagation_node.py`** (beyond the kinematic check):

```python
# New parameters
self.declare_parameter("use_tsdf_fusion", False)
self.declare_parameter("tsdf_fusion_service", "/tsdf_fusion/trigger")

# In __init__, create service client:
self._tsdf_client = self.create_client(TriggerGraspFusion, tsdf_fusion_service)

# In _run_idle_cycle, after hit detection (replacing click publish block):
if self._use_tsdf_fusion:
    # Call TSDF service instead of publishing clicks
    request = TriggerGraspFusion.Request()
    request.hit_point.x = hit_x
    request.hit_point.y = hit_y
    request.hit_point.z = hit_z
    request.hit_frame = self._cloud_frame
    future = self._tsdf_client.call_async(request)
    # Track future; transition to WAITING when service response arrives
else:
    # Legacy path: publish clicks (existing code)
    ...
```

**New service definition** (`tsdf_fusion/srv/TriggerGraspFusion.srv`):
```
# Request
geometry_msgs/PointStamped hit_point
float64 roi_radius
---
# Response
bool success
string message
sensor_msgs/PointCloud2 object_cloud
```

**Estimated change to twist_propagation_node.py:** ~60 lines (not 30 as in V4).

### 5.3 GTSAM Smoother Choice: FixedLagSmoother vs. Raw iSAM2

**Problem:** The V4 plan specifies "15 second sliding window" with iSAM2. However, raw `gtsam.ISAM2` does not naturally slide — old variables accumulate indefinitely. A 15 Hz rate with a 15s window produces ~450 nodes (head + arm), which grows unbounded.

**Solution:** Use `gtsam.FixedLagSmoother` (or `gtsam.BatchFixedLagSmoother`) instead of raw `iSAM2`. This automatically marginalizes out variables older than the specified lag.

```python
import gtsam

# FixedLagSmoother with 15-second window
smoother = gtsam.FixedLagSmoother(
    lag=15.0,  # seconds
    parameters=gtsam.ISAM2Params(
        relinearizeThreshold=0.001,
        relinearizeSkip=3,
    ),
)

# On each new odometry:
new_factors = gtsam.NonlinearFactorGraph()
new_values = gtsam.Values()
# ... add BetweenFactor from odom delta ...
new_timestamps = {key: stamp_sec}

smoother.update(new_factors, new_values, new_timestamps)
result = smoother.calculateEstimate()
```

**Why not `IncrementalFixedLagSmoother`:** It uses iSAM2 internally but with marginalization. This is exactly what we want — incremental updates with automatic variable pruning.

### 5.4 ArUco Factor Integration Details

The GTSAM tracker receives ArUco observations and converts them to factors:

**PriorFactor (world ArUco markers):**
```python
# From MarkerPoseObservation:
#   msg.pose.pose = T_map_imu (IMU pose in marker_map)
#   msg.pose.covariance = 6x6 diagonal covariance
# The observation is already in marker_map frame.

T_map_imu = pose_to_matrix(msg.pose.pose)
cov_diag = np.array(msg.pose.covariance).reshape(6, 6).diagonal()
noise = gtsam.noiseModel.Diagonal.Sigmas(gtsam.Point6(cov_diag))

# Find the corresponding GTSAM key for the nearest head/arm pose node
key = find_nearest_key(msg.header.stamp, camera_id="head")
factor = gtsam.PriorFactorPose3(key, gtsam.Pose3(T_map_imu), noise)
```

**BetweenFactor (dynamic arm marker):**
```python
# From DynamicMarkerObservation:
#   msg.pose.pose = T_cam_marker (in head camera optical frame)
#   msg.camera_frame = "head_d435i_head_color_optical_frame"
# Need to compute T_map_armimu from T_cam_marker + known extrinsics.

# From DynamicArmPoseObservation:
#   msg.pose.pose = T_map_armimu (already computed by Jetson)
#   This is more direct — use this when available.

T_map_armimu = pose_to_matrix(msg.pose.pose)
cov_diag = np.array(msg.pose.covariance).reshape(6, 6).diagonal()
noise = gtsam.noiseModel.Diagonal.Sigmas(gtsam.Point6(cov_diag))

head_key = find_nearest_key(msg.header.stamp, "head")
arm_key = find_nearest_key(msg.header.stamp, "arm")
T_map_headimu = smoother.calculateEstimate(head_key)
T_head_arm = np.linalg.inv(T_map_headimu) @ T_map_armimu

factor = gtsam.BetweenFactorPose3(head_key, arm_key,
    gtsam.Pose3(T_head_arm), noise)
```

### 5.5 Organized Point Cloud Verification

**Assumption:** PointCloud2 messages from the Jetson preserve `height > 1` (organized structure) after crossing the DDS bridge.

**Verification step (Phase 1, Day 1):** Before building the keyframe buffer, verify:
```bash
# On the laptop, during a live session:
ros2 topic echo /head/d435i_head/depth/color/points --field height --once
# Expected: 480 (not 1)
ros2 topic echo /arm/d435i_arm/depth/color/points --field height --once
# Expected: 480 (not 1)
```

**Fallback if not organized:** If `height == 1` (unorganized), the keyframe buffer must fall back to:
1. Store the unorganized cloud as-is.
2. When filtering by SAM mask, project each 3D point to 2D using `K` and `pose`, then check if the projected pixel falls inside the mask. This is slower (O(N) per keyframe) but correct.

### 5.6 TSDF Integration Detail: RGBD Image Construction

**Problem:** The V4 pseudocode calls `make_rgbd_image(filtered_xyz, filtered_rgb, kf.K)` but Open3D's `ScalableTSDFVolume.integrate()` requires an `o3d.geometry.RGBDImage` and `o3d.camera.PinholeCameraIntrinsic`, not raw numpy arrays.

**Corrected pipeline:**
```python
import open3d as o3d

def integrate_keyframe(volume, kf, mask):
    """
    kf: Keyframe with organized cloud (H, W) shape
    mask: 2D binary mask (H, W)
    """
    H, W = mask.shape

    # Create depth image from organized cloud (re-project using K)
    # For organized clouds, pixel (u,v) maps to cloud[v, u]
    depth_image = np.zeros((H, W), dtype=np.float32)
    rgb_image = kf.image.copy()  # (H, W, 3) uint8

    for v in range(H):
        for u in range(W):
            if mask[v, u]:
                xyz = kf.cloud_xyz[v * W + u]  # or [v, u] if (H,W,3) shaped
                # Project to depth: depth = z in camera frame
                # Transform to camera frame: p_cam = inv(kf.pose) @ [x, y, z, 1]
                p_cam = np.linalg.inv(kf.pose) @ np.array([*xyz, 1.0])
                depth_image[v, u] = p_cam[2]  # z in camera frame

    # Zero out masked pixels in RGB
    rgb_image[~mask] = 0

    # Create Open3D RGBD image
    o3d_rgb = o3d.geometry.Image(rgb_image.astype(np.uint8))
    o3d_depth = o3d.geometry.Image(depth_image.astype(np.float32))
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
        o3d_rgb, o3d_depth,
        depth_scale=1.0, depth_trunc=0.5, convert_rgb_to_intensity=False
    )

    # Create camera intrinsic
    intrinsic = o3d.camera.PinholeCameraIntrinsic(
        W, H, float(kf.K[0, 0]), float(kf.K[1, 1]),
        float(kf.K[0, 2]), float(kf.K[1, 2])
    )

    # Integrate
    volume.integrate(rgbd, intrinsic, np.linalg.inv(kf.pose))
```

**Optimization:** The per-pixel loop can be vectorized:
```python
# Vectorized version
p_hom = np.hstack([kf.cloud_xyz.reshape(H, W, 3), np.ones((H, W, 1))])
p_cam = np.einsum('ij,...j->...i', np.linalg.inv(kf.pose), p_hom)
depth_image = np.where(mask, p_cam[:, :, 2], 0.0).astype(np.float32)
```

### 5.7 SAM Cold Start Latency

**Problem:** MobileSAM's image encoder must load into GPU memory on first inference. On a GTX 3050 mobile, this can take 2-5 seconds.

**Solution:** Pre-load the model at TSDF fusion node startup (or at keyframe buffer startup):
```python
class TSDFFusionNode(Node):
    def __init__(self):
        # ...
        # Pre-load SAM model (even though we won't use it until grasp time)
        self._sam_model = load_mobilesam(self.get_parameter("sam_weights_path").value)
        self.get_logger().info("MobileSAM model pre-loaded and ready")
```

This costs ~300 MB steady-state VRAM but eliminates the cold-start penalty on first grasp.

### 5.8 Cross-Camera Feature Synchronization

**Problem:** The SIFT node needs synchronized head + arm image pairs. At 3-5 Hz processing with 30 Hz input, naive "latest from each" can produce up to 200ms temporal misalignment between cameras — significant for a moving arm.

**Solution:** Use `message_filters.ApproximateTimeSynchronizer`:
```python
from message_filters import ApproximateTimeSynchronizer, Subscriber

head_img_sub = Subscriber(self, Image, head_image_topic)
arm_img_sub = Subscriber(self, Image, arm_image_topic)
head_cloud_sub = Subscriber(self, PointCloud2, head_cloud_topic)
arm_cloud_sub = Subscriber(self, PointCloud2, arm_cloud_topic)

self._sync = ApproximateTimeSynchronizer(
    [head_img_sub, arm_img_sub, head_cloud_sub, arm_cloud_sub],
    queue_size=10,
    slop=0.05,  # 50ms tolerance
)
self._sync.registerCallback(self._on_synced_frames)
```

### 5.9 Segmentation Bridge End State

The current `segmentation_ros2_node.py` (which bridges to InterObject3D) will be **retained** during development but **retired** after the TSDF path is validated. The end state:

- `segmentation_ros2_node.py` — **removed** from `pipeline.launch.py` (or made conditional on a `legacy_segmentation` flag)
- `inference_server.py` (InterObject3D) — **removed** along with its Docker container
- `tsdf_fusion_node` — replaces both the inference server and the segmentation bridge
- The segmentation Docker container is replaced by a lightweight MobileSAM container (or SAM runs in-process)

**Launch file change:**
```python
# Current (pipeline.launch.py:318-327):
nodes.append(Node(
    package="segmentation_bridge",
    executable="segmentation_ros2_node",
    ...))

# After migration:
# Remove segmentation_bridge node
# Add tsdf_fusion node instead
nodes.append(Node(
    package="tsdf_fusion",
    executable="tsdf_fusion_node",
    name="tsdf_fusion",
    parameters=[_node_params(config, "tsdf_fusion")],
    output="screen",
))
```

---

## 6. Implementation Plan

### Phase 1: Foundation (Week 1)

#### 1.0 Pre-requisites (Day 1, Morning)

1. **Copy `sensor_fusion_msgs` to laptop workspace:**
   ```
   cp -r ../worktrees/.../docker_ws/multi_cam_localization/sensor_fusion_msgs/ src/sensor_fusion_msgs/
   colcon build --packages-select sensor_fusion_msgs
   ```

2. **Verify ArUco topics on laptop:**
   ```bash
   # During a live session:
   ros2 topic list | grep marker_pose
   ros2 topic echo /head/marker_pose/observation --once
   ```

3. **Verify organized point clouds:**
   ```bash
   ros2 topic echo /head/d435i_head/depth/color/points --field height --once
   # Expected: 480
   ```

4. **Install GTSAM Python bindings:**
   ```bash
   pip install gtsam  # or build from source for latest
   ```

#### 1.1 Kinematic Distance Check in Twist Propagation (Day 1)

**File:** `src/twist_propagation/twist_propagation/twist_propagation_node.py`

Add in `__init__` (parameter declarations) and `_run_idle_cycle`:

```python
# Parameter
self.declare_parameter("max_head_wrist_distance_m", 1.0)
self._max_head_wrist_dist = self.get_parameter("max_head_wrist_distance_m").value

# Check: look up head pose from TF (marker_map → head_cam0)
# Compute Euclidean distance to wrist pose
# If distance > 1.0m: log warning, optionally suppress twist propagation
```

**Effort:** ~1 day, ~30 lines
**Risk:** None — purely additive safety check

#### 1.2 Keyframe Buffer Node (Day 2-5)

**Package:** `src/keyframe_buffer/`

```
src/keyframe_buffer/
├── keyframe_buffer/
│   ├── __init__.py
│   ├── keyframe_buffer_node.py    # Main node (~350 lines)
│   └── keyframe.py               # Keyframe dataclass (~50 lines)
├── test/
│   └── test_keyframe_buffer.py   # Unit tests (~150 lines)
├── setup.py
└── package.xml
```

**Storage per keyframe:**
```python
@dataclass
class Keyframe:
    timestamp: float
    camera_id: str              # "head" or "arm"
    cloud_xyz: np.ndarray       # (H*W, 3) organized point cloud
    cloud_rgb: np.ndarray       # (H*W, 3) per-point RGB
    image: np.ndarray           # (H, W, 3) RGB image for SAM
    K: np.ndarray               # (3, 3) camera intrinsics
    pose: np.ndarray            # (4, 4) T_marker_map→camera from GTSAM
```

**Subscriptions (INDEPENDENT per camera — NOT synchronized):**

> **CRITICAL DESIGN RULE:** The keyframe buffer MUST subscribe to raw, un-synchronized individual camera topics independently. Do NOT use `ApproximateTimeSynchronizer` or any time-gating mechanism that requires head+arm frames to arrive together. During rapid arm movement, the temporal offset between head and wrist cameras can exceed 50ms. If the buffer is gated by a synchronizer, it will silently drop keyframes during the most critical phase of a reach — exactly when the arm is moving fastest and building up the multi-view coverage that TSDF needs.
>
> The `ApproximateTimeSynchronizer` is correct for the SIFT node (which needs matched pairs for feature matching), but the keyframe buffer must store every frame that passes its per-camera spatial gate, regardless of what the other camera is doing.

- `/head/d435i_head/depth/color/points` (organized PointCloud2) — **independent subscription**
- `/arm/d435i_arm/depth/color/points` — **independent subscription**
- `/head/d435i_head/color/image_raw` (Image, for SAM) — **independent subscription**
- `/arm/d435i_arm/color/image_raw` — **independent subscription**
- `/head/d435i_head/color/camera_info` (CameraInfo, for intrinsics)
- `/arm/d435i_arm/color/camera_info`
- `/gtsam/head_pose` (PoseStamped, optimized by GTSAM)
- `/gtsam/arm_pose`

Each camera stream maintains its own independent spatial gate and ring buffer. A new head keyframe is inserted when the head has moved > 10cm or > 15° since the last head keyframe — regardless of what the arm stream is doing, and vice versa.

**Spatial gate (per camera, independent):**
- Translation: insert new keyframe if `|t_new - t_last| > 0.10 m`
- Rotation: insert new keyframe if angle between quaternions > 15°
- Both relative to last keyframe from **the same camera**

**Eviction:**
- Max 50 keyframes per camera (configurable)
- Eviction policy: oldest first (simple, predictable)

**Service:**
- `GetKeyframesInROI(center, radius) → list of Keyframe` (used by TSDF fusion at grasp time)

**Effort:** ~3 days
**Risk:** Low — storage and filtering logic only

#### 1.3 GTSAM Trajectory Binder (Day 3-7, overlapping with keyframe buffer)

**Package:** `src/gtsam_tracker/`

```
src/gtsam_tracker/
├── gtsam_tracker/
│   ├── __init__.py
│   ├── trajectory_binder_node.py    # Main node (~400 lines)
│   ├── factor_graph.py              # Graph construction + FixedLagSmoother (~250 lines)
│   └── utils.py                     # SE(3) helpers (~80 lines)
├── test/
│   ├── test_factor_graph.py         # Unit tests with mock odom (~200 lines)
│   └── test_se3_helpers.py          # Unit tests (~80 lines)
├── setup.py
└── package.xml
```

**State:** Pose nodes at 15 Hz (decimated from 200 Hz odometry):
- `head_i` — SE(3) pose of head IMU in marker_map
- `arm_i` — SE(3) pose of arm IMU in marker_map

**Four factor types:**

1. **Between factors (odometry):** `gtsam.BetweenFactorPose3(pose_i, pose_{i+1}, delta, noise)`
   - Delta from consecutive OpenVINS odometry messages
   - Noise from odometry covariance (or fixed diagonal: 1cm translation, 0.01 rad rotation)

2. **Prior factors (world ArUco):** `gtsam.PriorFactorPose3(camera_i, T_map_cam, noise)`
   - When either camera sees a world marker (from `MarkerPoseObservation`)
   - T_map_cam computed from marker pose + known camera extrinsics
   - Noise from marker observation quality (already in message)

3. **Between factors (head→wrist visual):** `gtsam.BetweenFactorPose3(head_i, arm_i, T_head_arm, noise)`
   - From SIFT/SuperPoint feature matching (`/vis/head_arm_pose`)
   - Noise from Umeyama residual covariance or fixed conservative estimate

4. **Kinematic range factor:** Soft hinge-loss penalty when `|head_i.t - arm_i.t| > 1.0m`
   - Zero cost below threshold, quadratic above
   - Implemented as custom factor or using two `RangeFactor` components

**FixedLagSmoother parameters:**
- 15 second lag window
- iSAM2 backend with relinearization threshold 0.001, relinearize skip 3
- Wildfire threshold: 0.001

**Output:**
- `/gtsam/head_pose` (PoseStamped, 15 Hz)
- `/gtsam/arm_pose` (PoseStamped, 15 Hz)
- Optionally: `/gtsam/head_odom`, `/gtsam/arm_odom` (Odometry, for diagnostics)

**Effort:** ~4 days (overlapping with keyframe buffer development)
**Risk:** Moderate — GTSAM noise model tuning requires iteration

### Phase 2: Segmentation Pivot (Week 2)

#### 2.1 MobileSAM Inference Server (Day 1-3)

**New Dockerfile:** `docker/Dockerfile.segmentation_v2`

- Base: Python 3.10+ (not 3.8 — no MinkowskiEngine dependency)
- PyTorch 2.x with CUDA
- MobileSAM from `ultralytics` or `segment-anything` package

**New inference server:** `src/segmentation/nodes/inference_server_v2.py`

```python
# Core difference from current server:
# - No MinkowskiEngine
# - Takes RGB image (base64 JPEG) + 2D click pixel (u, v)
# - Returns 2D binary mask (not per-point mask)
# - Uses MobileSAM image encoder (cached after first load, ~300 MB VRAM)

@app.route("/segment_2d", methods=["POST"])
def segment_2d():
    data = request.get_json(force=True)
    image = _decode_image(data["image"])        # JPEG → numpy (H, W, 3)
    click_u = data["click_u"]                   # pixel x coordinate
    click_v = data["click_v"]                   # pixel y coordinate
    dilation_px = data.get("dilation_px", 15)   # mask dilation amount

    # MobileSAM inference with click prompt
    mask = _model.predict(image, points=[[click_u, click_v]], labels=[1])

    # Dilate for tracking error tolerance
    mask = cv2.dilate(mask, np.ones((dilation_px, dilation_px), np.uint8))

    # Return as base64-encoded PNG or run-length encoding
    return jsonify({"mask": _encode_mask(mask), "shape": mask.shape})
```

**Alternative — run SAM in-process:**
If the Flask HTTP bridge adds unacceptable latency for the grasp-time burst of ~20 SAM calls, run MobileSAM directly in the ROS node. This avoids serialization overhead. Start with Flask for development simplicity; profile and inline if needed.

#### 2.2 TSDF Grasp Fusion Node (Day 3-5)

**Package:** `src/tsdf_fusion/`

```
src/tsdf_fusion/
├── tsdf_fusion/
│   ├── __init__.py
│   ├── tsdf_fusion_node.py      # Main node (~350 lines)
│   └── sam_segmenter.py         # SAM bridge (~100 lines)
├── srv/
│   └── TriggerGraspFusion.srv   # Service definition
├── test/
│   └── test_tsdf_fusion.py      # Unit tests with synthetic data (~200 lines)
├── setup.py
└── package.xml
```

**Trigger:** ROS service call from `twist_propagation_node` when a hit is detected and grasp is imminent.

**Pipeline on trigger (corrected for Open3D API):**

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
        voxel_length=0.005,
        sdf_trunc=0.02,
        color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8)

    # 4. For each keyframe:
    for kf in keyframes:
        H, W = kf.image.shape[:2]

        # a. Project 3D hit point to 2D
        uv = project_3d_to_2d(hit_internal, kf.K, kf.pose)

        # b. Run MobileSAM on the keyframe's RGB image
        mask = sam_segment(kf.image, uv, dilation_px=15)

        # c. Create depth image from organized cloud (vectorized)
        p_hom = np.hstack([kf.cloud_xyz.reshape(H, W, 3),
                           np.ones((H, W, 1))])
        pose_inv = np.linalg.inv(kf.pose)
        p_cam = np.einsum('ij,...j->...i', pose_inv, p_hom)
        depth_image = np.where(mask, p_cam[:, :, 2], 0.0).astype(np.float32)

        # d. Create RGBD image
        rgb_image = kf.image.copy()
        rgb_image[~mask] = 0
        o3d_rgb = o3d.geometry.Image(rgb_image.astype(np.uint8))
        o3d_depth = o3d.geometry.Image(depth_image)
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            o3d_rgb, o3d_depth, depth_scale=1.0, depth_trunc=0.5,
            convert_rgb_to_intensity=False)

        # e. Create camera intrinsic
        intrinsic = o3d.camera.PinholeCameraIntrinsic(
            W, H, float(kf.K[0, 0]), float(kf.K[1, 1]),
            float(kf.K[0, 2]), float(kf.K[1, 2]))

        # f. Integrate into TSDF
        volume.integrate(rgbd, intrinsic, pose_inv)

    # 5. Extract fused cloud
    fused_cloud = volume.extract_point_cloud()

    # 6. DBSCAN cleanup (seed at hit point)
    clean_cloud = dbscan_cleanup(fused_cloud, hit_internal, eps=0.02)

    # 7. Publish to /segmentation/object_cloud (pipeline_manager listens here)
    object_cloud_pub.publish(clean_cloud)

    return TriggerGraspFusion.Response(success=True, message="OK")
```

**Why DBSCAN is still useful even with SAM-first:**

After TSDF fusion with SAM-filtered keyframes, the main object is a solid mass. But there may be small disconnected blobs from:
- Background that survived dilation in one view (but was correctly segmented in others)
- Floating points near the ROI sphere boundary

DBSCAN with `eps=2cm` and the hit point as seed grabs only the connected component containing the target. This is a ~1ms operation on a 15cm sphere — nearly free.

#### 2.3 Hit Point Inward Shift

```python
def shift_inward(hit_point, camera_position, offset=0.015):
    """Shift hit point along camera→hit ray into the object."""
    ray = hit_point - camera_position
    ray_norm = ray / np.linalg.norm(ray)
    return hit_point + ray_norm * offset
```

The `camera_position` comes from the GTSAM-optimized pose of whichever camera detected the hit (typically the wrist camera). The offset of 1-2 cm is enough to move past the surface noise layer without overshooting small objects.

### Phase 3: Cross-Camera Features (Week 2-3, parallel with Phase 2)

#### 3.1 SIFT Feature Node (Default, Day 1-3)

**Package:** `src/cross_camera_features/`

```python
from message_filters import ApproximateTimeSynchronizer, Subscriber

class SIFTFeatureNode(Node):
    """
    Extracts SIFT features from head and wrist RGB images,
    matches them, and publishes a relative pose constraint.
    Runs on CPU at 3-5 Hz. Zero VRAM cost.
    """

    def __init__(self):
        self._sift = cv2.SIFT_create(nfeatures=2000)
        self._matcher = cv2.BFMatcher(cv2.NORM_L2, crossCheck=True)

        # Synchronized subscriptions (50ms tolerance)
        head_img_sub = Subscriber(self, Image, head_image_topic)
        arm_img_sub = Subscriber(self, Image, arm_image_topic)
        head_cloud_sub = Subscriber(self, PointCloud2, head_cloud_topic)
        arm_cloud_sub = Subscriber(self, PointCloud2, arm_cloud_topic)
        self._sync = ApproximateTimeSynchronizer(
            [head_img_sub, arm_img_sub, head_cloud_sub, arm_cloud_sub],
            queue_size=10, slop=0.05)
        self._sync.registerCallback(self._on_synced_frames)

        self._pub = self.create_publisher(
            PoseWithCovarianceStamped, "/vis/head_arm_pose", 10)

    def _on_synced_frames(self, head_img_msg, arm_img_msg,
                          head_cloud_msg, arm_cloud_msg):
        # 1. Convert to numpy
        head_img = bridge.imgmsg_to_cv2(head_img_msg, "rgb8")
        arm_img = bridge.imgmsg_to_cv2(arm_img_msg, "rgb8")
        head_cloud = parse_organized_cloud(head_cloud_msg)  # (H, W, 3)
        arm_cloud = parse_organized_cloud(arm_cloud_msg)

        # 2. Extract SIFT keypoints + descriptors
        kp_head, des_head = self._sift.detectAndCompute(head_img, None)
        kp_arm, des_arm = self._sift.detectAndCompute(arm_img, None)

        # 3. Match descriptors
        matches = self._matcher.match(des_head, des_arm)
        matches = sorted(matches, key=lambda m: m.distance)[:100]

        # 4. Filter by depth validity (organized cloud lookup)
        valid_matches = []
        for m in matches:
            u_h, v_h = int(kp_head[m.queryIdx].pt[0]), int(kp_head[m.queryIdx].pt[1])
            u_a, v_a = int(kp_arm[m.trainIdx].pt[0]), int(kp_arm[m.trainIdx].pt[1])
            p3d_h = head_cloud[v_h, u_h]
            p3d_a = arm_cloud[v_a, u_a]
            if valid_depth(p3d_h) and valid_depth(p3d_a):
                valid_matches.append((p3d_h, p3d_a))

        # 5. Solve T_head_arm via Umeyama (3D-3D alignment)
        if len(valid_matches) >= 5:
            T, covariance = umeyama(valid_matches)
            publish_pose(T, covariance)
```

**Umeyama implementation:** Use `scipy.spatial.transform.Rotation` + SVD-based point cloud alignment. Open3D's `registration_icp` is an alternative but adds a heavy dependency for this use case.

```python
def umeyama(correspondences):
    """3D-3D alignment via SVD. Returns (T_4x4, 6x6_covariance)."""
    src = np.array([c[0] for c in correspondences])  # head points
    dst = np.array([c[1] for c in correspondences])  # arm points
    N = len(src)

    src_mean = src.mean(axis=0)
    dst_mean = dst.mean(axis=0)
    src_centered = src - src_mean
    dst_centered = dst - dst_mean

    H = src_centered.T @ dst_centered
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T
    t = dst_mean - R @ src_mean

    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t

    # Residual covariance (conservative)
    residuals = (R @ src.T).T + t - dst
    rms = np.sqrt(np.mean(np.sum(residuals**2, axis=1)))
    cov = np.eye(6) * (rms ** 2)
    cov[:3, :3] *= 10  # translation uncertainty scales more

    return T, cov
```

**Rate:** 3-5 Hz (process every 6th-10th frame pair at 30 Hz input)

#### 3.2 SuperPoint Upgrade Path (Optional, Day 4-5)

Same node structure, but replace SIFT extractor with SuperPoint + LightGlue. Same output topic — GTSAM doesn't know the difference.

**When to upgrade:** If SIFT consistently produces < 5 valid matches per frame pair with < 3 cm RMS error. This indicates the head-to-wrist viewpoint difference exceeds SIFT's ~40° limit.

### Phase 4: Integration and Tuning (Week 3-4)

#### 4.1 Launch File Updates

**File:** `src/prosthesis_launch/launch/pipeline.launch.py`

Add to the existing camera section:

```python
# GTSAM Trajectory Binder
nodes.append(Node(
    package="gtsam_tracker",
    executable="trajectory_binder_node",
    name="gtsam_tracker",
    parameters=[_node_params(config, "gtsam_tracker")],
    output="screen",
))

# Cross-Camera Features (SIFT default)
nodes.append(Node(
    package="cross_camera_features",
    executable="sift_feature_node",
    name="cross_camera_features",
    parameters=[_node_params(config, "cross_camera_features")],
    output="screen",
))

# Keyframe Buffer
nodes.append(Node(
    package="keyframe_buffer",
    executable="keyframe_buffer_node",
    name="keyframe_buffer",
    parameters=[_node_params(config, "keyframe_buffer")],
    output="screen",
))

# TSDF Grasp Fusion (replaces segmentation_bridge when use_tsdf_fusion=true)
if _as_bool(context, "use_tsdf_fusion"):
    nodes.append(Node(
        package="tsdf_fusion",
        executable="tsdf_fusion_node",
        name="tsdf_fusion",
        parameters=[_node_params(config, "tsdf_fusion")],
        output="screen",
    ))
else:
    # Legacy segmentation bridge (InterObject3D)
    nodes.append(Node(
        package="segmentation_bridge",
        executable="segmentation_ros2_node",
        name="segmentation_bridge",
        remappings={("/segmentation/input_cloud", "/fused_pointcloud")},
        parameters=[{"inference_url": inference_url, "roi_radius_m": float(roi_radius)}],
        output="screen",
    ))
```

#### 4.2 Configuration

**File:** `config/prosthesis_config.yaml` additions:

```yaml
gtsam_tracker:
  ros__parameters:
    head_odom_topic: "/ov_msckf/odomimu"
    arm_odom_topic: "/ov_msckf_arm/odomimu"
    aruco_marker_topic: "/head/marker_pose/observation"
    aruco_dynamic_topic: "/head/marker_pose/dynamic_observation"
    aruco_arm_pose_topic: "/arm/marker_pose/dynamic_arm_pose_observation"
    graph_rate_hz: 15.0
    smoother_lag_s: 15.0
    kinematic_range_m: 1.0
    kinematic_range_sigma_m: 0.05
    # Noise model defaults (used when odom covariance is unavailable)
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
    process_rate_hz: 5.0
    sync_slop_s: 0.05
    min_matches: 5
    backend: "sift"  # "sift" or "superpoint"

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

tsdf_fusion:
  ros__parameters:
    keyframe_service: "/keyframe_buffer/get_in_roi"
    sam_inference_url: "http://127.0.0.1:5679"  # new SAM server port
    sam_weights_path: "/weights/mobile_sam.pt"
    roi_radius_m: 0.15
    hit_point_shift_m: 0.015
    mask_dilation_px: 15
    voxel_size_m: 0.005
    sdf_trunc_m: 0.02
    dbscan_eps_m: 0.02
    dbscan_min_points: 10
    output_topic: "/segmentation/object_cloud"  # MUST match pipeline_manager subscription

twist_propagation:
  ros__parameters:
    # ... existing params ...
    max_head_wrist_distance_m: 1.0
    enforce_kinematic_constraint: false  # set true to suppress on violation
    use_tsdf_fusion: false               # set true to use TSDF path
    tsdf_fusion_service: "/tsdf_fusion/trigger"
```

#### 4.3 Migration Path from Current Segmentation

1. **Keep the existing InterObject3D server running** during development
2. **Build MobileSAM server on a different port** (5679 vs 5678)
3. **TSDF fusion node uses the new SAM server**
4. **Existing `segmentation_ros2_node.py` and `pointcloud_fusion_node.py` continue to work** for the current click-driven pipeline
5. **Cut over** when the TSDF fusion path is validated (set `use_tsdf_fusion:=true` in launch)
6. **Remove** InterObject3D container and Python 3.8 dependency
7. **Remove** `segmentation_bridge` node from launch (or make conditional)

#### 4.4 Validation Gates

| Gate | What to check | Tool |
|------|--------------|------|
| **ArUco topics visible** | `/head/marker_pose/observation` echoes on laptop | `ros2 topic echo` |
| **Organized clouds** | `height == 480` on both camera point clouds | `ros2 topic echo ... --field height` |
| **GTSAM correctness** | Pose output matches raw odometry within 2cm when no markers visible | `ros2 topic echo /gtsam/head_pose` vs `/ov_msckf/odomimu` |
| **GTSAM drift suppression** | Arm pose stays within 1.0m of head pose over 60s without markers | Record rosbag, plot head-arm distance over time |
| **GTSAM marginalization** | Memory usage stable over 10+ minutes (no unbounded growth) | Monitor `ros2 topic hz /gtsam/head_pose` for rate stability |
| **SIFT match quality** | ≥ 5 valid matches with < 3cm RMS error per frame pair | Node diagnostics logging |
| **Keyframe buffer latency** | < 5ms to insert a new keyframe | Built-in ROS timers |
| **Keyframe buffer memory** | < 600 MB at max capacity (100 keyframes) | `ros2 topic echo /keyframe_buffer/diagnostics` |
| **SAM latency (burst)** | < 300ms for 20 keyframes | Timestamp before/after batch |
| **SAM cold start** | < 5s from node start to model ready | Startup log timestamp |
| **TSDF quality** | Fused cloud has < 2cm point-to-point RMS vs ground truth | Compare against static marker-anchored reference cloud |
| **End-to-end hit-to-cloud** | < 500ms from hit detection to segmented cloud published | Pipeline-wide timestamp tracking |
| **Pipeline manager transition** | State machine reaches PRESHAPING after TSDF output | `ros2 topic echo /pipeline/state_name` |

---

## 7. What Changes vs. Current Code

### 7.1 New Packages

| Package | Lines (est.) | Purpose |
|---------|-------------|---------|
| `sensor_fusion_msgs` | ~50 (copied from Jetson) | Message definitions for ArUco observations |
| `gtsam_tracker` | ~730 + ~280 tests | GTSAM factor graph + FixedLagSmoother |
| `keyframe_buffer` | ~400 + ~150 tests | Spatial-gated keyframe storage with organized cloud + image |
| `tsdf_fusion` | ~450 + ~200 tests + ~20 srv | On-demand TSDF fusion with SAM-first pipeline |
| `cross_camera_features` | ~400 + ~150 tests | SIFT/SuperPoint feature matching → relative pose |

### 7.2 Modified Existing Files

| File | Change | Lines |
|------|--------|-------|
| `twist_propagation_node.py` | Add kinematic distance check + TSDF service trigger + `use_tsdf_fusion` branching | +60 |
| `pipeline.launch.py` | Add 4 new nodes, conditional segmentation/tsdf | +40 |
| `prosthesis_config.yaml` | Add 5 new config sections | +90 |
| `docker-compose.segmentation.cuda.yml` | Add new SAM segmentation service | +20 |
| `Dockerfile.segmentation` | Rewrite for Python 3.10 + MobileSAM | Rewrite |

### 7.3 Unchanged

- `pointcloud_fusion_node.py` — continues real-time fusion with raw TF
- `openvins_odom_tf_relay.py` — continues publishing TF from raw odometry
- `openvins_realsense_tf_bridge_node.py` — unchanged
- `pipeline_manager_node.py` — unchanged (subscribes to `/segmentation/object_cloud` which TSDF publishes to)
- `twist_propagation_node.py` main propagation logic — unchanged (only +60 lines for integration)
- Jetson-side: **nothing changes** — all topics already published

---

## 8. Implementation Sequence

```
Week 1 (Phase 1):
  Mon AM: Pre-requisites (sensor_fusion_msgs, verify topics, verify organized clouds)
  Mon PM: Kinematic constraint in twist_propagation (30 lines)
  Tue-Thu: Keyframe buffer node + tests
  Wed-Fri: GTSAM Trajectory Binder (overlaps with keyframe buffer)
  Fri:   Integration test: GTSAM + keyframe buffer on recorded rosbag

Week 2 (Phase 2):
  Mon-Wed: MobileSAM inference server (replace InterObject3D)
  Wed-Fri: TSDF grasp fusion node + service definition
  Fri:    Integration test: full SAM + TSDF pipeline on recorded rosbag

Week 3 (Phase 3 + 4 start):
  Mon-Wed: SIFT feature node (with ApproximateTimeSynchronizer)
  Thu:    Twist propagation TSDF service integration (+60 lines)
  Fri:    Launch file integration, end-to-end testing on recorded rosbag

Week 4 (Phase 4 finish):
  Mon-Tue: Noise model tuning for GTSAM (with real data)
  Wed:    Dry-run on hardware (no grasp execution, just data flow)
  Thu:    Hardware deployment, latency measurement, tuning
  Fri:    Validation: full pipeline with real grasp execution
```

---

## 9. Risk Assessment

| Risk | Likelihood | Impact | Mitigation |
|------|-----------|--------|------------|
| SIFT fails head-to-wrist viewpoint (too few matches) | Medium | High | Upgrade to SuperPoint at 3-5 Hz (VRAM allows after InterObject3D removal) |
| MobileSAM latency exceeds budget (>300ms for 20 keyframes) | Low | Medium | Run SAM in-process (avoid HTTP overhead); reduce keyframe count |
| GTSAM optimization takes > 67ms (15 Hz deadline missed) | Low | Medium | Reduce lag to 10s; decimate to 10 Hz |
| ArUco topics not discoverable on laptop via DDS | Low | Medium | Verify first (Phase 1.0). If missing, configure CycloneDDS peers |
| Keyframe buffer memory growth over long sessions | Low | Low | Hard limit 50 per camera (~530 MB max); oldest-first eviction; diagnostics topic |
| Organized point cloud assumption breaks (height==1) | Low | Medium | Validate in Phase 1.0; fall back to per-point projection to 2D |
| `sensor_fusion_msgs` build fails on laptop | Low | Low | Pure CMake message generation — should build on any ROS2 distro |
| GTSAM Python bindings unavailable or incompatible | Low | Medium | Build from source; fallback to custom EKF implementation |
| TSDF integration produces artifacts from mask dilation | Low | Low | DBSCAN cleanup removes disconnected blobs; dilate is conservative |
| Keyframe buffer drops frames during rapid arm movement | **AVOIDED** | **N/A** | **Design rule:** Buffer subscribes to raw per-camera topics independently (no `ApproximateTimeSynchronizer`). See Section 1.2 subscriptions note. Only the SIFT node uses `ApproximateTimeSynchronizer`. |

---

## 10. Keyframe Buffer Memory Budget (Corrected)

Per keyframe at 640x480:
- Organized cloud XYZ: 640 × 480 × 3 × 4 bytes = ~3.7 MB
- Organized cloud RGB: 640 × 480 × 3 × 1 byte = ~0.9 MB
- RGB image: 640 × 480 × 3 × 1 byte = ~0.9 MB
- **Total per keyframe: ~5.5 MB**

At max capacity (50 keyframes × 2 cameras = 100 keyframes):
- **Total: ~550 MB**

This is manageable on 16 GB RAM but should be monitored. Add a ROS diagnostics publisher that reports current memory usage.

---

## 11. Summary of Architecture Evolution

| Version | Core Insight | Why Evolved |
|---------|-------------|-------------|
| V1 | TSDF + keyframe buffer can produce better fused clouds | Pose accuracy wasn't addressed |
| V2 | GTSAM needed for retroactive smoothing; defer visual features | Only ArUco for cross-camera — not continuous enough |
| V3 | RGB images available; SuperPoint feasible on dev GPU | Assumed 12GB VRAM; not deployment-realistic |
| V4 | 4GB constraint is real; SAM-first/fuse-second is correct; segment at grasp time not continuously | Matches hardware reality and architectural clarity |
| **V5** | **Integration gaps with pipeline_manager, twist_propagation, ArUco topics, and Open3D API resolved; FixedLagSmoother replaces raw iSAM2; organized cloud verification added; synchronization strategy specified** | **Comprehensive review against actual codebase (both laptop and Jetson worktrees)** |
