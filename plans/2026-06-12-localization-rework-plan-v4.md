# Localization Rework — V4: 4GB-Safe Architecture with SAM-First, Fuse-Second

**Date:** 2026-06-12
**Status:** Consolidated plan incorporating all architectural decisions from the review process.

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
| RAM | 16+ GB | Keyframe buffer ~50-300 MB, negligible |
| Network | USB gadget Ethernet (10.42.0.x) | RGB images at 640×480 already flow; JPEG compression optional |

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

## 2. Architecture Overview

```
┌──────────────────────────────────────────────────────────────────────────────┐
│                              JETSON (unchanged)                               │
│                                                                              │
│  realsense2 → images + points + IMU + camera_info                            │
│  OpenVINS × 2 → odomimu (head + arm)                                        │
│  aruco_marker_pose_node → MarkerPoseObservation, DynamicMarkerObservation   │
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
│  │   /ov_msckf/odomimu   │                                                   │
│  │   /ov_msckf_arm/odom  │                                                   │
│  │   /aruco/marker_obs   │                                                   │
│  │   /aruco/dynamic_obs  │                                                   │
│  │   /vis/head_arm_pose  │ ← optional (SIFT or SuperPoint)                   │
│  │                       │                                                   │
│  │  iSAM2 factors:       │                                                   │
│  │   BetweenFactor(odom) │                                                   │
│  │   PriorFactor(ArUco)  │                                                   │
│  │   BetweenFactor(dyn.) │                                                   │
│  │   BetweenFactor(vis.) │ ← SIFT or SuperPoint                              │
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
│  │   RGB image          │   │                           │                      │
│  │   camera intrinsics  │   │  Mode B (upgrade):        │                      │
│  │   GTSAM-optimized    │   │    SuperPoint + LightGlue │                      │
│  │     pose             │   │    GPU, 3-5 Hz            │                      │
│  │                      │   │                           │                      │
│  │  Spatial gate:       │   │  Both publish:            │                      │
│  │   10cm / 15°         │   │    /vis/head_arm_pose     │                      │
│  │                      │   │    (PoseWithCovariance)   │                      │
│  │  Eviction:           │   └──────────────────────────┘                      │
│  │   max 50 per camera  │                                                     │
│  │   oldest first       │   ┌──────────────────────────┐                      │
│  └──────────┬───────────┘   │ segmentation_node        │ (MODIFIED)           │
│             │               │                           │                      │
│  ┌──────────▼───────────┐   │  Replaces InterObject3D   │                      │
│  │ tsdf_grasp_fusion    │   │  with MobileSAM           │                      │
│  │  (NEW)               │   │                           │                      │
│  │                      │   │  Runs at GRASP TIME only  │                      │
│  │  Trigger: service    │   │  Prompt: projected hit    │                      │
│  │   call from twist    │   │    point on each keyframe │                      │
│  │   propagation        │   │                           │                      │
│  │                      │   │  Dilation: 10-15 px       │                      │
│  │  Pipeline:           │   │                           │                      │
│  │   1. Shift hit point │   │  Output: 2D mask per      │                      │
│  │      inward 1-2 cm   │   │    keyframe image         │                      │
│  │   2. For each buffered│   └──────────────────────────┘                      │
│  │      keyframe:       │                                                     │
│  │      a. Project hit  │   ┌──────────────────────────┐                      │
│  │         point to 2D  │   │ openvins_odom_tf_relay   │ (EXISTING, unchanged) │
│  │      b. SAM(image,   │   │  TF: raw odom → frames   │                      │
│  │         px) → mask   │   └──────────────────────────┘                      │
│  │      c. Dilate mask  │                                                     │
│  │      d. Filter cloud │   ┌──────────────────────────┐                      │
│  │      e. Integrate    │   │ twist_propagation        │ (EXISTING, +30 lines) │
│  │         into TSDF    │   │  + kinematic distance    │                      │
│  │   3. Extract mesh    │   │    check                  │                      │
│  │   4. DBSCAN cleanup  │   └──────────────────────────┘                      │
│  │   5. Publish object  │                                                     │
│  │      cloud           │   ┌──────────────────────────┐                      │
│  └──────────────────────┘   │ pointcloud_fusion        │ (EXISTING, unchanged) │
│                              │  real-time merge+filter  │                      │
│                              └──────────────────────────┘                      │
└──────────────────────────────────────────────────────────────────────────────┘
```

---

## 3. Key Architectural Decisions

### 3.1 SAM-First, Fuse-Second (Segment Each Keyframe Before TSDF)

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

### 3.2 MobileSAM Replaces InterObject3D

**Why:**
- InterObject3D/MinkowskiEngine was already crashing (suspected CUDA OOM on 4GB)
- MinkowskiEngine locks us into Python 3.8 (fragile Docker dependency)
- MobileSAM uses ~300 MB VRAM vs MinkowskiEngine's likely 2+ GB
- 2D SAM is architecturally cleaner when we already have RGB images flowing
- The organized point cloud gives us pixel↔3D correspondence for free

**Impact on existing code:**
- `src/segmentation/nodes/inference_server.py` — replaced with MobileSAM Flask server (Python 3.10+)
- `src/segmentation/segmentation_bridge/segmentation_ros2_node.py` — modified to accept hit-point-based prompts (or replaced by new SAM bridge)
- `docker/Dockerfile.segmentation` — rewritten for Python 3.10+, PyTorch 2.x, no MinkowskiEngine
- `docker/docker-compose.yml` — updated segmentation service definition

### 3.3 Segment at Grasp Time, Not Continuously

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

### 3.4 Hit Point Shifted Inward 1-2 cm

The twist propagation hit point sits on the outer surface of the object. For the ROI sphere to fully encompass the object:

```
P_internal = P_surface + d_normalized * 0.015  (1.5 cm inward along camera ray)
```

This ensures the 15 cm ROI sphere is centered within the object's volume rather than on its front face. The shift is small enough that it won't overshoot small objects but large enough that the sphere captures the full structural mass.

### 3.5 Cross-Camera Features: SIFT First, SuperPoint Optional

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

## 4. Implementation Plan

### Phase 1: Foundation (Week 1)

#### 1.1 Kinematic Distance Check in Twist Propagation (Day 1)

**File:** `src/twist_propagation/twist_propagation/twist_propagation_node.py`

Add in `_run_idle_cycle()` or equivalent main loop:

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
│   ├── keyframe_buffer_node.py    # Main node (~300 lines)
│   └── keyframe.py               # Keyframe dataclass (~50 lines)
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

**Subscriptions:**
- `/head/d435i_head/depth/color/points` (organized PointCloud2)
- `/arm/d435i_arm/depth/color/points`
- `/head/d435i_head/color/image_raw` (Image, for SAM)
- `/arm/d435i_arm/color/image_raw`
- `/head/d435i_head/color/camera_info` (CameraInfo, for intrinsics)
- `/arm/d435i_arm/color/camera_info`
- `/gtsam/head_pose` (PoseStamped, optimized by GTSAM)
- `/gtsam/arm_pose`

**Spatial gate (per camera, independent):**
- Translation: insert new keyframe if `|t_new - t_last| > 0.10 m`
- Rotation: insert new keyframe if angle between quaternions > 15°
- Both relative to last keyframe from same camera

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
│   ├── trajectory_binder_node.py    # Main node (~350 lines)
│   ├── factor_graph.py              # Graph construction + iSAM2 (~200 lines)
│   └── utils.py                     # SE(3) helpers (~80 lines)
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

**iSAM2 parameters:**
- 15 second sliding window (~225 pose nodes)
- Relinearization every 3 timesteps
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
│   ├── tsdf_fusion_node.py      # Main node (~300 lines)
│   └── sam_segmenter.py         # SAM bridge (~100 lines)
├── setup.py
└── package.xml
```

**Trigger:** ROS service call from `twist_propagation_node` when a hit is detected and grasp is imminent.

**Pipeline on trigger:**

```python
def on_grasp_trigger(hit_point_3d, roi_radius=0.15):
    """
    hit_point_3d: (3,) array in marker_map frame
    roi_radius:   sphere radius in meters (default 0.15)
    """
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
        # a. Project 3D hit point to 2D
        uv = project_3d_to_2d(hit_internal, kf.K, kf.pose)
        
        # b. Run MobileSAM on the keyframe's RGB image
        mask = sam_segment(kf.image, uv, dilation_px=15)
        
        # c. Filter organized point cloud by mask
        filtered_xyz, filtered_rgb = filter_by_mask(kf.cloud_xyz, kf.cloud_rgb, mask)
        
        # d. Integrate into TSDF
        rgbd = make_rgbd_image(filtered_xyz, filtered_rgb, kf.K)
        volume.integrate(rgbd, kf.K.inv, kf.pose)
    
    # 5. Extract fused cloud
    fused_cloud = volume.extract_point_cloud()
    
    # 6. DBSCAN cleanup (seed at hit point)
    clean_cloud = dbscan_cleanup(fused_cloud, hit_internal, eps=0.02)
    
    # 7. Publish
    object_cloud_pub.publish(clean_cloud)
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
class SIFTFeatureNode(Node):
    """
    Extracts SIFT features from head and wrist RGB images,
    matches them, and publishes a relative pose constraint.
    Runs on CPU at 3-5 Hz. Zero VRAM cost.
    """
    
    def __init__(self):
        self._sift = cv2.SIFT_create(nfeatures=2000)
        self._matcher = cv2.BFMatcher(cv2.NORM_L2, crossCheck=True)
        # ... subscriptions, publisher to /vis/head_arm_pose
    
    def _on_synced_frames(self, head_img, arm_img, head_cloud, arm_cloud,
                          head_K, arm_K):
        # 1. Extract SIFT keypoints + descriptors
        kp_head, des_head = self._sift.detectAndCompute(head_img, None)
        kp_arm, des_arm = self._sift.detectAndCompute(arm_img, None)
        
        # 2. Match descriptors
        matches = self._matcher.match(des_head, des_arm)
        matches = sorted(matches, key=lambda m: m.distance)[:100]
        
        # 3. Filter by depth validity (both points must have valid depth)
        valid_matches = []
        for m in matches:
            u_h, v_h = int(kp_head[m.queryIdx].pt[0]), int(kp_head[m.queryIdx].pt[1])
            u_a, v_a = int(kp_arm[m.trainIdx].pt[0]), int(kp_arm[m.trainIdx].pt[1])
            p3d_h = head_cloud[u_h, v_h]  # organized cloud lookup
            p3d_a = arm_cloud[u_a, v_a]
            if valid_depth(p3d_h) and valid_depth(p3d_a):
                valid_matches.append((p3d_h, p3d_a))
        
        # 4. Solve T_head_arm via Umeyama (3D-3D alignment)
        if len(valid_matches) >= 5:
            T, covariance = umeyama(valid_matches)
            publish_pose(T, covariance)
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

# TSDF Grasp Fusion
nodes.append(Node(
    package="tsdf_fusion",
    executable="tsdf_fusion_node",
    name="tsdf_fusion",
    parameters=[_node_params(config, "tsdf_fusion")],
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
    aruco_dynamic_topic: "/head/arm_pose/observation"
    graph_rate_hz: 15.0
    window_duration_s: 15.0
    kinematic_range_m: 1.0
    kinematic_range_sigma_m: 0.05

cross_camera_features:
  ros__parameters:
    head_image_topic: "/head/d435i_head/color/image_raw"
    arm_image_topic: "/arm/d435i_arm/color/image_raw"
    head_cloud_topic: "/head/d435i_head/depth/color/points"
    arm_cloud_topic: "/arm/d435i_arm/depth/color/points"
    head_info_topic: "/head/d435i_head/color/camera_info"
    arm_info_topic: "/arm/d435i_arm/color/camera_info"
    process_rate_hz: 5.0
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
    roi_radius_m: 0.15
    hit_point_shift_m: 0.015
    mask_dilation_px: 15
    voxel_size_m: 0.005
    sdf_trunc_m: 0.02
    dbscan_eps_m: 0.02
    dbscan_min_points: 10
    output_topic: "/segmented_object_cloud"

twist_propagation:
  ros__parameters:
    # ... existing params ...
    max_head_wrist_distance_m: 1.0
    enforce_kinematic_constraint: false  # set true to suppress on violation
```

#### 4.3 Migration Path from Current Segmentation

1. **Keep the existing InterObject3D server running** during development
2. **Build MobileSAM server on a different port** (5679 vs 5678)
3. **TSDF fusion node uses the new SAM server**
4. **Existing `segmentation_ros2_node.py` and `pointcloud_fusion_node.py` continue to work** for the current click-driven pipeline
5. **Cut over** when the TSDF fusion path is validated
6. **Remove** InterObject3D container and Python 3.8 dependency

#### 4.4 Validation Gates

| Gate | What to check | Tool |
|------|--------------|------|
| **GTSAM correctness** | Pose output matches raw odometry within 2cm when no markers visible | `ros2 topic echo /gtsam/head_pose` vs `/ov_msckf/odomimu` |
| **GTSAM drift suppression** | Arm pose stays within 1.0m of head pose over 60s without markers | Record rosbag, plot head-arm distance over time |
| **SIFT match quality** | ≥ 5 valid matches with < 3cm RMS error per frame pair | Node diagnostics logging |
| **Keyframe buffer latency** | < 5ms to insert a new keyframe | Built-in ROS timers |
| **SAM latency (burst)** | < 300ms for 20 keyframes | Timestamp before/after batch |
| **TSDF quality** | Fused cloud has < 2cm point-to-point RMS vs ground truth | Compare against static marker-anchored reference cloud |
| **End-to-end hit-to-cloud** | < 500ms from hit detection to segmented cloud published | Pipeline-wide timestamp tracking |

---

## 5. What Changes vs. Current Code

### 5.1 New Packages

| Package | Lines (est.) | Purpose |
|---------|-------------|---------|
| `gtsam_tracker` | ~630 | GTSAM factor graph + iSAM2 sliding window |
| `keyframe_buffer` | ~350 | Spatial-gated keyframe storage with organized cloud + image |
| `tsdf_fusion` | ~400 + ~100 | On-demand TSDF fusion with SAM-first pipeline |
| `cross_camera_features` | ~350 | SIFT/SuperPoint feature matching → relative pose |

### 5.2 Modified Existing Files

| File | Change | Lines |
|------|--------|-------|
| `twist_propagation_node.py` | Add kinematic distance check | +30 |
| `pipeline.launch.py` | Add 4 new nodes to launch | +30 |
| `prosthesis_config.yaml` | Add 5 new config sections | +80 |
| `docker-compose.yml` | Add new SAM segmentation service | +20 |
| `Dockerfile.segmentation` | Rewrite for Python 3.10 + MobileSAM | Rewrite |

### 5.3 Unchanged

- `pointcloud_fusion_node.py` — continues real-time fusion with raw TF
- `openvins_odom_tf_relay.py` — continues publishing TF from raw odometry
- `openvins_realsense_tf_bridge_node.py` — unchanged
- `hand_pose_publisher.py` — unchanged
- `twist_propagation_node.py` main logic — unchanged (only +30 lines)
- Jetson-side: **nothing changes** — all topics already published

---

## 6. Implementation Sequence

```
Week 1 (Phase 1):
  Mon:   Kinematic constraint in twist_propagation (30 lines)
  Tue-Thu: Keyframe buffer node + tests
  Wed-Fri: GTSAM Trajectory Binder (overlaps with keyframe buffer)
  Fri:   Integration test: GTSAM + keyframe buffer on recorded rosbag

Week 2 (Phase 2):
  Mon-Wed: MobileSAM inference server (replace InterObject3D)
  Wed-Fri: TSDF grasp fusion node
  Fri:    Integration test: full SAM + TSDF pipeline on recorded rosbag

Week 3 (Phase 3 + 4 start):
  Mon-Wed: SIFT feature node
  Thu-Fri: Launch file integration, end-to-end testing on recorded rosbag
  Fri:    Dry-run on hardware (no grasp execution, just data flow)

Week 4 (Phase 4 finish):
  Mon-Tue: Noise model tuning for GTSAM (with real data)
  Wed-Thu: Hardware deployment, latency measurement, tuning
  Fri:    Validation: full pipeline with real grasp execution
```

---

## 7. Risk Assessment

| Risk | Likelihood | Impact | Mitigation |
|------|-----------|--------|------------|
| SIFT fails head-to-wrist viewpoint (too few matches) | Medium | High | Upgrade to SuperPoint at 3-5 Hz (VRAM allows after InterObject3D removal) |
| MobileSAM latency exceeds budget (>300ms for 20 keyframes) | Low | Medium | Run SAM in-process (avoid HTTP overhead); reduce keyframe count |
| GTSAM optimization takes > 67ms (15 Hz deadline missed) | Low | Medium | Reduce window to 10s; decimate to 10 Hz |
| ArUco topics not discoverable on laptop via DDS | Low | Medium | Verify first. If missing, add bridge or configure CycloneDDS |
| Keyframe buffer memory growth over long sessions | Low | Low | Hard limit 50 per camera; oldest-first eviction; ROS diagnostics monitoring |
| Organized point cloud assumption breaks (cloud not truly organized) | Low | Medium | Validate pixel↔point correspondence on first frames; fall back to KDTree lookup |

---

## 8. Summary of Architecture Evolution

| Version | Core Insight | Why Evolved |
|---------|-------------|-------------|
| V1 | TSDF + keyframe buffer can produce better fused clouds | Pose accuracy wasn't addressed |
| V2 | GTSAM needed for retroactive smoothing; defer visual features | Only ArUco for cross-camera — not continuous enough |
| V3 | RGB images available; SuperPoint feasible on dev GPU | Assumed 12GB VRAM; not deployment-realistic |
| **V4** | **4GB constraint is real; SAM-first/fuse-second is correct; segment at grasp time not continuously** | **Matches hardware reality and architectural clarity** |
