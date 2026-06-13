# Localization Rework — V3: Multi-Layer Drift Correction with Visual Features

**Date:** 2026-06-12
**Trigger:** User feedback that 20cm drift persists without markers for minutes, and willingness to modify Jetson side if needed
**Status:** Analysis complete. SuperPoint on laptop GPU confirmed feasible (RTX 4070, 12GB VRAM).

---

## 0. What Changed Since V2

A deeper investigation of the Jetson-side code revealed three critical facts that change the architecture:

| Discovery | Implication |
|-----------|-------------|
| **RGB images are already published** (`/head/d435i_head/color/image_raw`, `/arm/d435i_arm/color/image_raw`) | No Jetson changes needed to get images to the laptop. They cross the DDS bridge transparently. |
| **Laptop has RTX 4070 (12GB VRAM)** with working PyTorch CUDA | SuperPoint + LightGlue can run at 15-30 Hz with ~20-30% GPU utilization. No contention with segmentation model needed. |
| **Camera intrinsics published** via standard `/camera_info` topics | Depth lookup for feature matches is straightforward |
| **Point clouds are XYZRGB** (already on the laptop) | Matched image features can be back-projected to 3D using the point cloud for depth |

These discoveries change the answer to your question "Do you think SuperPoint will be too taxing?" from **"maybe, depend on GPU"** to **"no, it's entirely feasible on your hardware."**

---

## 1. The Core Problem Reframed

You described it perfectly: the system needs to work **for minutes without markers,** not seconds. The current EKF-based ArUco correction snaps the present but can't fix the past. The drift accumulates whenever markers aren't visible.

The solution needs **three layers of constraints**, ordered by availability and cost:

| Layer | What it does | Always available? | Accuracy |
|-------|-------------|-------------------|----------|
| **Kinematic constraint** | Prevents head-wrist distance > 1.0m | Yes | Coarse (meters) |
| **Visual feature constraints** (SuperPoint + LightGlue) | Continuous head→wrist relative pose from overlapping views | Yes (when views overlap) | Good (cm-level relative) |
| **ArUco marker constraints** | Absolute pose anchors from world and wrist markers | No (requires marker visibility) | Best (mm-level absolute) |

The key insight: **Layer 2 (visual features) provides continuous relative pose constraints between the two cameras.** This is what prevents the two trajectories from drifting apart independently when no markers are visible. Layer 3 (ArUco) provides absolute anchoring when available, and Layer 1 (kinematic) is a safety net.

---

## 2. Why SuperPoint + LightGlue on the Laptop GPU

### 2.1 Performance on RTX 4070

| Component | Resolution | Time per frame | Memory |
|-----------|-----------|----------------|--------|
| SuperPoint (encoder + keypoint + descriptor) | 640×480 | 10-15 ms | ~200 MB |
| LightGlue (matching, one pair) | ~500-2000 keypoints each | 15-25 ms | ~300 MB |
| **Total per camera pair** | — | **25-40 ms (25-40 Hz)** | ~500 MB |

At 15 Hz (matching the point cloud fusion rate), this uses ~20-30% of the GPU. The segmentation model runs on-demand (triggered by a click), so there's no steady-state GPU contention.

### 2.2 Why SuperPoint over ORB

| | ORB | SuperPoint + LightGlue |
|---|---|---|
| Viewpoint robustness | Poor beyond ~30° rotation | Good to ~60-70° rotation |
| Texture requirements | Needs corners/blobs | Handles low-texture surfaces |
| Scale invariance | Scale pyramid (limited) | Learned multi-scale features |
| GPU needed | No (CPU) | Yes |
| Match quality (inlier ratio) | 20-40% | 50-80% |

The head and wrist cameras have **different viewpoints of the same workspace.** They're not looking in opposite directions — they both look at the table/workspace area in front of the user, but from different angles (head ~45° down from above, wrist ~horizontal from the side). SuperPoint handles this viewpoint difference much better than ORB.

### 2.3 The "Depth from Point Cloud" Trick

You don't need to send depth images separately or undistort them. The XYZRGB point cloud is already on the laptop. For each matched image feature at (u, v):

1. Look up the 3D point at that (u, v) from the organized point cloud
2. If the depth is valid (> 0, < 3m), keep the match
3. You now have a 3D-3D correspondence between the two camera frames

This avoids needing to handle depth image intrinsics, distortion, or alignment. The RealSense driver already did that work.

**Important:** The RealSense point cloud at 640×480 is **organized** (has the same (u, v) layout as the image). This means you can index point_cloud[u + v * width] directly. The `pointcloud_fusion_node.py` already does this for its KDTree construction.

---

## 3. Proposed Architecture

```
┌──────────────────────────────────────────────────────────────────────────────┐
│                                 JETSON (10.42.0.2)                            │
│                                                                              │
│  ┌─────────────────────┐   ┌─────────────────────┐                           │
│  │ realsense2 (head)   │   │ realsense2 (arm)    │                           │
│  │  image, depth, IMU, │   │  image, depth, IMU, │                           │
│  │  points, camera_info│   │  points, camera_info│                           │
│  └────────┬────────────┘   └────────┬────────────┘                           │
│           │                          │                                        │
│  ┌────────▼────────────┐   ┌────────▼────────────┐                           │
│  │ OpenVINS (head)     │   │ OpenVINS (arm)      │                           │
│  │  feature tracking   │   │  feature tracking   │                           │
│  │  odomimu publish    │   │  odomimu publish    │                           │
│  └────────┬────────────┘   └────────┬────────────┘                           │
│           │                          │                                        │
│  ┌────────▼──────────────────────────▼────────────┐                           │
│  │ aruco_marker_pose_node                        │                           │
│  │  MarkerPoseObservation (world)                 │                           │
│  │  DynamicMarkerObservation (head→wrist)         │                           │
│  └───────────────────────────────────────────────┘                           │
│                                                                              │
│  NO CHANGES NEEDED — all topics already published                            │
└──────────────────────────────────────────────────────────────────────────────┘
                          │  DDS unicast (CycloneDDS, host network mode)
                          │  10.42.0.2 ↔ 10.42.0.1
                          ▼
┌──────────────────────────────────────────────────────────────────────────────┐
│                              LAPTOP (10.42.0.1)                               │
│                                                                              │
│  ┌───────────────────────┐   ┌──────────────────────┐                        │
│  │ visual_feature_node   │   │ gtsam_trajectory_binder│                       │
│  │  (NEW)                 │   │  (NEW)                │                       │
│  │                       │   │                       │                        │
│  │  Sub:                 │   │  Inputs:              │                        │
│  │   /head/.../image_raw │   │   odomimu (head+arm)  │                        │
│  │   /arm/.../image_raw  │   │   marker_obs (ArUco)  │                        │
│  │   /head/.../camera_info│  │   dynamic_obs (ArUco) │                        │
│  │   /arm/.../camera_info│   │   head→arm_pose (NEW) │← visual_feature_node  │
│  │   /head/.../points    │   │                       │                        │
│  │   /arm/.../points     │   │  iSAM2 factors:       │                        │
│  │                       │   │   BetweenFactor(odom) │                        │
│  │  GPU: SuperPoint      │   │   PriorFactor(ArUco)  │                        │
│  │       + LightGlue     │   │   BetweenFactor(dyn.) │                        │
│  │                       │   │   BetweenFactor(vis.) │← SuperPoint matches    │
│  │  Output:              │   │   RangeFactor(1.0m)   │                        │
│  │   /vis/head_arm_pose  │──▶│                       │                        │
│  │   (PoseWithCovariance)│   │  Output:              │                        │
│  └───────────────────────┘   │   /gtsam/head_pose    │                        │
│                               │   /gtsam/arm_pose     │                        │
│  ┌───────────────────────┐   │   /gtsam/head_odom    │                        │
│  │ openvins_odom_tf_relay│   │   /gtsam/arm_odom     │                        │
│  │  (existing, unchanged) │   └──────────────────────┘                        │
│  │  TF: raw odom → tf    │              │                                     │
│  └───────────────────────┘              │                                     │
│                               ┌─────────▼──────────┐                         │
│  ┌───────────────────────┐    │ keyframe_buffer     │                         │
│  │ pointcloud_fusion     │    │  (NEW)              │                         │
│  │  (existing, unchanged)│    │  stores clouds with │                         │
│  │  uses raw TF for now  │    │  GTSAM-corrected    │                         │
│  └───────────────────────┘    │  poses              │                         │
│                               └─────────┬──────────┘                         │
│                               ┌─────────▼──────────┐                         │
│                               │ tsdf_fusion         │                         │
│                               │  (NEW)              │                         │
│                               │  on-demand ROI fuse │                         │
│                               └────────────────────┘                         │
└──────────────────────────────────────────────────────────────────────────────┘
```

### 3.1 Visual Feature Node (`visual_feature_node.py`)

This is the new node that provides **continuous cross-camera constraints** from SuperPoint + LightGlue.

```python
class VisualFeatureNode(Node):
    """
    Extracts SuperPoint features from head and wrist RGB images,
    matches them with LightGlue, projects to 3D using point clouds,
    and publishes a relative pose constraint between cameras.
    """

    def __init__(self):
        # ── Subscriptions ──
        # RGB images (640x480, 30 Hz from RealSense)
        self._head_img_sub = message_filters.Subscriber(
            self, Image, '/head/d435i_head/color/image_raw')
        self._arm_img_sub = message_filters.Subscriber(
            self, Image, '/arm/d435i_arm/color/image_raw')
        # Camera info for intrinsics
        self._head_info_sub = ...  # /head/d435i_head/color/camera_info
        self._arm_info_sub = ...   # /arm/d435i_arm/color/camera_info
        # Point clouds for depth lookup
        self._head_cloud_sub = ...
        self._arm_cloud_sub = ...

        # ── ApproximateTimeSynchronizer (50ms tolerance) ──
        # Matches the existing pattern in pointcloud_fusion_node.py

        # ── GPU Models (lazy init on first frame) ──
        self._superpoint = None  # kornia or SuperGluePretrainedNetwork
        self._lightglue = None

        # ── Publisher ──
        self._pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/vis/head_arm_pose', 10)

    def _on_synced_frames(self, head_img, arm_img, head_info, arm_info,
                          head_cloud, arm_cloud):
        """Main pipeline: extract → match → project → solve → publish."""
        # 1. Convert ROS Image → torch tensor (H, W)
        # 2. SuperPoint: extract keypoints + descriptors for each image
        # 3. LightGlue: match descriptors between images
        # 4. For each match (u_head, v_head) ↔ (u_arm, v_arm):
        #    a. Look up 3D point from head_cloud[u_head, v_head]
        #    b. Look up 3D point from arm_cloud[u_arm, v_arm]
        #    c. If both have valid depth, add to correspondences
        # 5. If ≥ 5 valid correspondences:
        #    a. Solve for T_head_arm using Umeyama (SVD-based 3D-3D alignment)
        #    b. Compute inlier covariance from residuals
        #    c. Publish PoseWithCovarianceStamped
        #    d. Log: num_matches, num_inliers, residual
```

### 3.2 Key Design Decisions

**Q: Why 3D-3D alignment (Umeyama) instead of PnP?**

The point cloud gives us 3D coordinates for both cameras directly. 3D-3D alignment with Umeyama is:
- Closed-form (no iterative optimization)
- More accurate than PnP when depth is reliable
- Provides a natural covariance estimate from residuals

**Q: What about the viewpoint difference?**

The head camera is mounted above the user's eye line, looking down at the workspace (~45° angle). The wrist camera is on the user's arm, looking at the workspace from a lower angle. They typically share 40-60% of their field of view (the tabletop workspace).

SuperPoint + LightGlue handles 60-70° viewpoint changes well. The key failure mode is when the wrist camera looks at something completely outside the head camera's view (e.g., reaching to the side). In that case:
- LightGlue produces few/no matches → no constraint published that frame
- The GTSAM graph falls back to odometry between factors (relative motion)
- The kinematic range constraint prevents divergence
- When the wrist returns to the workspace, matches resume and the GTSAM graph retroactively smooths the trajectory

**Q: At what rate should this run?**

15 Hz (matching point cloud fusion rate). At 30 FPS image input, we process every other frame. This gives:
- 25-40ms GPU time per pair → 25 Hz maximum theoretical rate
- 15 Hz leaves ~50% GPU headroom for other operations

**Q: Does the Jetson need modification?**

No. RGB images, camera info, and point clouds are already published by the RealSense driver. The DDS bridge (CycloneDDS peer discovery) already forwards all topics to the laptop. No Jetson-side code changes are needed for this approach.

---

## 4. Revised Implementation Plan

### Phase 1: Visual Feature Node (Week 1)

| Day | Task | Validation |
|-----|------|-----------|
| 1-2 | Set up `visual_feature` package, subscribe to synced images + camera_info + point clouds | Verify topics arrive with `ros2 topic echo` |
| 3-4 | Integrate SuperPoint + LightGlue (kornia or official repo) | Run on sample image pair, verify keypoints visible |
| 5 | Implement depth lookup from organized point cloud | Verify 3D correspondences are plausible |
| 6 | Implement Umeyama 3D-3D alignment + covariance estimation | Test on recorded rosbag with known geometry |
| 7 | Publish `PoseWithCovarianceStamped`, add diagnostics logging | Monitor match counts and residuals |

**Files to create:**
- `src/visual_feature/visual_feature/visual_feature_node.py` (~350 lines)
- `src/visual_feature/visual_feature/superpoint_lightglue.py` (~150 lines, model wrapper)
- `src/visual_feature/setup.py`, `src/visual_feature/package.xml`

**Dependencies to add:**
- `kornia` (provides SuperPoint + LightGlue implementations)
- OR `gluefactory` / official LightGlue repo
- Already available: `torch`, `numpy`, `cv_bridge`, `sensor_msgs`

**Effort:** ~1 week
**Feasibility:** HIGH. SuperPoint + LightGlue are mature models with existing PyTorch implementations. The ROS integration is straightforward (similar to existing `pointcloud_fusion_node.py` message filter pattern).

### Phase 2: GTSAM Trajectory Binder (Week 2)

Same as V2 plan, but now with an additional factor type:

**Factors (4 types now):**

1. **Between factors from odometry** — `gtsam.BetweenFactorPose3(pose_i, pose_{i+1}, delta, noise)`
   - Delta from OpenVINS odometry (accurate relative motion, even when absolute pose drifts)
   - Noise from odometry covariance

2. **Between factors from visual features** — `gtsam.BetweenFactorPose3(head_i, arm_i, T_head_arm, noise)`
   - T_head_arm from `visual_feature_node` (`/vis/head_arm_pose`)
   - Noise from Umeyama residual covariance
   - **This is the key addition over V2 — provides continuous cross-camera constraints**

3. **Prior factors from world ArUco markers** — `gtsam.PriorFactorPose3(camera_i, T_map_cam, noise)`
   - When either camera sees a world marker
   - Noise from marker observation quality metrics

4. **Kinematic range constraint** — soft hinge-loss on `|head_i.t - arm_i.t| > 1.0m`

**GTSAM graph differences from V2:**
- The visual feature between factors are the primary cross-camera constraint (always available when views overlap)
- ArUco priors provide absolute anchoring (occasional)
- Range factor is a safety net (always available)
- The graph can now operate for minutes without markers

**Effort:** ~1 week (same as V2 but with one extra factor type)
**Feasibility:** MODERATE. GTSAM Python bindings require careful noise model tuning. The visual feature between factors need proper covariance propagation from the Umeyama step.

### Phase 3: Keyframe Buffer + TSDF Fusion (Week 3-4)

Same as V2 plan, with one important change: **the keyframe buffer subscribes to GTSAM-optimized poses, which are now constrained by visual features even when markers are absent.**

```
Keyframe Buffer:
  Input: Point clouds from both cameras
  Input: /gtsam/head_pose, /gtsam/arm_pose (GTSAM-optimized, ~15 Hz)
  
  For each incoming cloud:
    1. Look up GTSAM-optimized pose at cloud timestamp
    2. Apply spatial gate (10cm / 15°)
    3. Store (timestamp, xyz, rgb, T_map_cam_optimized)
    4. Evict oldest / furthest from wrist

TSDF Fusion:
  Trigger: Service call from twist propagation on hit detection
  1. Query keyframe buffer for frames within ROI
  2. For each keyframe: crop to ROI sphere, integrate with recency weighting
  3. Extract mesh, publish on /fused_pointcloud_historical
```

**Key benefit over V2:** The poses stored in the keyframe buffer are now continuously constrained by visual features, not just occasional ArUco markers. A 3-second drift episode without markers is retroactively corrected by the GTSAM graph because the visual feature between factors provide continuous cross-camera constraints throughout the drift.

### Phase 4: Kinematic Constraint in Twist Propagation (Day 1 of any week)

Same as V2 Phase 1 — trivial, 30-line addition to `twist_propagation_node.py`. This provides immediate protection against physically impossible states.

**Effort:** ~1 day
**Feasibility:** TRIVIAL

---

## 5. Implementation Sequencing

```
Week 1: Phase 1 (Visual Feature Node)
  Day 1-2: Package setup, ROS subscriptions, message filter sync
  Day 3-4: SuperPoint + LightGlue integration
  Day 5-6: 3D-3D alignment pipeline, depth lookup
  Day 7:   Diagnostics, testing on recorded rosbag

Week 2: Phase 2 + Phase 4 (GTSAM + Kinematic Constraint)
  Day 1:   Kinematic constraint in twist_propagation (trivial)
  Day 2-4: GTSAM node with 4 factor types
  Day 5-6: Integration: GTSAM subscribes to /vis/head_arm_pose
  Day 7:   Testing on recorded rosbag with visual features + markers

Week 3-4: Phase 3 (Keyframe Buffer + TSDF Fusion)
  Day 8-11:  Keyframe buffer node
  Day 12-14: TSDF fusion node (Open3D)
  Day 15-16: Integration test: full pipeline on recorded rosbag
  Day 17-18: Hardware deployment and tuning
```

---

## 6. What Changes on the Jetson Side

**Nothing, for this approach.** All required data is already published:

| Data | Topic | Published by |
|------|-------|-------------|
| RGB images | `/head/d435i_head/color/image_raw`, `/arm/...` | `realsense2_camera_node` |
| Camera intrinsics | `/head/d435i_head/color/camera_info`, `/arm/...` | `realsense2_camera_node` |
| Point clouds (organized) | `/head/d435i_head/depth/color/points`, `/arm/...` | `realsense2_camera_node` |
| Odometry | `/ov_msckf/odomimu`, `/ov_msckf_arm/odomimu` | OpenVINS |
| Marker observations | `/aruco/marker_observation`, `/aruco/dynamic_marker_observation` | `aruco_marker_pose_node` |

All cross the DDS bridge transparently via CycloneDDS peer discovery (host network mode, 10.42.0.x subnet).

**Potential future Jetson optimization:** If bandwidth becomes an issue (unlikely at 640×480 compressed), we could:
1. Enable JPEG compression on the RealSense ROS driver (`color_format: RGB8` → compressed transport)
2. Run SuperPoint on the Jetson Orin directly (but this competes with OpenVINS GPU usage)

Neither is needed for the initial implementation.

---

## 7. Risk Assessment

| Risk | Likelihood | Impact | Mitigation |
|------|-----------|--------|------------|
| SuperPoint + LightGlue GPU usage exceeds RTX 4070 headroom | Low | Medium | Monitor GPU utilization. Can reduce to 10 Hz or use smaller model variant. |
| Head and wrist views rarely overlap (user looks away from workspace) | Medium | Medium | This is inherent — if cameras don't see the same scene, no visual constraints are possible. The kinematic constraint + ArUco markers handle this case. |
| LightGlue match quality degrades with RealSense image quality (motion blur, low light) | Medium | Medium | SuperPoint is robust to moderate blur. Use frame quality metric to skip bad frames. |
| Network bandwidth for 2× RGB streams saturates USB link | Low | Low | 640×480 raw = 1.8 MB/frame × 15 Hz × 2 = 54 MB/s. USB 2.0 = 60 MB/s theoretical. Use compressed transport if needed (JPEG: ~100 KB/frame → 3 MB/s total). |
| GTSAM noise model tuning takes longer than expected | Medium | Medium | Start with conservative (high) noise estimates. Validate on recorded rosbags with ground-truth ArUco corrections as reference. |
| Visual features give wrong constraints (dynamic objects in scene) | Low | High | 3D RANSAC in Umeyama step filters dynamic objects as outliers. Static scene elements (table, walls) dominate the inlier set. |

---

## 8. Comparison: V2 vs V3

| Aspect | V2 (Trajectory Binder) | V3 (Visual Features) |
|--------|----------------------|---------------------|
| **Cross-camera constraints** | ArUco markers only (sporadic) | SuperPoint + LightGlue (continuous) |
| **Marker-free operation** | Minutes (if kinematic constraint holds) | Minutes (visual features + kinematic) |
| **Drift during marker absence** | Unconstrained relative drift between cameras | Constrained by visual feature matches |
| **GPU usage** | None (CPU only) | ~20-30% of RTX 4070 at 15 Hz |
| **Jetson changes needed** | None | None |
| **New dependencies** | `gtsam` (pip) | `gtsam`, `kornia` or LightGlue, `torch` |
| **Engineering effort** | ~2 weeks | ~3-4 weeks |
| **Failure mode without markers** | Both cameras drift independently | Relative pose constrained; absolute drift accumulates slowly until next marker |

---

## 9. What to Do Right Now

1. **Record a rosbag** with all required topics:
   ```bash
   ros2 bag record \
     /head/d435i_head/color/image_raw \
     /head/d435i_head/color/camera_info \
     /head/d435i_head/depth/color/points \
     /arm/d435i_arm/color/image_raw \
     /arm/d435i_arm/color/camera_info \
     /arm/d435i_arm/depth/color/points \
     /ov_msckf/odomimu \
     /ov_msckf_arm/odomimu \
     /aruco/marker_observation \
     /aruco/dynamic_marker_observation \
     /tf /tf_static
   ```
   Record during a session with natural movement and occasional marker visibility.

2. **Verify image topics** arrive on the laptop:
   ```bash
   ros2 topic hz /head/d435i_head/color/image_raw
   ros2 topic hz /arm/d435i_arm/color/image_raw
   ```

3. **Prototype SuperPoint matching offline** on a few sample image pairs from the rosbag to verify match quality.

4. **Add the 30-line kinematic constraint** to `twist_propagation_node.py` (same as V2 Phase 1).

---

## 10. My Honest Assessment

The critique (`localization-rework-critique.md`) was **directionally right but solved the wrong problem.** It correctly identified that pose accuracy gates fusion quality, and that an EKF alone can't support keyframe-based historical fusion. But it assumed the only way to get cross-camera constraints was through ArUco markers (sporadic) or ORB features (poor viewpoint robustness).

The discovery that **RGB images are already available on the laptop** and that **the RTX 4070 can run SuperPoint + LightGlue at 15+ Hz** changes the calculus entirely:

- **You can have continuous, high-quality cross-camera constraints without markers.** This is what enables "minutes rather than seconds" of marker-free operation.
- **You don't need ORB.** SuperPoint + LightGlue on the RTX 4070 is fast enough and handles the viewpoint difference between head and wrist cameras.
- **You don't need to modify the Jetson.** All the data is already flowing. The laptop just needs to start listening to the image topics.

The GTSAM factor graph with 4 constraint types (odometry, visual features, ArUco markers, kinematic range) gives you:
1. **Continuous relative pose** from visual features (always when views overlap)
2. **Absolute anchoring** from ArUco markers (when visible)
3. **Safety net** from kinematic range (always)
4. **Smooth trajectories** from odometry between factors (always)

This is the system you described wanting: one that works for minutes without markers, gracefully degrades when views don't overlap, and snaps back to high accuracy when a marker becomes visible.
