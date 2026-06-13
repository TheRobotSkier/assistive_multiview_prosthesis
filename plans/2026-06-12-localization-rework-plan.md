# Localization Rework — Implementation Plan & Feasibility Review

**Date:** 2026-06-12
**Source:** `localization-rework.md` (architectural discussion with AI assistant)
**Codebase:** `assistive_multiview_prosthesis` (laptop side) + Jetson worktree

---

## 1. Current Architecture Summary

### 1.1 Hardware Split

| Location | Role | Key Software |
|----------|------|-------------|
| **Jetson Orin** | Edge sensor hub | Two independent OpenVINS instances (head/arm), IMU driver at 400 Hz, ArUco marker detection + correction layer |
| **Laptop** | Central processing | TF relay from odometry, point cloud fusion, twist propagation, segmentation bridge, grasp preshaping |

### 1.2 Current Data Flow

```
Jetson:
  D435i #1 (head) --> OpenVINS /ov_msckf/odomimu      --> ArUco correction --> corrected odom
  D435i #2 (arm)  --> OpenVINS /ov_msckf_arm/odomimu   --> ArUco correction --> corrected odom
  Color images     --> ArUco detector (aruco_marker_pose_node.py)
                        - Detects static world markers (known T_map_marker)
                        - Detects dynamic wrist markers
                        - Computes T_map_global correction via EKF-like soft/hard updates
                        - Publishes corrected odometry + covariance

Laptop receives via DDS:
  openvins_odom_tf_relay.py:
    - Subscribes to /ov_msckf/odomimu, /ov_msckf_arm/odomimu (BEST_EFFORT, ~200 Hz)
    - Publishes marker_map -> head_imu, marker_map -> arm_imu TF transforms
    - Publishes static imu -> cam0 extrinsics
    - Self-calibrates extrinsics from RealSense TF tree (optional)

  openvins_realsense_tf_bridge_node.py:
    - Bridges OpenVINS cam0 frames to RealSense link frames
    - Publishes nominal D435i static chain (link -> depth_frame -> color_frame -> ...)
    - Uses liveness timer to re-send on /tf for CycloneDDS reliability

  hand_pose_publisher.py:
    - Looks up world -> wrist_link TF at 50 Hz
    - Publishes /hand_pose (PoseStamped, latched)

  pointcloud_fusion_node.py:
    - Subscribes to both camera point clouds (~15-30 Hz each)
    - Transforms to marker_map via TF2 (using latest available transform)
    - Merges into single cloud, distance-filters (2m from arm), bbox-removes arm/hand
    - Voxel-downsamples (5mm leaf), publishes /fused_pointcloud

  twist_propagation_node.py:
    - Estimates hand twist from pose buffer (finite differences) or external odom twist
    - Propagates pose forward (2s horizon, 0.02s steps)
    - Checks collision with /fused_pointcloud via KDTree
    - On hit: publishes click cluster to /segmentation/click_positive
    - State machine: IDLE -> WAITING_FOR_SEGMENTATION -> IDLE

  segmentation_ros2_node.py:
    - Accumulates positive/negative clicks
    - Crops cloud to ROI sphere around click centroid
    - Sends to inference server (localhost:5678)
    - Publishes /segmentation/object_cloud
```

### 1.3 What Already Works Well

1. **Dual OpenVINS**: Running independently, publishing odometry at ~200 Hz with covariance
2. **ArUco Drift Correction**: Sophisticated correction layer on Jetson with:
   - Quality metrics (reprojection error, view angle, corner geometry)
   - Temporal stability checks (8-frame stable requirement)
   - Soft (Kalman-gain) and hard (direct set) correction modes
   - Chi-squared innovation gating
   - Cooldown periods and periodic correction intervals
   - Proper covariance propagation and growth
3. **TF Bridge Infrastructure**: Robust handling of CycloneDDS /tf_static unreliability with liveness timers, self-calibration of extrinsics, TF-wait gates
4. **Point Cloud Fusion**: Working pipeline with distance filtering, bbox arm removal, voxel downsampling
5. **Twist Propagation + Collision Detection**: Functional predictive system with KDTree spatial queries
6. **Segmentation Pipeline**: Click-driven SAM-based segmentation with ROI cropping

### 1.4 What Doesn't Work Well (Pain Points)

1. **Wild Drift**: OpenVINS covariance occasionally balloons (mentioned in conversation)
2. **No Cross-Camera Constraints**: The two OpenVINS instances are completely independent except for ArUco corrections. When the wrist camera is out of view of markers for extended periods, it drifts independently
3. **Single-Frame Fusion**: The fused point cloud only uses the current frame pair -- no historical accumulation, no multi-view fusion, resulting in incomplete coverage of objects
4. **No Dense Map**: Objects are seen from only one or two viewpoints, leaving gaps for segmentation
5. **No Feature-Based Relocalization**: ArUco markers are the only visual anchors; there are no sparse feature landmarks for cross-camera or temporal alignment

---

## 2. Gap Analysis: Proposed vs. Current Architecture

| Component | Proposed (localization-rework.md) | Current State | Gap |
|-----------|----------------------------------|---------------|-----|
| **Dual OpenVINS** | Run independently as local odometry sources | Already done | None |
| **Factor Graph Optimizer (GTSAM)** | Central FGO with iSAM2 sliding window | Not present -- uses TF2 + ArUco correction | Major gap -- would require new dependency and significant integration |
| **ArUco as FGO Priors** | Inject marker detections as Prior/Between factors | Already doing correction, but via EKF-style updates in Jetson-side node | Moderate -- would need to move correction logic to laptop FGO |
| **Sparse Landmarks (SuperPoint + LightGlue)** | Multi-modal feature matching with depth validation | Not present | Major gap -- new ML models, GPU load |
| **Smart Projection Factors** | Landmarks as implicit graph factors | Not present | Part of GTSAM integration |
| **Human Kinematic Constraint** | Soft-bounded distance factor (<=1m) | Not present | Minor -- could be added as simple check in twist propagation or fusion |
| **Keyframe Buffer** | Spatial/temporal eviction with overlap culling | Not present -- fusion uses only latest clouds (0.5s max age) | Moderate -- new component |
| **On-Demand TSDF Fusion** | Open3D ScalableTSDFVolume with recency weighting | Not present | Moderate -- Open3D dependency, new pipeline stage |
| **Two-Stage Trigger** | Coarse (raw depth + twist) -> Fine (TSDF fusion) | Partially done -- twist propagation already does coarse collision detection | Minor -- the trigger mechanism exists; TSDF fusion is the missing downstream component |

---

## 3. Detailed Implementation Plan

### Phase 1: Keyframe Buffer & Basic Historical Fusion (Low Risk, High Value)

**Rationale:** Before tackling the factor graph, build the data management infrastructure. This phase alone yields significant improvement by accumulating multiple viewpoints of objects before segmentation.

#### 1.1 Keyframe Buffer Component (`src/keyframe_buffer/`)

- **Language:** Python (rclpy) -- consistent with existing codebase, sufficient for 15-30 Hz camera rates
- **Data Structure:** Per-camera deque of `(timestamp_ns, xyz, rgb, pose_matrix_4x4, frame_id)` with configurable max size
- **Spatial Gate:** Save keyframe only if translation > 10cm OR rotation > 15deg from last saved keyframe
- **Eviction Policies:**
  - Max buffer size (e.g., 50 keyframes per camera)
  - Age-based eviction (oldest first) when overlapping
  - Distance-based eviction (furthest from current wrist position)
- **Integration:** Subscribes to the same cloud topics as `pointcloud_fusion_node`, also subscribes to `/tf` for pose lookup at each keyframe's timestamp
- **Output:** Stores keyframe data indexed by timestamp; exposes a service or topic for retrieval

**Files to create/modify:**
- `src/keyframe_buffer/keyframe_buffer/keyframe_buffer_node.py` -- new node
- `src/keyframe_buffer/setup.py`, `package.xml` -- new package
- `config/prosthesis_config.yaml` -- add `keyframe_buffer` section

**Effort:** ~3-5 days

#### 1.2 On-Demand TSDF Fusion Node

- **Dependency:** `open3d` (pip install open3d) -- Python package, no C++ build required
- **Trigger:** Service call from twist propagation when hit is detected (instead of or in addition to current click publishing)
- **Pipeline:**
  1. Receive trigger with ROI center point
  2. Query keyframe buffer for frames within ROI sphere (e.g., 20cm radius)
  3. Look up each keyframe's pose (from stored pose or TF2)
  4. Crop each cloud to ROI sphere
  5. Apply arm bbox mask (reuse existing bbox transforms from `pointcloud_fusion_node`)
  6. Integrate into `open3d.pipelines.integration.ScalableTSDFVolume` with recency weighting
  7. Extract colored point cloud via `extract_point_cloud()`
  8. Publish on a new topic (e.g., `/fused_pointcloud_historical`) or replace current `/fused_pointcloud`
- **Recency Weighting:** New frames weight=1.0, older frames decay exponentially with time constant ~5s

**Files to create/modify:**
- `src/pointcloud_fusion/pointcloud_fusion/tsdf_fusion_node.py` -- new node (or extend existing fusion node)
- `config/prosthesis_config.yaml` -- add `tsdf_fusion` section

**Effort:** ~3-5 days

**Feasibility:** HIGH. Open3D Python API is mature and well-documented. The TSDF volume with a 20cm ROI at 5mm resolution is ~64K voxels -- integration is sub-millisecond. Main risk is the Open3D dependency (package size ~200MB), but it's pip-installable.

### Phase 2: Human Kinematic Constraint & Twist Improvements (Low Risk)

#### 2.1 Kinematic Distance Check in Twist Propagation

- **Add parameter:** `max_head_wrist_distance_m` (default 1.0)
- **Logic:** Before publishing twist or clicks, compute Euclidean distance between head pose and wrist pose (both available via TF2). If distance > threshold, apply a warning log and optionally suppress propagation (configurable)
- **Alternative:** Scale twist magnitude down proportionally as distance approaches the limit (soft constraint)

**Files to modify:**
- `src/twist_propagation/twist_propagation/twist_propagation_node.py` -- add distance check to `_run_idle_cycle()`
- `config/prosthesis_config.yaml` -- add parameter

**Effort:** ~1 day

**Feasibility:** TRIVIAL. Both poses are already available in the TF tree.

#### 2.2 Covariance-Aware Twist Estimation

- **Current:** Twist is estimated from pose buffer via finite differences or external odom twist
- **Improvement:** When external odom covariance is available, use it to scale the propagation horizon or collision threshold (higher covariance -> shorter horizon or larger threshold)
- **Current already does this partially:** `enable_covariance_propagation` and `max_prediction_covariance_trace` exist

**Effort:** ~1 day

### Phase 3: GTSAM Factor Graph Integration (High Risk, High Value)

**Critical Assessment:** This is the most architecturally significant change. The current ArUco correction layer on the Jetson already performs a similar function (drift correction via marker observations). Moving this to a laptop-side FGO with GTSAM would:

**Pros:**
- More principled optimization (re-linearization, outlier rejection via robust kernels)
- Native handling of asynchronous measurements
- Can incorporate multiple constraint types (odometry, markers, kinematic, features)

**Cons:**
- Major new dependency (GTSAM ~500MB compiled, complex build)
- Requires rewriting the ArUco correction logic (currently ~2250 lines of battle-tested Jetson code)
- Learning curve for GTSAM/iSAM2 API
- Risk of regressing on the current working correction system
- The Jetson would still need to detect markers -- moving correction to laptop adds network round-trip latency

**Recommendation:** DEFER this phase. The current ArUco correction works. If drift remains problematic after Phase 1-2 improvements, consider a limited GTSAM integration:

#### 3.1 (If Needed) Minimal GTSAM Node

- **Language:** Python (gtsam Python bindings via `pip install gtsam`)
- **Inputs:** 
  - Head odometry (nav_msgs/Odometry from Jetson)
  - Arm odometry (nav_msgs/Odometry from Jetson)
  - ArUco marker observations (custom message from Jetson -- already published as `MarkerPoseObservation`)
  - Dynamic marker observations (wrist marker -- `DynamicMarkerObservation`)
- **Graph Structure:**
  - Pose nodes at odometry rate (decimated to ~30 Hz for graph, raw odom used as between factors)
  - Between factors from odometry deltas (with covariance as information matrix)
  - Prior factors from world ArUco detections (T_map_global correction)
  - Between factors from wrist ArUco detections (head -> wrist when marker visible)
  - Custom distance constraint factor (soft-bounded at 1.0m)
- **Sliding Window:** iSAM2 with 10-15 second lag
- **Output:** Publish optimized poses to TF (replacing/augmenting `openvins_odom_tf_relay`)

**Files to create:**
- `src/gtsam_graph/gtsam_graph/gtsam_estimator_node.py`
- `src/gtsam_graph/` package

**Effort:** ~2-3 weeks

**Feasibility:** MODERATE. GTSAM Python bindings are available via pip. The main challenge is correctly setting up the factor graph structure, tuning noise models, and ensuring real-time performance. Risk of introducing worse behavior than current system if not carefully tuned.

### Phase 4: Sparse Feature Landmarks (High Complexity, Low Urgency)

**Assessment:** The localization-rework.md proposes SuperPoint + LightGlue for cross-camera feature matching with depth validation. This is the most complex and least immediately necessary component.

**Why it's low priority:**
1. ArUco markers already provide strong relative constraints between head and wrist
2. The environment is a workshop/lab with the user's own body and objects -- SuperPoint features on blank walls or fast-moving wrist views may not be reliable
3. GPU load: SuperPoint + LightGlue would compete with the segmentation model for GPU resources
4. The current system doesn't require a persistent map -- it only needs poses for point cloud alignment

**If implemented, architecture would be:**
- Async Python node running at 2-5 Hz
- SuperPoint (via `torch.hub` or ONNX) for keypoint detection on grayscale
- LightGlue (via `kornia` or ONNX) for matching
- 3D RANSAC filter using depth values to reject geometrically impossible matches
- SmartProjectionPoseFactor in GTSAM (requires Phase 3 FGO)

**Effort:** ~3-4 weeks (including model optimization for laptop GPU)

**Feasibility:** LOW-MODERATE. The ML models can run on laptop GPU but inference time (50-200ms per pair) makes this an async/slow-loop component. The value is uncertain for this specific use case.

---

## 4. Concrete Recommendations

### 4.1 What to Build Now (Phases 1-2)

```
Priority 1: Keyframe Buffer + On-Demand TSDF Fusion
  - Directly improves segmentation quality by providing multi-view fused point clouds
  - Low risk, well-understood technology (Open3D)
  - Independent of other architectural changes
  - Can be integrated with existing twist propagation trigger

Priority 2: Human Kinematic Constraint
  - Simple distance check that can prevent physically impossible states
  - ~1 day of work
  - Can be added to twist propagation without any architectural changes

Priority 3: Covariance-Aware Propagation Tuning
  - Already partially implemented -- review and tune existing parameters
  - May help reduce false triggers during high-uncertainty periods
```

### 4.2 What to Defer (Phases 3-4)

```
Defer: GTSAM Factor Graph
  - High engineering cost (2-3 weeks)
  - Current ArUco correction on Jetson already works
  - Only pursue if Phase 1-2 improvements don't sufficiently reduce drift

Defer: SuperPoint + LightGlue Landmarks
  - High complexity, uncertain value for this use case
  - GPU contention with segmentation model
  - Only consider if ArUco markers are insufficient for head-wrist constraint
```

### 4.3 Quick Wins (Minimal Code Changes)

1. **Increase pose buffer size in `pointcloud_fusion_node`:** Currently uses `cloud_max_age_s: 0.5` -- consider making this larger when the TF wait gate has confirmed connectivity, to allow more historical clouds into fusion

2. **Add wrist distance logging to twist propagation:** Track the head-wrist distance over time and log warnings when it exceeds 1.0m -- this data will help determine if a kinematic constraint is needed

3. **Expose ArUco marker covariance on laptop:** The Jetson-side `aruco_marker_pose_node.py` already computes sophisticated covariance. Subscribe to `MarkerPoseObservation` on the laptop to get uncertainty estimates for the current correction

4. **Add `/tf` connection health monitoring:** The `pointcloud_fusion_node` already has TF-wait gates. Extend this to publish a diagnostic topic so the pipeline manager can change behavior when TF quality degrades

---

## 5. Architectural Fit Assessment

### 5.1 What the localization-rework.md Got Right

1. **Dual OpenVINS as local odometry sources:** Correct -- this is already the architecture
2. **ArUco markers as anchors:** Correct -- the current system uses this effectively
3. **Laptop as the heavy processing node:** Correct -- matches current hardware split
4. **Keyframe-based on-demand fusion:** Excellent idea for this use case
5. **TSDF for handling dynamic objects:** Correct choice for ROI-sphere fusion
6. **ROI sphere cropping before fusion:** Already done in segmentation, should extend to fusion
7. **Arm self-filter:** Already implemented via bbox removal

### 5.2 What the localization-rework.md Got Wrong or Oversimplified

1. **GTSAM as "the answer":** The conversation presents factor graph optimization as the definitive solution without acknowledging that the current EKF-style ArUco correction already solves much of the same problem. A full FGO rewrite would be a 4-6 week engineering project with significant risk.

2. **SuperPoint + LightGlue as straightforward:** Deploying these models in a real-time ROS2 pipeline on a laptop that's also running segmentation is non-trivial. The conversation doesn't address GPU memory contention, inference latency, or the challenge of robust feature matching across extreme viewpoint changes (head vs. wrist).

3. **"Human kinematic constraint" implementation:** The conversation suggests a custom GTSAM factor, but doesn't mention that a simple Euclidean distance check (trivial to add to the existing twist propagation node) achieves 90% of the benefit.

4. **Rust for keyframe management:** The user correctly pushed back on this ("wrapping rust in cpp to get it to talk with ros2 was a hassle"). Python with numpy is perfectly adequate for 15-30 Hz keyframe management.

5. **SmartProjectionPoseFactor complexity:** The conversation recommends this GTSAM feature without noting that it requires all camera poses observing a landmark to be in the active optimization window simultaneously, which requires careful graph management.

---

## 6. Implementation Sequencing

```
Week 1-2:  Phase 1.1 -- Keyframe Buffer Node
           - New package, basic spatial gating, age-based eviction
           - Unit tests for eviction logic
           - Integration test with existing pointcloud_fusion topics

Week 3-4:  Phase 1.2 -- TSDF Fusion Node  
           - Open3D integration, recency weighting
           - Integration with keyframe buffer service
           - Modify twist_propagation to trigger TSDF fusion on hit
           - Comparison testing: TSDF-fused cloud vs. current single-frame cloud

Week 5:    Phase 2 -- Kinematic Constraint & Tuning
           - Distance check in twist propagation
           - Covariance-aware parameter tuning
           - End-to-end testing with real hardware

Week 6+:   Evaluation gate: If drift is still problematic after Phase 1-2:
           - Begin Phase 3 (GTSAM integration) as a parallel branch
           - Otherwise, focus on tuning and hardening Phase 1-2
```

---

## 7. Risk Assessment

| Risk | Likelihood | Impact | Mitigation |
|------|-----------|--------|------------|
| Open3D integration causes segfaults | Low | High | Use Python bindings (not C++), test with recorded bags first |
| Keyframe buffer memory growth | Medium | High | Hard limit on buffer size, monitor with ROS diagnostics |
| TSDF fusion latency exceeds grasp deadline | Low | Medium | 20cm ROI at 5mm voxel = 64K voxels, integration is <10ms on laptop |
| GTSAM build/install failures | Medium | High | Defer Phase 3, consider `pip install gtsam` for Python-only path |
| ArUco correction regression if moved to laptop | High | Critical | Keep Jetson-side correction running; add laptop FGO as augmentation, not replacement |
| GPU contention between segmentation and feature extraction | Medium | Medium | Keep feature extraction async and low-rate (2 Hz); skip if segmentation quality degrades |

---

## 8. Summary

The localization-rework.md conversation presents an aspirational architecture that is largely correct in principle but underestimates the engineering complexity of a full rewrite and overestimates the marginal benefit over the current working system.

**Recommendation:** Implement Phases 1-2 (Keyframe Buffer + TSDF Fusion + Kinematic Constraint) as incremental improvements to the existing architecture. These changes are low-risk, high-value, and directly address the stated need for "a super good fused point cloud, maybe even with some history of different views."

**Defer** the GTSAM factor graph and SuperPoint/LightGlue landmark tracking until empirical evidence shows they are necessary. The current ArUco-based correction layer on the Jetson is sophisticated and battle-tested -- it should not be replaced without a clear demonstration that it is insufficient.
