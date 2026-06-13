# Localization Rework — Revised Plan (Trajectory Binder Approach)

**Date:** 2026-06-12
**Trigger:** `localization-rework-critique.md` — user feedback that 20cm drift makes the fused point cloud unusable
**Status:** Analysis complete, implementation plan follows

---

## 0. Evaluation of the Critique

### What the critique gets right

1. **"You cannot use history-based fusion to fix bad tracking"** — Correct. TSDF with 20cm pose errors will produce space-carved garbage, not a useful fused cloud.

2. **"An EKF can only update the present moment"** — Correct, and this matters once a keyframe buffer exists. The Jetson-side ArUco EKF snaps the *current* pose but cannot retroactively fix poses captured by the keyframe buffer during a 3-second drift episode.

3. **"An FGO re-optimizes the timeline"** — Correct. When an ArUco marker becomes visible after a drift period, iSAM2 retroactively smooths the entire sliding window, fixing the keyframe poses that were captured with drifted odometry.

### What the critique gets wrong or oversimplifies

1. **"Your current ArUco EKF layer is failing"** — This is an overstatement. The EKF correction layer (`aruco_marker_pose_node.py`, 2250 lines) is sophisticated: it has temporal stability gating (8-frame stable requirement), chi-squared innovation gating, soft/hard correction modes, cooldown periods, and proper covariance propagation. It *does* correct drift. The issue is that it corrects only the present, not the past — and this only matters once a keyframe buffer exists.

2. **"Put GTSAM and the keyframe buffer in Week 1-2"** — This is riskier than presented. GTSAM (even Python bindings) has a learning curve. Building a correct factor graph with proper noise models takes time. Starting with the simplest useful component (the kinematic constraint) is better than trying to build the full graph in week 1.

3. **"ORB + 3D RANSAC for cross-camera feature matching"** — The critique proposes ORB as a CPU-friendly alternative to SuperPoint/LightGlue. However, ORB descriptor quality degrades significantly with the large viewpoint differences between head and wrist cameras. ORB is useful for temporal (within-camera) matching but unreliable for cross-camera matching. Until proven otherwise, the kinematic constraint and head-to-wrist ArUco detections are more reliable cross-camera constraints.

### Overall verdict

The critique correctly identifies that pose accuracy is the gating factor for fusion quality, and that an EKF alone cannot support keyframe-based historical fusion. However, it over-indexes on "build GTSAM first" without acknowledging that:

- The simplest GTSAM factor — a kinematic range constraint between head and wrist poses — provides immediate value
- The current EKF already mitigates drift in real-time; the GTSAM layer should *augment*, not *replace* it
- The keyframe buffer should be built in parallel with the trajectory binder, not after it, because the buffer needs the GTSAM-corrected poses to be useful

---

## 1. Root Cause Analysis: The 20cm Drift Problem

### 1.1 How drift happens

```
Timeline of a typical drift episode:

t=0s:   Wrist camera sees ArUco marker → OpenVINS pose is corrected → accurate
t=1s:   Wrist camera looks down at table → no markers, no visual features
        → OpenVINS accumulates IMU bias error → pose drifts 3cm
t=2s:   Still no markers, rapid hand movement → drift accelerates → 10cm error
t=3s:   Drift reaches 20cm → wrist pose is now 20cm away from true position
t=4s:   Wrist camera sees ArUco marker again → EKF snaps pose back → accurate now

But at t=4s, if a keyframe buffer captured clouds at t=2.5s with the drifted pose,
those clouds are permanently misaligned.
```

### 1.2 Why the two cameras drift independently

The two OpenVINS instances run **completely independently**. There are no cross-camera constraints in the estimation. The only thing tying them together is:

1. **World ArUco markers** — both cameras can see the same world markers, providing a shared global reference. But the wrist camera may not see markers for extended periods.
2. **Head-to-wrist ArUco detections** — when the head camera sees the wrist marker, a direct measurement of T_head_wrist is available. This is published as `DynamicMarkerObservation` by the Jetson ArUco node.
3. **No kinematic constraint** — nothing prevents the two poses from diverging beyond the physical reach of a human arm (~1m).

### 1.3 Available cross-camera constraints (already published, not yet used on laptop)

| Constraint | Message Type | Published by | Status |
|------------|-------------|-------------|--------|
| World ArUco marker observations | `MarkerPoseObservation` | Jetson ArUco node | Published, but NOT consumed on laptop |
| Dynamic wrist marker observations | `DynamicMarkerObservation` | Jetson ArUco node | Published, but NOT consumed on laptop |
| Odometry (head) | `nav_msgs/Odometry` | Jetson OpenVINS | Consumed by `openvins_odom_tf_relay.py` |
| Odometry (arm) | `nav_msgs/Odometry` | Jetson OpenVINS | Consumed by `openvins_odom_tf_relay.py` and `odom_to_pose_relay.py` |

**Key gap:** The `MarkerPoseObservation` and `DynamicMarkerObservation` messages contain rich data (pose with covariance, quality metrics, stability) that is **not currently used on the laptop side**. These messages provide exactly the information needed to constrain the two camera trajectories together.

---

## 2. Proposed Architecture: Lightweight GTSAM Trajectory Binder

### 2.1 Design principles

1. **Augment, don't replace.** The existing TF pipeline (`openvins_odom_tf_relay.py`, `openvins_realsense_tf_bridge_node.py`) continues to work. The GTSAM node adds optimized poses as an *alternative source* for downstream consumers.

2. **Start with the simplest useful graph.** Begin with odometry between factors + kinematic range constraint. Add ArUco factors incrementally.

3. **Keep the Jetson ArUco EKF running.** The Jetson-side correction provides real-time drift suppression. The laptop-side GTSAM provides retroactive smoothing for the keyframe buffer.

4. **Python-only GTSAM.** Use `pip install gtsam` for the Python bindings. No C++ build required.

### 2.2 System topology

```
┌─────────────────────────────────────────────────────────────────────────┐
│                              LAPTOP                                      │
│                                                                          │
│  ┌──────────────────────┐    ┌──────────────────────┐                    │
│  │ openvins_odom_tf_relay│    │ gtsam_trajectory_binder│ ← NEW           │
│  │  (existing)           │    │                        │                 │
│  │  marker_map→head_imu  │    │  Inputs:               │                 │
│  │  marker_map→arm_imu   │    │   /ov_msckf/odomimu    │                 │
│  │  *_imu→*_cam0 (static)│    │   /ov_msckf_arm/odomimu│                 │
│  └──────────────────────┘    │   /aruco/marker_obs     │ ← NEW SUB       │
│                               │   /aruco/dynamic_obs   │ ← NEW SUB       │
│  ┌──────────────────────┐    │                        │                 │
│  │ pointcloud_fusion    │    │  iSAM2 sliding window:  │                 │
│  │  (existing)           │    │   BetweenFactors(odom) │                 │
│  │  TF→marker_map        │    │   PriorFactors(aruco)  │                 │
│  │  merge+filter+publish │    │   BetweenFactors(dyn.) │                 │
│  └──────────────────────┘    │   RangeFactor(1.0m)     │                 │
│                               │                        │                 │
│  ┌──────────────────────┐    │  Output:                │                 │
│  │ keyframe_buffer      │←───│   /gtsam/head_pose      │                 │
│  │  (NEW)                │    │   /gtsam/arm_pose       │                 │
│  │  spatial/temporal     │    │   /gtsam/head_odom      │                 │
│  │  eviction             │    │   /gtsam/arm_odom       │                 │
│  └──────────────────────┘    └──────────────────────┘                    │
│            │                           │                                 │
│            ▼                           ▼                                 │
│  ┌──────────────────────┐    ┌──────────────────────┐                    │
│  │ tsdf_fusion          │    │ twist_propagation    │                    │
│  │  (NEW)                │    │  (existing)           │                    │
│  │  on-demand ROI fusion │    │  optionally sub to    │                    │
│  │  with corrected poses │    │  /gtsam/arm_pose      │                    │
│  └──────────────────────┘    └──────────────────────┘                    │
└─────────────────────────────────────────────────────────────────────────┘
```

### 2.3 Why this works better than the current approach

| Scenario | Current (EKF only) | With Trajectory Binder |
|----------|-------------------|----------------------|
| Wrist drifts 20cm over 3s, then sees marker | EKF snaps current pose; past keyframes corrupted | iSAM2 retroactively smooths all poses in window; keyframe poses are corrected |
| Wrist and head diverge >1m | No constraint; system proceeds with impossible geometry | Range factor penalizes divergence, keeping poses physically plausible |
| Head sees wrist marker | Detection published but ignored on laptop | Between factor binds head→wrist trajectories |
| World marker detected by one camera | EKF corrects that camera only | Prior factor constrains that camera's trajectory; range factor pulls the other camera |

---

## 3. Detailed Implementation Plan

### Phase 1: Kinematic Constraint in Twist Propagation (Day 1-2)

**Goal:** Immediate, low-risk improvement. Prevent the system from acting on physically impossible pose divergences.

#### Implementation

Add a distance check to `twist_propagation_node.py` `_run_idle_cycle()`:

```python
# New parameters
self.declare_parameter("max_head_wrist_distance_m", 1.0)
self._max_head_wrist_dist = self.get_parameter("max_head_wrist_distance_m").value

# In _run_idle_cycle(), after getting the latest pose:
# Look up head pose from TF (marker_map -> head_cam0)
# Compute Euclidean distance to wrist pose
# If distance > threshold, log warning and optionally suppress propagation
```

**Files modified:**
- `src/twist_propagation/twist_propagation/twist_propagation_node.py:1487` (in `_run_idle_cycle`)
- `config/prosthesis_config.yaml` — add `max_head_wrist_distance_m` parameter

**Verification:** With the system running, move the wrist far from the head. Verify that twist propagation logs a warning and (optionally) suppresses propagation when distance exceeds 1.0m.

**Effort:** ~1 day. TRIVIAL feasibility.

---

### Phase 2: GTSAM Trajectory Binder Node (Week 1-2)

**Goal:** Core architectural addition. Build the factor graph that binds the two camera trajectories together and provides retroactive smoothing.

#### 2.1 Package structure

```
src/gtsam_tracker/
├── gtsam_tracker/
│   ├── __init__.py
│   ├── trajectory_binder_node.py    # Main ROS node
│   ├── factor_graph.py              # GTSAM graph construction + iSAM2
│   └── utils.py                     # Quaternion / transform helpers
├── setup.py
├── package.xml
└── config/
    └── trajectory_binder.yaml
```

#### 2.2 Factor graph design

**State:** At each timestep (decimated to 15 Hz from 200 Hz odometry), two pose variables:
- `head_i` — 6-DOF pose in `marker_map` frame (represented as `gtsam.Pose3`)
- `arm_i` — 6-DOF pose in `marker_map` frame

**Factors (in order of implementation):**

1. **Between factors from odometry** — `gtsam.BetweenFactorPose3(head_i, head_{i+1}, delta, noise_model)`
   - Delta computed from consecutive odometry messages
   - Noise model from odometry covariance (or fixed diagonal)
   - Applied to both head and arm trajectories independently

2. **Kinematic range constraint** — Custom factor or `gtsam.RangeFactor`:
   - `distance(head_i.translation(), arm_i.translation()) ≤ 1.0m`
   - Implemented as a hinge-loss: zero cost below 1.0m, quadratic penalty above

3. **Prior factors from world ArUco markers** — `gtsam.PriorFactorPose3(camera_i, T_map_marker * T_marker_cam * T_cam_imu, noise_model)`
   - When a world marker is detected by either camera, compute the camera's pose from the marker
   - Add as a prior on the corresponding pose node
   - Noise model from the marker observation's covariance

4. **Between factors from head-to-wrist ArUco** — `gtsam.BetweenFactorPose3(head_i, arm_i, T_head_wrist, noise_model)`
   - When the head camera sees the wrist marker, compute T_head_wrist
   - Add as a between factor connecting the two trajectories

**iSAM2 configuration:**
- Sliding window: 15 seconds (225 pose nodes at 15 Hz)
- Relinearization: every 3 timesteps
- Wildfire threshold: 0.001

#### 2.3 Node implementation (`trajectory_binder_node.py`)

```python
class TrajectoryBinderNode(Node):
    """GTSAM-based trajectory binder for dual-camera localization."""

    def __init__(self):
        # ── Parameters ──
        self.declare_parameter("head_odom_topic", "/ov_msckf/odomimu")
        self.declare_parameter("arm_odom_topic", "/ov_msckf_arm/odomimu")
        self.declare_parameter("aruco_marker_topic", "/aruco/marker_observation")
        self.declare_parameter("aruco_dynamic_topic", "/aruco/dynamic_marker_observation")
        self.declare_parameter("graph_rate_hz", 15.0)
        self.declare_parameter("window_duration_s", 15.0)
        self.declare_parameter("kinematic_range_m", 1.0)
        self.declare_parameter("kinematic_range_sigma_m", 0.05)  # soft constraint width

        # ── State ──
        self._head_odom_buf: deque = deque(maxlen=300)
        self._arm_odom_buf: deque = deque(maxlen=300)
        self._aruco_observations: deque = deque(maxlen=500)
        self._dynamic_observations: deque = deque(maxlen=200)

        # ── GTSAM ──
        self._graph = gtsam.NonlinearFactorGraph()
        self._initial_estimate = gtsam.Values()
        self._isam = None  # Initialized after first solve
        self._parameters = gtsam.ISAM2Params()
        self._parameters.setRelinearizeThreshold(0.001)
        self._parameters.relinearizeSkip(3)

        # ── Subscriptions ──
        best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=30)
        self.create_subscription(Odometry, head_odom_topic, self._on_head_odom, best_effort)
        self.create_subscription(Odometry, arm_odom_topic, self._on_arm_odom, best_effort)
        # ArUco topics — need to check if these arrive via DDS from Jetson
        # (may need RELIABLE QoS and explicit topic bridging)

        # ── Publishers ──
        self._head_pose_pub = self.create_publisher(PoseStamped, "/gtsam/head_pose", 10)
        self._arm_pose_pub = self.create_publisher(PoseStamped, "/gtsam/arm_pose", 10)
        self._head_odom_pub = self.create_publisher(Odometry, "/gtsam/head_odom", 10)
        self._arm_odom_pub = self.create_publisher(Odometry, "/gtsam/arm_odom", 10)

        # ── Timer: graph optimization at rate ──
        self.create_timer(1.0 / graph_rate_hz, self._graph_tick)

    def _on_head_odom(self, msg: Odometry):
        """Buffer head odometry."""
        ts = stamp_to_sec(msg.header.stamp)
        self._head_odom_buf.append((ts, msg))

    def _on_arm_odom(self, msg: Odometry):
        """Buffer arm odometry."""
        ...

    def _graph_tick(self):
        """Main graph loop: add factors, optimize, publish."""
        # 1. Drain odometry buffers → add new pose nodes + between factors
        # 2. Drain ArUco observation buffers → add prior/between factors
        # 3. Add kinematic range factors for all active pose pairs
        # 4. iSAM2.update()
        # 5. iSAM2.calculateEstimate()
        # 6. Publish latest optimized poses
        # 7. Marginalize old poses (outside sliding window)
```

#### 2.4 Integration with TF pipeline

**Option A: Augment existing TF relay** (preferred for minimum disruption)

The GTSAM node publishes optimized poses on `/gtsam/head_pose` and `/gtsam/arm_pose`. A new TF publisher (or modified `openvins_odom_tf_relay.py`) subscribes to these and publishes them as TF:

```
/gtsam/head_pose → TF: marker_map_optimized → head_imu
/gtsam/arm_pose  → TF: marker_map_optimized → arm_imu
```

The keyframe buffer and TSDF fusion node use the optimized TF frame.

**Option B: Replace the existing TF relay**

The GTSAM node directly publishes TF transforms. This requires modifying `pointcloud_fusion_node.py` to subscribe to the optimized frame, which is a larger change.

**Recommendation:** Start with Option A. The existing TF relay continues to work for the current (non-buffered) fusion pipeline. The optimized poses are used by the keyframe buffer and TSDF fusion. This allows A/B comparison.

#### 2.5 Key decisions

1. **Odometry as between factors vs. priors:**
   - **Recommendation: Between factors.** OpenVINS provides excellent *relative* motion estimates (the delta between consecutive poses is accurate even when the absolute pose drifts). Using between factors allows the graph to re-distribute ArUco corrections across the window.

2. **Noise model for odometry between factors:**
   - Use the odometry covariance from OpenVINS (extracted from `msg.pose.covariance` and `msg.twist.covariance`)
   - Scale by dt between consecutive poses
   - Floor at a minimum value to prevent overconfidence

3. **Handling the Jetson-side ArUco EKF:**
   - The ArUco EKF continues to run on Jetson, correcting odometry in real-time
   - The GTSAM node receives the *corrected* odometry and *raw* ArUco observations
   - This is actually fine for between factors: the corrected odometry still provides accurate relative motion
   - The ArUco observations are used as *additional* priors in GTSAM, complementing the EKF's real-time correction

4. **Head-to-wrist ArUco distance check:**
   - The `DynamicMarkerObservation` message includes `distance_m` (already computed on Jetson)
   - This provides a direct measurement of T_head_wrist
   - Use this as a BetweenFactor in GTSAM, with covariance from the observation quality metrics

**Files to create:**
- `src/gtsam_tracker/gtsam_tracker/trajectory_binder_node.py` — main node (~400 lines)
- `src/gtsam_tracker/gtsam_tracker/factor_graph.py` — graph construction helpers (~200 lines)
- `src/gtsam_tracker/gtsam_tracker/utils.py` — quaternion/transform helpers (~100 lines)
- `src/gtsam_tracker/setup.py`, `src/gtsam_tracker/package.xml`
- `config/trajectory_binder.yaml`

**Files to modify:**
- `config/prosthesis_config.yaml` — add `gtsam_tracker` section
- `src/prosthesis_launch/launch/pipeline.launch.py` — add GTSAM node to camera section
- `requirements.txt` or equivalent — add `gtsam`

**Effort:** ~1.5-2 weeks
**Feasibility:** MODERATE. GTSAM Python bindings are mature but require careful tuning of noise models. Risk of worse behavior than current system if covariance models are wrong.

---

### Phase 3: Keyframe Buffer + TSDF Fusion with Corrected Poses (Week 3-4)

**Goal:** Build the keyframe buffer and on-demand TSDF fusion, now with GTSAM-corrected poses so that historical fusion works correctly.

#### 3.1 Keyframe Buffer Node (`src/keyframe_buffer/`)

Same design as in the original plan, with one critical change: **poses are queried from the GTSAM output rather than raw TF.**

```python
class KeyframeBufferNode(Node):
    """Buffers per-camera point clouds with associated poses for later fusion."""

    def __init__(self):
        # Subscribe to both camera clouds (same QoS as pointcloud_fusion)
        # For each incoming cloud:
        #   1. Compute spatial gate (translation >10cm OR rotation >15° from last)
        #   2. Look up optimized pose from GTSAM (via TF or direct topic)
        #   3. Store (timestamp, xyz, rgb, T_marker_map_camera)
        #   4. Evict if buffer exceeds max size (oldest or furthest from wrist)

    # Service: get_keyframes_in_roi(center, radius) → list of (xyz, rgb, pose)
```

#### 3.2 TSDF Fusion Node

- **Trigger:** Service call from twist propagation on hit detection
- **Pipeline:**
  1. Query keyframe buffer for frames within ROI sphere (20cm radius)
  2. For each keyframe:
     - Look up GTSAM-optimized pose at that keyframe's timestamp
     - Crop cloud to ROI sphere
     - Integrate into Open3D `ScalableTSDFVolume` with recency weighting
  3. Extract colored mesh via `extract_point_cloud()`
  4. Publish on `/fused_pointcloud_historical`

**Key difference from original plan:** The poses used for integration come from GTSAM's optimized output, not from raw TF. This ensures that even keyframes captured during drift episodes are correctly positioned after retroactive smoothing.

**Effort:** ~1.5-2 weeks
**Feasibility:** HIGH. Open3D integration is straightforward. The main value-add here is the GTSAM-corrected poses, which ensure fusion quality.

---

## 4. Integration with Existing Pipeline

### 4.1 What changes in the launch file

```python
# In pipeline.launch.py _launch_setup(), camera section:

# NEW: GTSAM Trajectory Binder
nodes.append(Node(
    package="gtsam_tracker",
    executable="trajectory_binder_node",
    name="gtsam_tracker",
    parameters=[_node_params(config, "gtsam_tracker")],
    output="screen",
))

# NEW: Keyframe Buffer
nodes.append(Node(
    package="keyframe_buffer",
    executable="keyframe_buffer_node",
    name="keyframe_buffer",
    parameters=[_node_params(config, "keyframe_buffer")],
    output="screen",
))

# EXISTING: pointcloud_fusion — unchanged
# EXISTING: openvins_odom_tf_relay — unchanged
```

### 4.2 What the keyframe buffer subscribes to

```
Subscriptions:
  /head/d435i_head/depth/color/points  (same as cam1_topic)
  /arm/d435i_arm/depth/color/points    (same as cam2_topic)
  /gtsam/head_pose  (PoseStamped, ~15 Hz)  ← NEW
  /gtsam/arm_pose   (PoseStamped, ~15 Hz)  ← NEW
```

### 4.3 What the twist propagation could optionally use

Currently twist propagation uses `/hand_pose` from `odom_to_pose_relay.py` (which reads arm odometry). It could optionally subscribe to `/gtsam/arm_pose` for a smoothed pose estimate, but this is not required for Phase 1-2 benefits.

### 4.4 What continues to work unchanged

- `pointcloud_fusion_node.py` — continues to use raw TF for real-time fusion
- `twist_propagation_node.py` — continues to use `/hand_pose` and `/fused_pointcloud`
- `segmentation_ros2_node.py` — continues to use `/fused_pointcloud`
- `openvins_odom_tf_relay.py` — continues to publish TF from odometry

### 4.5 Jetson-side topic bridging check

The `MarkerPoseObservation` and `DynamicMarkerObservation` messages are published inside the Jetson Docker container. Verify that these topics are discoverable on the laptop via DDS:

```bash
# On laptop, check if topics are available:
ros2 topic list | grep aruco
```

If not available, ensure the Docker container's DDS configuration allows cross-machine discovery (typically via `ROS_DOMAIN_ID` or CycloneDDS configuration).

---

## 5. Configuration Parameters

### 5.1 `config/prosthesis_config.yaml` additions

```yaml
gtsam_tracker:
  ros__parameters:
    head_odom_topic: "/ov_msckf/odomimu"
    arm_odom_topic: "/ov_msckf_arm/odomimu"
    aruco_marker_topic: "/aruco/marker_observation"
    aruco_dynamic_topic: "/aruco/dynamic_marker_observation"
    graph_rate_hz: 15.0
    window_duration_s: 15.0
    kinematic_range_m: 1.0
    kinematic_range_sigma_m: 0.05
    between_factor_sigma_translation: 0.01
    between_factor_sigma_rotation: 0.01
    aruco_prior_sigma_translation: 0.02
    aruco_prior_sigma_rotation: 0.02
    dynamic_between_sigma_translation: 0.03
    dynamic_between_sigma_rotation: 0.05

keyframe_buffer:
  ros__parameters:
    head_cloud_topic: "/head/d435i_head/depth/color/points"
    arm_cloud_topic: "/arm/d435i_arm/depth/color/points"
    head_pose_topic: "/gtsam/head_pose"
    arm_pose_topic: "/gtsam/arm_pose"
    max_keyframes_per_camera: 50
    spatial_gate_translation_m: 0.10
    spatial_gate_rotation_deg: 15.0
    max_keyframe_age_s: 30.0
    eviction_policy: "furthest_from_wrist"  # or "oldest_first"

tsdf_fusion:
  ros__parameters:
    roi_radius_m: 0.20
    voxel_size_m: 0.005
    recency_time_constant_s: 5.0
    output_topic: "/fused_pointcloud_historical"
    keyframe_buffer_service: "/keyframe_buffer/get_in_roi"

twist_propagation:
  ros__parameters:
    # ... existing params ...
    max_head_wrist_distance_m: 1.0
    enforce_kinematic_constraint: false  # set to true to suppress propagation
```

---

## 6. Implementation Sequencing

```
Week 1-2 (Phase 1 + Phase 2 start):
  Day 1-2:    Phase 1 — Kinematic constraint in twist propagation
              - Add max_head_wrist_distance_m parameter
              - Add distance check in _run_idle_cycle()
              - Test on hardware

  Day 3-7:    Phase 2 — GTSAM Trajectory Binder
              - Set up gtsam_tracker package
              - Implement factor_graph.py (BetweenFactor + RangeFactor first)
              - Implement trajectory_binder_node.py (subscriptions, buffering)
              - Test with recorded rosbag of odometry

  Day 8-10:   Phase 2 — ArUco integration
              - Subscribe to MarkerPoseObservation and DynamicMarkerObservation
              - Add PriorFactor and BetweenFactor (head→wrist) to graph
              - Verify ArUco topic availability on laptop via DDS
              - Test with recorded rosbag including ArUco detections

Week 3-4 (Phase 2 polish + Phase 3):
  Day 11-14:  Phase 2 — Tuning and validation
              - Tune noise models on recorded data
              - Add diagnostic logging (graph size, optimization time, residuals)
              - Integrate into pipeline.launch.py
              - End-to-end test with real hardware

  Day 15-20:  Phase 3 — Keyframe Buffer + TSDF Fusion
              - Implement keyframe_buffer_node.py
              - Implement tsdf_fusion_node.py (Open3D)
              - Keyframe buffer queries GTSAM-optimized poses
              - Test TSDF fusion quality before/after GTSAM correction
```

---

## 7. Risk Assessment

| Risk | Likelihood | Impact | Mitigation |
|------|-----------|--------|------------|
| ArUco topics not available on laptop via DDS | Medium | High | Verify DDS discovery first. If needed, add explicit topic relay on Jetson. |
| GTSAM optimization latency > real-time | Low | Medium | 15 Hz graph rate with 15s window ≈ 225 nodes. iSAM2 handles this easily on a laptop. If slow, reduce window or rate. |
| Incorrect noise models cause worse behavior than current system | Medium | High | Start with conservative (high) noise values. Validate on recorded rosbags before deploying. |
| Head-to-wrist ArUco detections are rare | Medium | Medium | The kinematic constraint provides a fallback. Even without direct ArUco measurements, the range factor prevents impossible divergence. |
| GTSAM Python bindings installation issues | Low | Medium | `pip install gtsam` works on Ubuntu. If issues arise, use gtsam from apt: `sudo apt install ros-jazzy-gtsam` |
| Keyframe buffer memory growth | Medium | High | Hard limit of 50 keyframes per camera at ~300K points each ≈ 60MB total per camera. Monitor with ROS diagnostics. |

---

## 8. Summary of Changes from Previous Plan

| Aspect | Previous Plan (localization-rework-plan.md) | Revised Plan |
|--------|--------------------------------------------|--------------|
| **Phase ordering** | Build keyframe buffer first, defer GTSAM | Build GTSAM trajectory binder first (or in parallel with keyframe buffer) |
| **GTSAM scope** | Full FGO replacing ArUco correction (deferred) | Lightweight trajectory binder using existing ArUco observations (build now) |
| **Kinematic constraint** | Simple distance check in twist propagation | Distance check in twist propagation + GTSAM RangeFactor for retroactive smoothing |
| **ArUco usage** | Keep on Jetson only | Consume `MarkerPoseObservation` and `DynamicMarkerObservation` on laptop as GTSAM factors |
| **Cross-camera features** | Defer SuperPoint/LightGlue | Defer both SuperPoint/LightGlue AND ORB — rely on ArUco + kinematic constraint |
| **Keyframe buffer** | Store poses from raw TF | Store poses from GTSAM-optimized output |
| **Risk of building TSDF without pose accuracy** | Acknowledged but deferred | Addressed by building GTSAM first |

---

## 9. What to Do Right Now (This Week)

1. **Verify ArUco topic availability** on the laptop:
   ```bash
   ros2 topic list | grep aruco
   ros2 topic echo /aruco/marker_observation --once
   ros2 topic echo /aruco/dynamic_marker_observation --once
   ```

2. **Add the kinematic distance check** to `twist_propagation_node.py` (~30 lines of code). This provides immediate protection against physically impossible states.

3. **Install GTSAM** and verify it works:
   ```bash
   pip install gtsam
   python -c "import gtsam; print(gtsam.__version__)"
   ```

4. **Record a rosbag** with all relevant topics during a session where the wrist camera drifts:
   ```bash
   ros2 bag record \
     /ov_msckf/odomimu \
     /ov_msckf_arm/odomimu \
     /head/d435i_head/depth/color/points \
     /arm/d435i_arm/depth/color/points \
     /aruco/marker_observation \
     /aruco/dynamic_marker_observation \
     /tf /tf_static
   ```
   This bag will be essential for developing and validating the GTSAM trajectory binder offline before deploying on hardware.
