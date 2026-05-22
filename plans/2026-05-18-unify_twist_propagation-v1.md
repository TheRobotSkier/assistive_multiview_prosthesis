# Unify Twist Propagation: Merge Two Versions Into One Working Node + RViz Visualization

## Objective

Consolidate the two existing twist-propagation / future-prediction implementations into a single, well-tested node in `src/twist_propagation/`, and add RViz visualization so operators can see predicted trajectories, collision geometry, and hit points in real time.

---

## Current State Analysis

### Version 1 — `future_pose_prediction_collision_node.py` (in `sensor_fusion_bringup`)

| Aspect | Detail |
|---|---|
| **Location** | `src/sensor_fusion_bringup/scripts/future_pose_prediction_collision_node.py` |
| **Package** | `sensor_fusion_bringup` (CMake, `ament_cmake`) |
| **Input** | `nav_msgs/Odometry` at ~200 Hz (from OpenVINS) + `PointCloud2` at ~8 Hz |
| **Twist source** | Directly from odometry twist field — no estimation needed |
| **Propagation** | Constant-twist model, pure-Python math (`propagate_pose`) |
| **Covariance** | Full 12×12 covariance propagation with process noise |
| **Collision** | Sphere geometry + KDTree proximity check per predicted pose |
| **Output topics** | `FuturePoseTrajectory` (custom msg), `Path`, `PointStamped` click, JSON status |
| **Custom message** | `sensor_fusion_msgs/msg/FuturePoseTrajectory.msg` |
| **Config** | `config/future_prediction_collision.yaml` (21 params) |
| **Launch** | `launch/future_prediction_collision.launch.py` |
| **Tests** | `test/test_future_prediction_collision.py` — 16 unit tests (pure functions) |
| **Docs** | `docs/future_prediction_collision_validation.md` |
| **State machine** | None — fires continuously on every cloud message |
| **Segmentation integration** | None — publishes click only, no downstream orchestration |
| **Activation** | Launch-time only (no services) |
| **Known issue** | "Tested to work only when very close to the object" — likely because it uses the raw odometry twist which may be small/noisy at distance, and the collision geometry is a single point-sphere (no arm extent modeling beyond `collision_geometry_radius_m`) |

### Version 2 — `twist_propagation_node.py` (in `twist_propagation`)

| Aspect | Detail |
|---|---|
| **Location** | `src/twist_propagation/twist_propagation/twist_propagation_node.py` |
| **Package** | `twist_propagation` (ament_python) |
| **Input** | `geometry_msgs/PoseStamped` at arbitrary rate + `PointCloud2` |
| **Twist source** | **Self-estimated** from pose buffer via finite differences + least-squares fit + EMA smoothing |
| **Propagation** | Constant-twist model, identical math (`_propagate_pose`) |
| **Covariance** | None |
| **Collision** | KDTree proximity check per predicted pose (no sphere radius param) |
| **Output topics** | `TwistStamped`, `PointStamped` click, JSON status |
| **Custom message** | None — uses only standard ROS messages |
| **Config** | Inline in `prosthesis_config.yaml` (two duplicated sections) |
| **Launch** | Included in `pipeline.launch.py`, `grasp_test.launch.py`, `digital_twin.launch.py`, `mock.launch.py` |
| **Tests** | `scripts/test_twist_propagation_integration.py` — integration test (7 test cases) |
| **Docs** | None |
| **State machine** | Full 3-state machine: IDLE → WAITING_FOR_SEGMENTATION → WAITING_FOR_PRESHAPING → IDLE |
| **Segmentation integration** | Watches `/segmentation/object_cloud`, calls `/grasp_preshaping/compute_grasp` service |
| **Activation** | Runtime via `/twist_propagation/activate` and `/twist_propagation/deactivate` services |
| **TF2 support** | Yes — transforms hand pose from arbitrary frame to cloud frame |
| **Missing** | No launch file of its own, no config YAML of its own, no `resource/` marker file |

### Key Differences Summary

| Feature | V1 (future_prediction) | V2 (twist_propagation) |
|---|---|---|
| Twist source | Odometry twist field | Self-estimated from poses |
| Covariance tracking | Yes (12×12) | No |
| Custom message | `FuturePoseTrajectory` | Standard msgs only |
| Collision radius | Configurable sphere | Point-only (no radius) |
| State machine | None | Full 3-state |
| Segmentation pipeline | None | Full (watch + preshaping call) |
| Runtime activation | Launch-time only | Services |
| TF2 frame handling | Assumes same frame | Full TF2 transforms |
| RViz path output | Yes (`Path` msg) | No |
| Integrated into pipeline | Standalone | Yes (4 launch files) |
| Package type | CMake | ament_python |

---

## Recommendation: Unify Into Version 2 (`twist_propagation`)

**Rationale:**

1. **V2 is already integrated** into 4 launch files (`pipeline`, `grasp_test`, `digital_twin`, `mock`). V1 is standalone.
2. **V2 has the full pipeline** — state machine, segmentation orchestration, preshaping service call. V1 just publishes a click.
3. **V2 has runtime activation** via services. V1 requires restart.
4. **V2 has TF2 support** for cross-frame transforms. V1 assumes odom and cloud are in the same frame.
5. **V2's self-estimated twist** is more general — works with any pose source, not just OpenVINS odometry.

**What V1 has that V2 needs:**

1. **Covariance propagation** — useful for uncertainty-aware collision thresholds
2. **Collision sphere radius** — V2 checks point-only; needs `collision_geometry_radius_m`
3. **Path visualization** — V1 publishes `nav_msgs/Path`; V2 publishes nothing visual
4. **Voxel downsampling** — V1 downsamples the cloud; V2 uses it raw
5. **Odometry staleness checks** — V1 has robust guard rails
6. **Config YAML file** — V1 has a dedicated config; V2 relies on inline params in `prosthesis_config.yaml`

---

## Implementation Plan

### Phase 1: Merge V1 Strengths Into V2 Node

- [x] **1.1 Add collision sphere radius parameter** to `twist_propagation_node.py`. Added `collision_geometry_radius_m` parameter (default 0.10). Modified `_propagate_and_find_hit` to use `effective_hit_thresh = hit_threshold_m + collision_radius`.

- [x] **1.2 Add voxel downsampling** to `twist_propagation_node.py`. Ported `_voxel_downsample` function and added `voxel_leaf_m` parameter (default 0.02). Applied in `_on_input_cloud` after parsing XYZ.

- [x] **1.3 Add path trajectory publishing** to `twist_propagation_node.py`. Collects predicted positions during propagation, publishes as `nav_msgs/Path` on `/twist_propagation/predicted_path`. Also publishes current pose as `PoseStamped` on `/twist_propagation/current_pose`.

- [x] **1.4 Add covariance propagation**. Ported `build_initial_covariance_from_odom`, `build_initial_covariance_from_pose_buf`, and `propagate_covariance` from V1. Added `max_prediction_covariance_trace` guard. Supports both Odometry (with real covariance) and PoseStamped (with estimated covariance from pose buffer scatter). Conditionally enabled via `enable_covariance_propagation` parameter.

- [x] **1.5 Add pose staleness checks**. Added `pose_max_age_s` parameter (default 1.0). Checks both cloud age and pose age before running propagation.

- [x] **1.6 Add `process_noise_linear_mps2_per_s` and `process_noise_angular_radps2_per_s` parameters**. Used by covariance propagation.

### Phase 2: Add RViz Visualization

- [x] **2.1 Created `rviz/twist_propagation.rviz`** with displays for:
  - Scene PointCloud (subscribes to input cloud)
  - Predicted Path (green line from `/twist_propagation/predicted_path`)
  - Current Hand Pose (arrow from `/twist_propagation/current_pose`)
  - Collision Spheres (MarkerArray from `/twist_propagation/collision_spheres`)
  - Click Positive (sphere from `/segmentation/click_positive`)
  - Hit Marker (MarkerArray from `/twist_propagation/hit_marker`)
  - Trajectory Line (MarkerArray from `/twist_propagation/trajectory_line`)

- [x] **2.2 Added collision sphere visualization**. Publishes `MarkerArray` on `/twist_propagation/collision_spheres` with semi-transparent red spheres at subsampled predicted positions. Spheres sized to `collision_radius * 2`.

- [x] **2.3 Added hit point visualization marker**. Publishes persistent green sphere (6cm, 2s lifetime) on `/twist_propagation/hit_marker` at the collision point.

- [x] **2.4 Added trajectory line marker**. Publishes `LINE_STRIP` on `/twist_propagation/trajectory_line`. Green when no hit, red when collision detected.

### Phase 3: Create Dedicated Launch + Config for V2

- [x] **3.1 Created `src/twist_propagation/launch/twist_propagation.launch.py`**. Standalone launch with arguments: `active`, `input_cloud_topic`, `hand_pose_topic`, `odom_topic`, `rviz`, `use_sim_time`.

- [x] **3.2 Created `src/twist_propagation/config/twist_propagation.yaml`**. All parameters with documented sections. `prosthesis_config.yaml` updated to include new params in both sections.

- [x] **3.3 Updated `setup.py`** to install `launch/` and `config/` directories. Created `resource/twist_propagation` marker file.

- [x] **3.4 Updated `package.xml`** with `nav_msgs`, `visualization_msgs`, `tf2_ros`, `tf2_geometry_msgs` dependencies.

### Phase 4: Remove V1 (Future Prediction Collision)

- [x] **4.1 Removed**: `src/sensor_fusion_bringup/scripts/future_pose_prediction_collision_node.py`
- [x] **4.2 Removed**: `src/sensor_fusion_bringup/launch/future_prediction_collision.launch.py`
- [x] **4.3 Removed**: `src/sensor_fusion_bringup/config/future_prediction_collision.yaml`
- [x] **4.4 Removed**: `src/sensor_fusion_bringup/test/test_future_prediction_collision.py`
- [x] **4.5 Updated `CMakeLists.txt`**: Cleaned V1 references.
- [x] **4.6 Removed `sensor_fusion_msgs` package entirely**. No other consumers found. V2 uses standard `nav_msgs/Path` instead.
- [x] **4.7 Removed `docs/future_prediction_collision_validation.md`**.

### Phase 5: Update Tests

- [x] **5.1 Ported V1 unit tests** to `src/twist_propagation/test/test_twist_propagation.py`. 392 lines covering: propagation (stationary, linear, rotational), covariance (growth, symmetry, initial build from odom, initial build from pose buf), proximity (with/without collision radius), odometry guards (uninitialized, initialized, NaN, Inf), voxel downsampling (empty, single, merge, passthrough, order), point cloud parsing (basic, empty, NaN filter), quaternion multiply.

- [x] **5.2 Added new unit tests** for collision sphere radius behavior (hit_near_no_radius, hit_with_collision_radius, no_hit_far).

- [x] **5.3 Updated integration test** at `scripts/test_twist_propagation_integration.py`. Added subscriptions for `Path`, `MarkerArray`, `Marker` (collision spheres, hit marker, trajectory line). Added `test_visualization_published` and `test_hit_marker_on_collision` test cases.

- [x] **5.4 Created visual smoke test** at `scripts/visual_smoke_test_twist_propagation.py`. Publishes mock hand poses moving toward a point cloud cluster. Usage: launch node with `active:=true`, then run the script.

### Phase 6: Update Pipeline Launch Files

- [x] **6.1-6.4 Verified**: All 4 pipeline launch files (`pipeline.launch.py`, `grasp_test.launch.py`, `digital_twin.launch.py`, `mock.launch.py`) already use `twist_propagation` and pass either `config_file` or inline params. The node declares all new parameters with defaults, so no launch file changes were needed.
---

## Verification Criteria

- [x] `twist_propagation_node` compiles and launches without errors
- [x] All unit tests ported + new tests added (`src/twist_propagation/test/test_twist_propagation.py`)
- [x] Integration test updated with visualization tests (`scripts/test_twist_propagation_integration.py`)
- [x] RViz config created with: predicted trajectory path, collision spheres, hit point marker, trajectory line, input pointcloud
- [x] Collision detection works with configurable sphere radius (`collision_geometry_radius_m`)
- [x] State machine transitions preserved: IDLE → WAITING_FOR_SEGMENTATION → WAITING_FOR_PRESHAPING → IDLE
- [x] Runtime activation/deactivation via services preserved
- [x] V1 node and all its files fully removed
- [x] No remaining references to `future_pose_prediction_collision` in code (only in docs/chat-logs)
- [x] `sensor_fusion_msgs` package removed (no other consumers)

---

## Potential Risks and Mitigations

1. **Breaking existing pipeline integration**
   Mitigation: V2 is already used in 4 launch files. The merge adds features to V2 without changing its existing interface (topics, services, state machine). All existing parameters keep their defaults.

2. **Twist estimation quality at distance**
   The user reports V1 "only works when very close." This is likely because the raw odometry twist is small at low speeds and the collision geometry is too small. V2's self-estimated twist with least-squares fitting + EMA smoothing should be more robust. The added collision sphere radius parameter also allows larger detection envelopes. Mitigation: Tune `collision_geometry_radius_m` (e.g., 0.15-0.20m) and `hit_threshold_m` (e.g., 0.10m) for earlier detection.

3. **Missing `resource/` marker file in V2 package**
   `setup.py:12` references `resource/twist_propagation` but the file doesn't exist. This will cause `colcon build` to fail. Mitigation: Create the file as part of Phase 3.

4. **Duplicate config sections in `prosthesis_config.yaml`**
   Two identical `twist_propagation:` sections exist (lines 96-112 and 231-245). The second one will override the first. Mitigation: Consolidate into one section and move to a dedicated config file.

5. **`sensor_fusion_msgs` dependency chain**
   If we keep `FuturePoseTrajectory.msg` output in V2, we add a build-time dependency from `twist_propagation` (ament_python) to `sensor_fusion_msgs` (ament_cmake). Mitigation: Consider dropping the custom message and using standard `nav_msgs/Path` + separate covariance topic instead. This simplifies the dependency graph.

---

## Alternative Approaches

1. **Keep both nodes, make V1 a "library"**: Extract the pure math functions from V1 into a shared Python module that V2 imports. This avoids merging but adds a cross-package dependency. **Trade-off**: Less cleanup needed but two packages to maintain for the same functionality.

2. **Rewrite in C++**: Both nodes are Python. A C++ implementation would be faster for the propagation loop and KDTree queries. **Trade-off**: Significant effort, harder to iterate on, overkill for 8 Hz processing.

3. **Drop covariance entirely**: Skip Phase 1.4 (covariance propagation). The covariance is not used for any downstream decision — only the trace-based horizon cutoff matters, and a simpler time-based horizon achieves the same goal. **Trade-off**: Less rigorous but simpler. Recommended if time is constrained.

4. **Use `sensor_fusion_msgs/FuturePoseTrajectory` as V2's trajectory output**: Instead of just `nav_msgs/Path`, publish the custom message with covariance diagonal. **Trade-off**: Keeps the custom message package alive but adds a build dependency.
