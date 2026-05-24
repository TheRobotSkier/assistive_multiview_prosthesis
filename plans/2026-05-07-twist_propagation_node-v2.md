# Twist Propagation Target Selector Node

## Objective

Build a Python ROS 2 node that continuously estimates the hand's twist (velocity), propagates it forward in time against the live point cloud, and when a propagated hand position intersects a point in the cloud within a configurable distance threshold, triggers the segmentation + preshaping pipeline. The node must be activatable/deactivatable on demand and run on a continuous cycle with a configurable delay.

Additionally, fix the pre-existing topic name mismatch between the segmentation node's output and the preshaping bridge's input, and ensure preshaping is only called after a segmented cloud is available.

## System Analysis

### Verified Topic Map

| Topic | Type | Published By | Consumed By |
|-------|------|-------------|-------------|
| `/hand_pose` | `geometry_msgs/msg/Pose` | **External** (hand tracking / TF) | preshaping_service_bridge_node, grasp_proximity_controller_node, **twist_propagation_node** |
| `/hand_twist` | `geometry_msgs/msg/TwistWithCovarianceStamped` | **twist_propagation_node** (new) | preshaping_service_bridge_node |
| `/camera/depth/color/points` | `sensor_msgs/msg/PointCloud2` | RealSense D435 or mock_cloud_publisher | **twist_propagation_node** |
| `/segmentation/input_cloud` | `sensor_msgs/msg/PointCloud2` | **Not yet connected** | segmentation_ros2_node |
| `/segmentation/click_positive` | `geometry_msgs/msg/PointStamped` | **twist_propagation_node** (new) | segmentation_ros2_node |
| `/segmentation/object_cloud` | `sensor_msgs/msg/PointCloud2` | segmentation_ros2_node | preshaping_service_bridge_node (**after fix**) |
| `/grasp_preshaping/compute_grasp` | `std_srvs/srv/Trigger` | preshaping_service_bridge_node | pipeline_manager_node, **twist_propagation_node** |
| `/pipeline/state` | `std_msgs/msg/Int32` | pipeline_manager_node | force_controller_node |

### Critical Findings

1. **No existing `/hand_pose` publisher found in the codebase.** The preshaping_service_bridge_node (`src/grasp_preshaping/nodes/preshaping_service_bridge_node.cpp:101-107`) subscribes to `/hand_pose` and `/hand_twist`, but no node in the repo publishes them. These are expected to come from an external hand tracking system (e.g., OptiTrack, ROS tf from a URDF). The new node must also subscribe to `/hand_pose` and will **publish** `/hand_twist`.

2. **Topic name mismatch (BUG — to be fixed):** The segmentation node publishes to `/segmentation/object_cloud` (`src/segmentation/segmentation_bridge/segmentation_ros2_node.py:137`), but the preshaping bridge subscribes to `/segmented_object_cloud` (`src/grasp_preshaping/nodes/preshaping_service_bridge_node.cpp:83`). The config file also uses the wrong name (`config/prosthesis_config.yaml:16`), and the RViz config references it (`rviz/prosthesis.rviz:53`). **All three must be updated to `/segmentation/object_cloud`.**

3. **Preshaping must wait for segmented cloud.** The preshaping service bridge (`preshaping_service_bridge_node.cpp:236-261`) checks `has_cloud_` before computing. The new node must subscribe to the segmented cloud output and only call the preshaping service after receiving a new segmented cloud — not immediately after triggering segmentation.

4. **Point cloud source for propagation**: The node should subscribe to `/camera/depth/color/points` (the raw scene cloud) for twist propagation intersection checking — not the segmented cloud, which doesn't exist yet at this stage of the pipeline.

5. **The segmentation node is click-driven** (`src/segmentation/segmentation_bridge/segmentation_ros2_node.py:178`). It only runs inference when a new positive/negative click arrives. The new node must publish a `PointStamped` to `/segmentation/click_positive` to trigger segmentation.

6. **The preshaping service is a Trigger service** (`src/grasp_preshaping/nodes/preshaping_service_bridge_node.cpp:148-154`). It uses whatever latest pose, twist, and cloud it has buffered. The new node must ensure the segmented cloud has been received by the preshaping bridge before calling the service.

### Package Placement Decision

**Recommendation: Create a new package `src/twist_propagation/`** rather than adding to an existing one.

Rationale:
- The `pipeline_manager` package is specifically for the state machine orchestrator
- The `grasp_preshaping` package is a CMake (C++) package; adding a Python node requires extra wiring
- The `segmentation_bridge` package is purely for the inference bridge
- This node has a distinct responsibility (twist estimation + propagation + target selection) that doesn't fit neatly into any existing package
- Following the existing pattern of single-responsibility Python packages (like `force_controller`, `wrist_driver`, `emg_bridge`)

## Implementation Plan

### Phase 0: Fix Segmentation Topic Name Mismatch

- [ ] **0.1** Update `src/grasp_preshaping/nodes/preshaping_service_bridge_node.cpp:83` — change the default value of the `cloud_topic` parameter from `"/segmented_object_cloud"` to `"/segmentation/object_cloud"` to match the actual topic published by `segmentation_ros2_node.py:137`.

- [ ] **0.2** Update `config/prosthesis_config.yaml:16` — change `segmented_object_cloud: "/segmented_object_cloud"` to `segmented_object_cloud: "/segmentation/object_cloud"`. This aligns the config entry with the actual topic name.

- [ ] **0.3** Update `rviz/prosthesis.rviz:53` — change the topic value from `/segmented_object_cloud` to `/segmentation/object_cloud` so the RViz display subscribes to the correct topic.

### Phase 1: Package Scaffolding

- [ ] **1.1** Create `src/twist_propagation/` package directory with `ament_python` build type, following the pattern of `src/force_controller/`:
  - `src/twist_propagation/package.xml` — depend on `rclpy`, `geometry_msgs`, `sensor_msgs`, `std_msgs`, `std_srvs`, `tf2_ros`, `tf2_geometry_msgs`
  - `src/twist_propagation/setup.py` — with entry point `twist_propagation_node = twist_propagation.twist_propagation_node:main`
  - `src/twist_propagation/setup.cfg` — with `install_scripts=$base/lib/twist_propagation`
  - `src/twist_propagation/resource/twist_propagation` — empty marker file
  - `src/twist_propagation/twist_propagation/__init__.py` — empty
  - `src/twist_propagation/twist_propagation/twist_propagation_node.py` — the main node

### Phase 2: Node Core Logic

- [ ] **2.1** Create the `TwistPropagationNode` class extending `rclpy.node.Node` with node name `twist_propagation`.

- [ ] **2.2** Declare all ROS parameters:
  - `cycle_delay_s` (float, default=0.1) — delay between propagation cycles
  - `propagation_time_horizon_s` (float, default=2.0) — max time to propagate twist forward
  - `propagation_dt_s` (float, default=0.02) — time step for propagation
  - `hit_threshold_m` (float, default=0.05) — distance threshold for a point "hit" in the cloud
  - `min_points_near_hit` (int, default=3) — minimum points within threshold to count as a valid hit
  - `twist_estimation_window` (int, default=5) — number of poses to use for twist estimation via finite differences
  - `active` (bool, default=false) — initial activation state
  - `input_cloud_topic` (string, default="/camera/depth/color/points") — raw scene cloud for propagation
  - `segmented_cloud_topic` (string, default="/segmentation/object_cloud") — segmented cloud to watch before calling preshaping
  - `hand_pose_topic` (string, default="/hand_pose") — current hand pose
  - `click_positive_topic` (string, default="/segmentation/click_positive") — segmentation trigger
  - `compute_grasp_service` (string, default="/grasp_preshaping/compute_grasp") — preshaping service
  - `activate_service` (string, default="/twist_propagation/activate") — service to activate
  - `deactivate_service` (string, default="/twist_propagation/deactivate") — service to deactivate
  - `segmentation_timeout_s` (float, default=5.0) — max wait for segmented cloud after triggering segmentation

- [ ] **2.3** Implement subscriptions:
  - Subscribe to `/hand_pose` (`geometry_msgs/msg/Pose`) — buffer recent poses with timestamps for twist estimation
  - Subscribe to the input cloud topic (`sensor_msgs/msg/PointCloud2`) — store the latest cloud as a NumPy XYZ array for propagation checks
  - Subscribe to the segmented cloud topic (`sensor_msgs/msg/PointCloud2`) — track when a new segmented cloud arrives after triggering segmentation

- [ ] **2.4** Implement TF2 listener to transform hand pose into the cloud frame before propagation checks. The hand pose may be in a different frame (e.g., `world` or `hand_link`) than the cloud (e.g., `camera_front_depth_optical_frame`). Use `tf2_ros.Buffer` + `tf2_ros.TransformListener` to look up the transform at the cloud's timestamp.

- [ ] **2.5** Implement twist estimation from pose history:
  - Maintain a rolling buffer of the last N poses with their receive timestamps (note: `/hand_pose` is `geometry_msgs/Pose`, not `PoseStamped`, so timestamps must be recorded at receive time using `get_clock().now()`)
  - Compute the twist as finite differences between the most recent two poses: linear velocity = (p1 - p0) / dt, angular velocity estimated from quaternion difference
  - Optionally use a larger window with least-squares fitting for smoother estimation
  - Publish the estimated twist to `/hand_twist` as `geometry_msgs/msg/TwistWithCovarianceStamped` so the preshaping bridge can use it

- [ ] **2.6** Implement twist propagation and cloud intersection:
  - Starting from the current hand pose, apply the estimated twist in small time steps (`propagation_dt_s`)
  - At each step, compute the propagated position: p(t+dt) = p(t) + v*dt (linear), q(t+dt) = q(t) * delta_q(omega*dt) (angular)
  - For each propagated position, check if any point in the stored point cloud is within `hit_threshold_m` of the propagated hand position (using a KD-tree from `scipy.spatial.KDTree` or a brute-force distance check for small clouds)
  - If a hit is found (>= `min_points_near_hit` points within threshold), record the 3D position and stop propagation

- [ ] **2.7** Implement the continuous cycle timer with segmentation-then-preshaping sequencing:
  - Use `create_timer(cycle_delay_s, self._cycle_callback)` for the main loop
  - The cycle has an internal state machine with states: `IDLE`, `WAITING_FOR_SEGMENTATION`, `WAITING_FOR_PRESHAPING`
  - **IDLE state** (active but waiting for a hit):
    1. Check if node is active; if not, skip
    2. Verify we have a recent hand pose and cloud
    3. Estimate twist from pose history
    4. Transform hand pose into cloud frame via TF2
    5. Propagate twist forward until hit or time horizon exceeded
    6. If hit found: publish the hit point to `/segmentation/click_positive` as `geometry_msgs/PointStamped`, transition to `WAITING_FOR_SEGMENTATION`, start a timeout timer
    7. If no hit within horizon: do nothing (log at debug level)
  - **WAITING_FOR_SEGMENTATION state**:
    - The segmented cloud subscription callback detects a new cloud (by checking header stamp changes)
    - When a new segmented cloud arrives: call `/grasp_preshaping/compute_grasp` service, transition to `WAITING_FOR_PRESHAPING`
    - If timeout expires without segmented cloud: log warning, transition back to `IDLE`
  - **WAITING_FOR_PRESHAPING state**:
    - Wait for the preshaping service response
    - On success or failure: transition back to `IDLE`

- [ ] **2.8** Implement activation/deactivation:
  - Create two `std_srvs/srv/Trigger` services: `/twist_propagation/activate` and `/twist_propagation/deactivate`
  - The activate service sets the internal `active` flag to True, resets internal state to `IDLE`, and clears the pose buffer
  - The deactivate service sets the flag to False, clears all state, and cancels any pending segmentation/preshaping wait
  - Log state changes at INFO level

- [ ] **2.9** Implement the `/hand_twist` publisher:
  - Create a publisher for `geometry_msgs/msg/TwistWithCovarianceStamped` on `/hand_twist`
  - Publish the estimated twist each cycle so that downstream nodes (preshaping_service_bridge_node) can use it
  - Set the covariance to fixed diagonal values matching the Rust config: `fixed_cov_omega: [0.001, 0.001, 0.001]`, `fixed_cov_v: [0.0005, 0.0005, 0.0005]` (from `config/grasp_preshaping.yaml:27-28`)

- [ ] **2.10** Implement status publisher:
  - Publish a `std_msgs/msg/String` on `/twist_propagation/status` with current state (active/inactive, internal state machine state, last hit point, twist magnitude, etc.)
  - Useful for debugging and RViz monitoring

### Phase 3: Build Integration

- [ ] **3.1** Add `twist_propagation` to the launch files:
  - In `src/prosthesis_launch/launch/pipeline.launch.py`: add a `Node` entry for `twist_propagation_node` from the `twist_propagation` package
  - In `src/prosthesis_launch/launch/mock.launch.py`: add the same entry for mock mode
  - Add `<depend>twist_propagation</depend>` to `src/prosthesis_launch/package.xml`

- [ ] **3.2** Rebuild the Docker image: `docker compose build prosthesis` (or `make build-prosthesis`)

### Phase 4: Configuration

- [ ] **4.1** Add twist propagation parameters to `config/prosthesis_config.yaml`:
  - New section `twist_propagation:` with all the parameters listed in 2.2
  - Reference these in the launch file via parameter file loading

## Verification Criteria

- [ ] Node starts without errors: `ros2 run twist_propagation twist_propagation_node`
- [ ] Node subscribes to `/hand_pose`, the input cloud topic, and the segmented cloud topic
- [ ] Node publishes estimated twist to `/hand_twist` when receiving poses
- [ ] Activation via `ros2 service call /twist_propagation/activate std_srvs/srv/Trigger` enables the cycle
- [ ] Deactivation via `ros2 service call /twist_propagation/deactivate std_srvs/srv/Trigger` disables the cycle
- [ ] When active, with a hand pose moving toward a point in the cloud, the node publishes a `PointStamped` to `/segmentation/click_positive`
- [ ] **After publishing the click, the node waits for a new segmented cloud on `/segmentation/object_cloud` before calling the preshaping service** — it does NOT call preshaping immediately
- [ ] **If segmentation times out (no segmented cloud received), the node returns to IDLE without calling preshaping**
- [ ] When the segmented cloud arrives, the node calls `/grasp_preshaping/compute_grasp` service
- [ ] When no point is hit within the propagation horizon, the node does nothing (no segmentation trigger, no service call)
- [ ] The cycle runs at the configured rate and does not block ROS spinning
- [ ] TF2 transforms are used correctly to align hand pose and cloud frames
- [ ] **Topic name fix verified**: `ros2 topic info /segmentation/object_cloud` shows both the segmentation node publishing and the preshaping bridge subscribed

## Potential Risks and Mitigations

1. **No `/hand_pose` publisher in the system**
   - The node depends on an external hand tracking system publishing `/hand_pose`. Without it, the node will simply wait (logging warnings).
   - Mitigation: Add a clear startup warning if no poses are received within N seconds. For testing, the mock_cloud_publisher pattern can be extended to also publish mock hand poses.

2. **TF2 frame availability**
   - The hand pose and cloud may be in different frames. If the TF tree is incomplete, transforms will fail.
   - Mitigation: Fall back to assuming same frame if TF lookup fails, with a warning. Log the frame IDs at startup.

3. **Point cloud size and KD-tree performance**
   - Raw D435 clouds are ~300K points. Building a KD-tree every cycle could be expensive.
   - Mitigation: Only rebuild the KD-tree when the cloud updates (check header stamp), not every cycle. Consider downsampling or using a voxel grid filter. Use `scipy.spatial.KDTree` which is efficient for this scale.

4. **Twist estimation noise**
   - Finite-difference twist from noisy poses will be noisy, especially at low pose rates.
   - Mitigation: Use a rolling window of N poses with least-squares linear fit for velocity. Apply a simple low-pass filter or exponential moving average to the twist estimate.

5. **Segmentation latency or failure**
   - Segmentation inference may take time (HTTP call to the inference server) or fail entirely. The node must not hang.
   - Mitigation: The `segmentation_timeout_s` parameter guards against this. If the segmented cloud doesn't arrive within the timeout, the node returns to IDLE and logs a warning. The node does NOT block while waiting — it uses the timer callback to check for the segmented cloud arrival.

6. **Race condition: preshaping service called before preshaping bridge has the segmented cloud**
   - Even though the twist_propagation_node sees the segmented cloud, the preshaping bridge might not have received it yet.
   - Mitigation: Add a small configurable delay (e.g., 0.2s) after detecting the segmented cloud before calling the preshaping service. Alternatively, subscribe to the segmented cloud and verify the preshaping bridge has likely received it by checking message receipt timing.

7. **Topic name fix may break existing remappings**
   - If any launch file or script remaps the old topic name `/segmented_object_cloud`, those remappings will break.
   - Mitigation: Search the entire codebase for the old name (already done — only 3 locations found, all being updated).

## Alternative Approaches

1. **Place node in `pipeline_manager` package**: Could add it as another entry point in the existing `pipeline_manager` Python package. Simpler package management but violates single-responsibility principle. The pipeline_manager is meant to be the state machine orchestrator, not a twist estimator.

2. **Place node in `grasp_preshaping` package**: Could add the Python script to `src/grasp_preshaping/nodes/` like `grasp_proximity_controller_node.py`. However, `grasp_preshaping` is a CMake package which makes Python dependency management awkward. The proximity controller was placed there as a temporary measure.

3. **Use an action server instead of a timer-based cycle**: Instead of a continuous timer, use a ROS 2 action server that runs the propagation loop and returns when a target is found. This would give better control over cancellation but adds complexity and doesn't match the "continuous cycle" requirement.

4. **Estimate twist via TF2 lookup**: Instead of subscribing to `/hand_pose` and computing finite differences, look up the TF transform at two close timestamps. This is more robust but requires the hand pose to be published to TF, which is not currently the case.

5. **Fix the topic mismatch by renaming the segmentation output instead**: Change the segmentation node to publish `/segmented_object_cloud` instead of changing the preshaping bridge. This avoids touching C++ code but changes the segmentation node's established API. The chosen approach (changing the preshaping bridge default) is simpler because it's a one-line default parameter change and the actual topic is already established by the segmentation publisher.

## Data Flow Diagram

```
                         ┌──────────────────────┐
                         │  Hand Tracking System │ (external)
                         │  (publishes /hand_pose)│
                         └──────────┬───────────┘
                                    │ Pose
                                    ▼
┌──────────────┐    cloud    ┌─────────────────────────────────────────┐
│  RealSense   │────────────▶│                                         │
│  D435        │             │      twist_propagation_node             │
└──────────────┘             │                                         │
                             │  1. Buffer poses                        │
                             │  2. Estimate twist                      │
                             │  3. Publish /hand_twist                 │
                             │  4. Propagate twist → raw cloud         │
                             │  5. If hit:                             │
                             │     → publish /segmentation/click_positive│
                             │     → wait for segmented cloud          │
                             │     → call /grasp_preshaping/compute_grasp│
                             └──────┬──────────┬──────────┬────────────┘
                                    │          │          │
                    click_positive  │          │ cloud    │ compute_grasp
                                    ▼          │ (after   │ (after seg.
                          ┌──────────────┐     │  seg.)   │  cloud)
                          │ Segmentation │     │          │
                          │ Bridge Node  │     │          │
                          └──────┬───────┘     │          │
                                 │             │          │
                   /segmentation/object_cloud   │          ▼
                                 │             │  ┌─────────────────────┐
                                 └─────────────┘  │ Preshaping Service  │
                                  (after fix)     │ Bridge Node (C++)   │
                                                  └─────────────────────┘
```

## Files to Create/Modify

### New Files
| File | Purpose |
|------|---------|
| `src/twist_propagation/package.xml` | Package manifest |
| `src/twist_propagation/setup.py` | Build configuration |
| `src/twist_propagation/setup.cfg` | Install configuration |
| `src/twist_propagation/resource/twist_propagation` | Ament resource marker |
| `src/twist_propagation/twist_propagation/__init__.py` | Package init |
| `src/twist_propagation/twist_propagation/twist_propagation_node.py` | Main node implementation |

### Files to Modify
| File | Change | Lines |
|------|--------|-------|
| `src/grasp_preshaping/nodes/preshaping_service_bridge_node.cpp` | Change cloud_topic default from `"/segmented_object_cloud"` to `"/segmentation/object_cloud"` | Line 83 |
| `config/prosthesis_config.yaml` | Change `segmented_object_cloud` value from `"/segmented_object_cloud"` to `"/segmentation/object_cloud"` | Line 16 |
| `rviz/prosthesis.rviz` | Change topic from `/segmented_object_cloud` to `/segmentation/object_cloud` | Line 53 |
| `src/prosthesis_launch/launch/pipeline.launch.py` | Add twist_propagation node entry | After line 96 |
| `src/prosthesis_launch/launch/mock.launch.py` | Add twist_propagation node entry | After line 78 |
| `src/prosthesis_launch/package.xml` | Add `<depend>twist_propagation</depend>` | After line 14 |
| `config/prosthesis_config.yaml` | Add `twist_propagation:` parameter section | New section |
