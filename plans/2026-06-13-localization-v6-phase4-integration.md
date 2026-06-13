# Localization Rework V6 — Phase 4: Integration (Twist, Launch, Config, Mock)

**Date:** 2026-06-13
**Parent plan:** `plans/2026-06-13-localization-rework-plan-v6.md`
**Depends on:** Phase 0, Phase 1, Phase 2, Phase 3 (all complete)
**Blocks:** Nothing (this is the final phase)
**Estimated time:** 2–3 days
**Parallelism:** 1 agent (serial — this phase wires everything together)

---

## Objective

Wire all the new components into the existing pipeline:
- **A:** Twist propagation TSDF service integration (~60 lines)
- **B:** GTSAM trajectory binder ROS node (wraps Phase 1 Task C factor graph)
- **C:** Launch file updates (`pipeline.launch.py` + `mock.launch.py`)
- **D:** Configuration additions (`prosthesis_config.yaml`)
- **E:** End-to-end integration testing on mock + rosbag replay

This phase has no new pure-logic components — it's all integration plumbing and validation.

---

## Task A: Twist Propagation TSDF Service Integration

**Agent:** 1 (~0.5 day)
**Depends on:** Phase 3 Task A (`tsdf_fusion` node with `TriggerGraspFusion` service)
**File:** `src/twist_propagation/twist_propagation/twist_propagation_node.py` (1782 lines)

### Current flow

The twist propagation node currently:
1. Estimates hand twist from `/hand_pose`.
2. Propagates forward in time against the live point cloud.
3. On collision: publishes a `PointStamped` to `/segmentation/click_positive`.
4. Transitions to `WAITING_FOR_SEGMENTATION` state.
5. Waits for `/segmentation/object_cloud` (published by `segmentation_bridge`).
6. On segmented cloud arrival: returns to `IDLE` (pipeline manager handles preshaping).

See `twist_propagation_node.py:1433-1456` (WAITING_FOR_SEGMENTATION state) and `twist_propagation_node.py:1700-1736` (click publish + state transition).

### V6 flow (additive — behind a parameter)

Add a `use_tsdf_fusion` parameter (default `false`). When `true`, instead of publishing a click to the segmentation bridge, the node calls the `TriggerGraspFusion` service:

- [ ] **A.1** Add parameters (V6 §6.9):
  ```python
  self.declare_parameter("use_tsdf_fusion", False)
  self.declare_parameter("tsdf_fusion_service", "/tsdf_fusion/trigger")
  self.declare_parameter("max_head_wrist_distance_m", 1.0)
  self.declare_parameter("enforce_kinematic_constraint", False)
  ```

- [ ] **A.2** Add the kinematic distance check (V6 §6.1, ~30 lines):
  Before triggering segmentation, verify the hand/wrist is within `max_head_wrist_distance_m` of the head camera. If `enforce_kinematic_constraint` is true and the distance exceeds the limit, skip the trigger (the object is out of reach). This prevents futile segmentation on unreachable targets.
  - Look up head pose and wrist pose via TF (or from the pose buffer).
  - Compute Euclidean distance.
  - Log a warning if the constraint is violated.

- [ ] **A.3** Add the TSDF service client:
  ```python
  if self._use_tsdf_fusion:
      client = self.create_client(TriggerGraspFusion, self._tsdf_service)
      request = TriggerGraspFusion.Request()
      request.hit_point = hit_point_msg.point
      request.camera_id = "arm"  # or determine from TF
      request.roi_radius = self._roi_radius
      future = client.call_async(request)
      # Handle response in a callback — the segmented cloud is published
      # by the tsdf_fusion node to /segmentation/object_cloud, which the
      # existing _on_segmented_cloud callback already watches.
  else:
      # Existing path: publish click_positive
      self._click_pub.publish(click)
  ```
  The key insight: **the WAITING_FOR_SEGMENTATION state machine doesn't change.** The tsdf_fusion node publishes to the same `/segmentation/object_cloud` topic that the existing `_on_segmented_cloud` callback (`twist_propagation_node.py:837-846`) already monitors. So the only change is *how* the segmentation is triggered (service call vs. click publish), not *how* the result is consumed.

- [ ] **A.4** Verify the state transition still works:
  - After the service call (or click publish), set `self._cycle_state = CycleState.WAITING_FOR_SEGMENTATION` (already done at `twist_propagation_node.py:1736`).
  - The `_on_segmented_cloud` callback fires when `/segmentation/object_cloud` arrives.
  - The `_cycle_callback` detects the new cloud (`twist_propagation_node.py:1436`) and returns to IDLE.

### A.5 Tests

- [ ] **Unit test:** With `use_tsdf_fusion=False` (default), the existing click-publish path is unchanged — verify no regression.
- [ ] **Integration test:** With `use_tsdf_fusion=True`, mock the TriggerGraspFusion service, verify the node calls it instead of publishing a click, and transitions to WAITING_FOR_SEGMENTATION correctly.

---

## Task B: GTSAM Trajectory Binder ROS Node

**Agent:** 1 (~1 day)
**Depends on:** Phase 1 Task A (`sensor_fusion_msgs`), Phase 1 Task C (`factor_graph`, `se3_helpers`)
**File:** `src/gtsam_tracker/gtsam_tracker/gtsam_tracker_node.py` (NEW)

### B.1 Node structure

The package skeleton was created in Phase 1 Task C. Now add the ROS node:

```
src/gtsam_tracker/
├── gtsam_tracker/
│   ├── __init__.py
│   ├── se3_helpers.py           # DONE in Phase 1
│   ├── factor_graph.py          # DONE in Phase 1
│   ├── umeyama.py               # DONE in Phase 1
│   └── gtsam_tracker_node.py    # NEW: ROS wrapper (~400 lines)
├── test/
│   ├── test_se3_helpers.py      # DONE in Phase 1
│   ├── test_factor_graph.py     # DONE in Phase 1
│   └── test_gtsam_node.py       # NEW: ROS smoke test
├── resource/
│   └── gtsam_tracker
├── setup.py                     # UPDATE: add node entry point
└── package.xml                  # UPDATE: add deps
```

### B.2 Node implementation

Implement `gtsam_tracker_node.py` following V6 plan §6.3:

- [ ] **Parameters** (V6 §6.9):
  - `head_odom_topic: "/ov_msckf/odomimu"`
  - `arm_odom_topic: "/ov_msckf_arm/odomimu"`
  - `head_pose_source: "odom"` (V6 NEW — "odom" or "tf")
  - ArUco topics: `aruco_marker_topic`, `aruco_dynamic_topic`, `aruco_arm_pose_topic`
  - `graph_rate_hz: 15.0`, `smoother_lag_s: 15.0`
  - `kinematic_range_m: 1.0`, `kinematic_range_sigma_m: 0.05`
  - Noise sigmas: `odom_translation_sigma_m`, `odom_rotation_sigma_rad`

- [ ] **Odometry subscriptions:**
  - Subscribe to both odom topics (decimate to `graph_rate_hz`).
  - On each odom message: compute the delta from the previous pose, add a `BetweenFactorPose3` using `factor_graph.add_odometry_factor()`.
  - Extract noise model from the odom covariance using `factor_graph.noise_from_covariance_diag()`.

- [ ] **Head pose TF fallback (V6 §6.3):**
  If `head_pose_source == "tf"`: use a TF listener to look up `marker_map → head_imu` at the graph rate. Compute deltas and add between-factors. The bags confirm this transform is published dynamically.
  ```python
  if self._head_pose_source == "tf":
      self._tf_buffer = Buffer()
      self._tf_listener = TransformListener(self._tf_buffer, self)
  ```

- [ ] **ArUco subscriptions (graceful degradation — V6 §9):**
  - Subscribe to the three ArUco topics.
  - On `MarkerPoseObservation`: add a `PriorFactorPose3` via `factor_graph.add_aruco_prior()`.
  - On `DynamicMarkerObservation`: add a `BetweenFactorPose3` via `factor_graph.add_visual_between()`.
  - **If no ArUco messages arrive (topics absent on live system):** the graph runs on odometry + visual factors only. Log a one-time warning. Do NOT crash or block.

- [ ] **Visual factor subscription:**
  - Subscribe to `/vis/head_arm_pose` (from `cross_camera_features` node).
  - Add a between-factor between the head and arm chains.

- [ ] **Graph update loop:**
  - Timer at `graph_rate_hz`: call `factor_graph.update()` (runs the FixedLagSmoother).
  - Query current head and arm poses: `factor_graph.get_pose()`.
  - Publish to `/gtsam/head_pose` and `/gtsam/arm_pose` as `geometry_msgs/PoseWithCovarianceStamped`.

- [ ] **Kinematic range factor (V6 §6.3):**
  Every N updates, check head-arm distance. If > `kinematic_range_m`, add a soft range factor pulling them together.

### B.3 Tests

- [ ] **ROS smoke test:** Launch the node with mock odom publishers. Verify it publishes `/gtsam/head_pose` and `/gtsam/arm_pose`.
- [ ] **TF fallback test:** With `head_pose_source="tf"` and a mock TF publisher, verify the node produces head poses.
- [ ] **Graceful degradation test:** Launch without ArUco topics — verify the node runs without error and logs a warning.

---

## Task C: Launch File Updates

**Agent:** 1 (~0.5 day)
**Files:** `src/prosthesis_launch/launch/pipeline.launch.py`, `src/prosthesis_launch/launch/mock.launch.py`

### C.1 Pipeline launch

Add the new nodes to `pipeline.launch.py` (currently 507 lines). Follow the existing pattern of `_node_params(config, node_name)` to pull parameters from `prosthesis_config.yaml`.

- [ ] **Add gtsam_tracker node** (conditional on `camera` being true, since it needs odom):
  ```python
  if _as_bool(context, "camera"):
      nodes.append(Node(
          package="gtsam_tracker",
          executable="gtsam_tracker_node",
          name="gtsam_tracker",
          parameters=[_node_params(config, "gtsam_tracker")],
          output="screen",
      ))
  ```

- [ ] **Add keyframe_buffer node** (conditional on `camera`):
  ```python
  nodes.append(Node(
      package="keyframe_buffer",
      executable="keyframe_buffer_node",
      name="keyframe_buffer",
      parameters=[_node_params(config, "keyframe_buffer")],
      output="screen",
  ))
  ```

- [ ] **Add cross_camera_features node** (conditional on `camera`):
  ```python
  nodes.append(Node(
      package="cross_camera_features",
      executable="sift_feature_node",
      name="cross_camera_features",
      parameters=[_node_params(config, "cross_camera_features")],
      output="screen",
  ))
  ```

- [ ] **Add tsdf_fusion node** (conditional on a new `use_tsdf_fusion` launch arg):
  Add a `DeclareLaunchArgument("use_tsdf_fusion", default_value="false")`. When true, launch the tsdf_fusion node. When false, keep the existing `segmentation_bridge` (the old InterObject3D path). This allows A/B testing.

- [ ] **Update the segmentation bridge** to be conditional:
  Currently `pipeline.launch.py:318-327` unconditionally launches `segmentation_bridge`. Make it conditional on `use_tsdf_fusion` being false (so the new TSDF path replaces it when enabled).

### C.2 Mock launch

Update `mock.launch.py` to add mock publishers for the new nodes:

- [ ] **Mock odom publishers:** Publish fake `/ov_msckf/odomimu` and `/ov_msckf_arm/odomimu` with a slowly drifting trajectory. This lets the gtsam_tracker be tested in mock mode.
- [ ] **Mock image + cloud publishers:** The mock launch already has some — verify they publish to the topics the new nodes expect (`/head/d435i_head/color/image_raw`, etc.).
- [ ] **Mock GTSAM pose publishers:** If the gtsam_tracker isn't ready for mock testing, publish fake `/gtsam/head_pose` and `/gtsam/arm_pose` directly so the keyframe_buffer can be tested.
- [ ] **Add the new nodes to mock launch** (same as pipeline launch, but with mock-friendly parameters).

---

## Task D: Configuration

**Agent:** 1 (~0.5 day)
**File:** `config/prosthesis_config.yaml`

- [ ] **Add all new sections** from V6 plan §6.9:
  - `gtsam_tracker` (with `head_pose_source: "odom"` default)
  - `cross_camera_features` (with `process_rate_hz: 5.0`)
  - `keyframe_buffer` (with `expect_organized_clouds: false`)
  - `tsdf_fusion`
  - Update `twist_propagation` section with the new params (`use_tsdf_fusion`, `tsdf_fusion_service`, `max_head_wrist_distance_m`, `enforce_kinematic_constraint`)

- [ ] **Verify the config loads correctly** in the container:
  ```bash
  python3 -c "import yaml; yaml.safe_load(open('config/prosthesis_config.yaml'))"
  ```

- [ ] **Verify `_node_params()` extracts each section correctly** (the function at `pipeline.launch.py:56-68` expects `ros__parameters` sub-keys).

---

## Task E: End-to-End Integration Testing

**Agent:** 1 (~1 day)
**Depends on:** All tasks A–D complete

### E.1 Mock pipeline integration test

- [ ] Launch the full mock pipeline with the new nodes:
  ```bash
  make shell
  ros2 launch prosthesis_launch mock.launch.py use_tsdf_fusion:=true
  ```
  Verify:
  - All new nodes start without error.
  - gtsam_tracker publishes `/gtsam/head_pose` and `/gtsam/arm_pose`.
  - keyframe_buffer logs cloud organization detection.
  - cross_camera_features runs (may log "no matches" with synthetic data — acceptable).
  - tsdf_fusion node starts and health-checks the SAM server.

### E.2 Rosbag replay test

- [ ] Replay Bag 2 (`rosbag2_2026_05_21-16_48_20` — has images + clouds + camera_info) against the keyframe_buffer and cross_camera_features nodes:
  ```bash
  ros2 bag play rosbags/rosbag2_2026_05_21-16_48_20 --clock
  ```
  Verify:
  - keyframe_buffer detects `height=1` (UNORGANIZED) and logs it.
  - keyframe_buffer stores keyframes as the bag plays.
  - cross_camera_features receives synced image pairs.
  - Memory stays bounded (check diagnostics topic).

- [ ] Replay Bag 1 (`rosbag2_2026_05_21-16_37_45` — has arm odom) against the gtsam_tracker:
  ```bash
  ros2 bag play rosbags/rosbag2_2026_05_21-16_37_45 --clock
  ```
  Verify:
  - gtsam_tracker receives `/ov_msckf_arm/odomimu` and builds the arm chain.
  - `/gtsam/arm_pose` is published.
  - Memory is stable over the 65s bag (marginalization works).

### E.3 TSDF fusion trigger test (mock)

- [ ] With the mock pipeline running and `use_tsdf_fusion:=true`:
  - Manually call the `TriggerGraspFusion` service with a synthetic hit point.
  - Verify the fused cloud is published to `/segmentation/object_cloud`.
  - Verify the twist_propagation node transitions through WAITING_FOR_SEGMENTATION → IDLE.

### E.4 Validation gate checklist

Run through the V6 plan §6.10 validation gates that are testable offline:

| Gate | Testable in Phase 4? | Method |
|------|---------------------|--------|
| Keyframe buffer latency < 5ms | Yes | Mock pipeline timing |
| Keyframe buffer memory < 350 MB | Yes | Diagnostics topic during bag replay |
| SAM latency < 300ms burst | Yes | Mock SAM server timing |
| SAM cold start < 5s | Yes | Container startup timing |
| GTSAM marginalization (memory stable) | Yes | Bag 1 replay (arm odom only) |
| Cloud organization detection | Yes | Bag 2 replay |
| TSDF quality < 2cm RMS | **No** — needs full topic set | Synthetic scene test (Phase 3) covers logic |
| End-to-end hit-to-cloud < 500ms | **Partial** — mock only | Mock pipeline timing |
| ArUco topics visible | **No** — needs live session | Deferred to hardware |
| Head odom visible | **No** — needs live session | TF fallback tested instead |

---

## Verification Gate

Phase 4 is complete when ALL of the following are true:
1. `twist_propagation` node supports `use_tsdf_fusion` parameter (default false, no regression).
2. `gtsam_tracker` node builds, launches, and publishes poses in mock mode.
3. `pipeline.launch.py` launches all new nodes with `use_tsdf_fusion:=true`.
4. `mock.launch.py` exercises the new nodes with synthetic data.
5. `prosthesis_config.yaml` contains all new sections and loads without error.
6. Bag 2 replay confirms keyframe_buffer detects unorganized clouds and stores keyframes.
7. Bag 1 replay confirms gtsam_tracker processes arm odometry and publishes stable poses.
8. Mock TSDF trigger test produces a cloud on `/segmentation/object_cloud`.
9. All existing tests still pass (`make test`).
10. All new unit tests pass.

---

## Dependency Graph

```
Phase 3 Task A (tsdf_fusion) ──► Task A (twist integration)
Phase 1 Task C (factor_graph) ──► Task B (gtsam_tracker node)
All phases ────────────────────► Task C (launch files)
All phases ────────────────────► Task D (config)
Tasks A+B+C+D ─────────────────► Task E (integration testing)
```

Tasks A, B, C, D can partially overlap (C and D are quick once the node names are known). Task E is strictly last.

---

## Risk Notes

- **Launch file complexity:** `pipeline.launch.py` is already 507 lines with many conditionals. Adding 4 more nodes increases complexity. Keep the new nodes behind clear conditionals (`camera`, `use_tsdf_fusion`) so the default launch behavior is unchanged.
- **Parameter namespace collisions:** Ensure the new config sections don't collide with existing ones. The `_node_params()` function keys on the top-level section name, so `gtsam_tracker`, `keyframe_buffer`, etc. must be unique.
- **Mock data fidelity:** The mock publishers must produce data realistic enough to exercise the new nodes. If the mock odom trajectory is too simple (e.g., static), the gtsam_tracker may not produce meaningful output. Use a slowly-moving synthetic trajectory.
- **The biggest remaining risk is live-data validation.** Phases 0–4 prove the code is correct in isolation and in mock mode. The final step — running on real hardware with live ArUco + head odom — cannot be done without the hardware. The fresh comprehensive bag (V6 §12) is the bridge: record it on the hardware, then replay it through the full pipeline as the final pre-deployment test.
- **Backward compatibility:** The `use_tsdf_fusion` parameter defaults to `false`. This means deploying Phase 4 does NOT change the existing pipeline behavior until the operator explicitly enables TSDF fusion. This is intentional — it allows safe rollout and A/B comparison.