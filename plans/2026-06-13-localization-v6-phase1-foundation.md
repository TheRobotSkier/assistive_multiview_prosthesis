# Localization Rework V6 — Phase 1: Foundation (Pure Logic + Messages)

**Date:** 2026-06-13
**Parent plan:** `plans/2026-06-13-localization-rework-plan-v6.md`
**Depends on:** Phase 0 (`plans/2026-06-13-localization-v6-phase0-environment.md`) must be DONE.
**Blocks:** Phase 2, Phase 3
**Estimated time:** 2–3 days
**Parallelism:** 3 agents (Tasks A, B, C are fully independent)

---

## Objective

Build the three foundational pieces that everything else depends on:
- **A:** `sensor_fusion_msgs` package (message definitions for ArUco observations)
- **B:** `cloud_utils` pure-numpy library (unorganized/organized cloud handling — the V6-critical path)
- **C:** `se3_helpers` + `factor_graph` pure-logic library (SE(3) math + GTSAM factor construction)

All three are **fully testable without ROS2 running** (Tasks B and C have zero ROS imports). Task A is a CMake message-generation package that just needs `colcon build`.

---

## Design Principle: Pure Logic Separation

Follow the existing pattern from `src/pointcloud_fusion/test/test_pointcloud_fusion.py:1-40`, which copies a pure-numpy function and tests it in isolation without `rclpy`. Every new module must have:
1. A `core`/`utils` module with **zero** ROS imports (only numpy, scipy, gtsam, open3d, cv2).
2. A pytest test file that exercises it with **synthetic data** (random arrays, known transforms).
3. A thin ROS wrapper (added in Phase 2/3) that does only message serialization.

---

## Task A: `sensor_fusion_msgs` Package

**Agent:** 1 (quick, ~1 hour)
**Blocks:** Phase 2 Task C (GTSAM tracker node needs these messages)

- [x] **A.1** Locate the Jetson-side `sensor_fusion_msgs` package. Found at `worktrees/assistive_multiview_prosthesis/ample-linden/assistive_multiview_prosthesis/docker_ws/multi_cam_localization/sensor_fusion_msgs/`.

- [x] **A.2** Copy the package into the laptop workspace:
  ```bash
  cp -r <source>/sensor_fusion_msgs/ src/sensor_fusion_msgs/
  ```

- [x] **A.3** Verify the package contains these `.msg` files (from V6 §2.2):
  - `msg/MarkerPoseObservation.msg`
  - `msg/DynamicMarkerObservation.msg`
  - `msg/DynamicArmPoseObservation.msg`

- [x] **A.4** Verify `CMakeLists.txt` uses `rosidl_generate_interfaces` for all three messages and that `package.xml` has `<buildtool_depend>rosidl_default_generators</buildtool_depend>` and `<exec_depend>rosidl_default_runtime</exec_depend>`.

- [x] **A.5** Build the package in-container:
  ```bash
  make build-pkg PKG=sensor_fusion_msgs
  ```
  Must succeed with zero errors.

- [x] **A.6** Verify the Python messages are importable:
  ```bash
  python3 -c "from sensor_fusion_msgs.msg import MarkerPoseObservation; print('OK')"
  python3 -c "from sensor_fusion_msgs.msg import DynamicMarkerObservation; print('OK')"
  python3 -c "from sensor_fusion_msgs.msg import DynamicArmPoseObservation; print('OK')"
  ```

---

## Task B: `cloud_utils` Library

**Agent:** 1 (independent, ~1 day)
**Blocks:** Phase 2 Task A (keyframe buffer), Phase 3 Task A (TSDF fusion), Phase 3 Task B (SIFT node)
**Key V6 insight:** The recorded bags show ALL clouds are `height==1` (unorganized). This library makes unorganized-safe handling the default.

This is a **pure Python package** (not a ROS package) — it lives inside the `keyframe_buffer` package as a sub-module but has no ROS dependencies.

- [x] **B.1** Create the directory structure:
  ```
  src/keyframe_buffer/
  ├── keyframe_buffer/
  │   ├── __init__.py
  │   └── cloud_utils.py        # Pure numpy, zero ROS imports
  ├── test/
  │   └── test_cloud_utils.py   # pytest, synthetic data
  ├── resource/
  │   └── keyframe_buffer       # empty marker file
  ├── setup.py
  └── package.xml
  ```
  Create `setup.py` and `package.xml` following the pattern in `src/pointcloud_fusion/setup.py:1-27` and `src/pointcloud_fusion/package.xml:1-24`.

- [x] **B.2** Implement `cloud_utils.py` with these functions (all pure numpy, vectorized, no Python loops over points):

  **`detect_organization(height: int) -> bool`**
  Returns `True` if `height > 1` (organized). One-liner but documents the convention.

  **`mask_unorganized_cloud(cloud_xyz, mask, K, pose) -> np.ndarray`**
  - `cloud_xyz`: `(N, 3)` points in world frame
  - `mask`: `(H, W)` binary array (from SAM)
  - `K`: `(3, 3)` intrinsics
  - `pose`: `(4, 4)` T_world_camera
  - Returns: `(N,)` boolean keep array
  - Algorithm: transform points to camera frame, project to pixels, test against mask. Vectorized. See V6 plan §5.5 for reference implementation.

  **`mask_organized_cloud(cloud_xyz, mask) -> np.ndarray`**
  - `cloud_xyz`: `(H, W, 3)` organized
  - `mask`: `(H, W)` binary
  - Returns masked points via direct indexing.

  **`build_depth_image(cloud_xyz, K, pose, H, W, mask=None) -> np.ndarray`**
  - Rasterize unorganized cloud into `(H, W)` float32 depth image.
  - Z-buffer: keep nearest point per pixel (sort far-to-near, overwrite).
  - See V6 plan §5.5 for reference implementation.

  **`project_3d_to_2d(point_3d, K, pose) -> tuple[int, int]`**
  - Project a single 3D world point to pixel `(u, v)`.
  - Used by TSDF fusion to find the SAM click point.

  **`lookup_depth_3d(cloud, organized, u, v, K=None, pose=None) -> np.ndarray | None`**
  - For organized: return `cloud[v, u]` directly.
  - For unorganized: find nearest 3D point to the ray through pixel `(u, v)`. Used by SIFT node for depth lookup at keypoint locations.

- [x] **B.3** Write `test/test_cloud_utils.py` with synthetic data tests:
  - **Test projection round-trip:** Generate random 3D points, project to 2D, verify re-projection error < 0.5 px.
  - **Test unorganized masking:** Create a synthetic scene (points in front of camera), create a mask covering half the image, verify the keep array matches expected geometry.
  - **Test depth rasterization:** Generate points at known depths, rasterize, verify depth image values match within 1mm.
  - **Test organized fast-path:** Create `(H, W, 3)` cloud, apply mask, verify output shape and values.
  - **Test z-buffer correctness:** Two points project to same pixel at different depths — verify nearest is kept.
  - **Test edge cases:** empty cloud, all points behind camera, points at image boundary.
  - Run with: `python3 -m pytest src/keyframe_buffer/test/test_cloud_utils.py -v` (works on host since no ROS deps).

- [x] **B.4** Verify the package builds in-container:
  ```bash
  make build-pkg PKG=keyframe_buffer
  ```
  (It will build even though the node doesn't exist yet — `setup.py` just needs valid structure.)

---

## Task C: `se3_helpers` + `factor_graph` Library

**Agent:** 1 (independent, ~1.5 days)
**Blocks:** Phase 2 Task C (GTSAM tracker node), Phase 3 Task B (SIFT Umeyama)
**Note:** This depends on Task A being complete (needs `sensor_fusion_msgs` for type references in factor construction, but the core math does not).

This lives inside the `gtsam_tracker` package as sub-modules.

- [x] **C.1** Create the directory structure:
  ```
  src/gtsam_tracker/
  ├── gtsam_tracker/
  │   ├── __init__.py
  │   ├── se3_helpers.py        # Pure numpy, zero ROS imports
  │   └── factor_graph.py       # GTSAM + numpy, zero ROS imports
  ├── test/
  │   ├── test_se3_helpers.py
  │   └── test_factor_graph.py
  ├── resource/
  │   └── gtsam_tracker
  ├── setup.py
  └── package.xml
  ```

- [x] **C.2** Implement `se3_helpers.py` (pure numpy):

  **`pose_to_matrix(pose_msg) -> np.ndarray`**
  Convert `geometry_msgs/Pose` (position + quaternion) to `(4, 4)` homogeneous matrix.

  **`matrix_to_pose(T) -> tuple`**
  Convert `(4, 4)` matrix to `(translation_tuple, quaternion_tuple)`.

  **`pose3_to_matrix(pose3: gtsam.Pose3) -> np.ndarray`**
  GTSAM Pose3 to numpy matrix.

  **`matrix_to_pose3(T: np.ndarray) -> gtsam.Pose3`**
  Numpy matrix to GTSAM Pose3.

  **`inverse_se3(T) -> np.ndarray`**
  Efficient SE(3) inverse (transpose rotation, negate translated).

  **`compose_se3(T1, T2) -> np.ndarray`**
  Matrix multiply (document the order convention: T1 @ T2).

  **`relative_transform(T_from, T_to) -> np.ndarray`**
  Returns `inv(T_from) @ T_to`.

  **`quaternion_to_rotation_matrix(q) -> np.ndarray`**
  `(3, 3)` from `(x, y, z, w)`.

  **`rotation_matrix_to_quaternion(R) -> np.ndarray`**
  Shepperd's method or scipy.

  **`angle_between_quaternions(q1, q2) -> float`**
  Returns angle in radians (for spatial gate: 15° threshold).

- [x] **C.3** Write `test/test_se3_helpers.py`:
  - **Round-trip:** matrix → pose → matrix, verify identity within 1e-9.
  - **Inverse:** `T @ inv(T) ≈ I`.
  - **Compose:** verify `compose(A, B) == A @ B`.
  - **Quaternion angle:** known 30° rotation → `angle_between` returns ~0.524 rad.
  - **GTSAM round-trip:** numpy → Pose3 → numpy, verify identity.
  - Run on host: `python3 -m pytest src/gtsam_tracker/test/test_se3_helpers.py -v` (needs `gtsam` installed — run in container if host lacks it).

- [x] **C.4** Implement `factor_graph.py` (GTSAM, zero ROS):

  **`class TrajectoryFactorGraph`**
  - `__init__(self, lag_s=15.0)`: create `gtsam.BatchFixedLagSmoother` or `gtsam.FixedLagSmoother` with the given lag. Use ISAM2 params: `relinearizeThreshold=0.001`, `relinearizeSkip=3`.
  - `add_odometry_factor(self, key_head, key_arm, stamp, delta_head, delta_arm, noise_head, noise_arm)`: add `BetweenFactorPose3` for consecutive odom deltas on both chains.
  - `add_aruco_prior(self, key, T_map_imu, covariance_diag)`: add `PriorFactorPose3`.
  - `add_visual_between(self, key_head, key_arm, T_head_arm, covariance)`: add `BetweenFactorPose3`.
  - `add_range_factor(self, key_head, key_arm, max_distance, sigma)`: soft constraint when head-arm distance exceeds threshold.
  - `update(self)`: run smoother update.
  - `get_pose(self, key) -> gtsam.Pose3`: query current estimate.
  - Key generation: `gtsam.symbol('h', index)` for head, `gtsam.symbol('a', index)` for arm.

  **`noise_from_covariance_diag(cov_diag_6) -> gtsam.noiseModel`**
  `gtsam.noiseModel.Diagonal.Sigmas(gtsam.Point6(cov_diag_6))`.

  **`default_odom_noise(sigma_t=0.01, sigma_r=0.01) -> gtsam.noiseModel`**
  Fixed diagonal noise when odom covariance unavailable.

- [x] **C.5** Write `test/test_factor_graph.py` with synthetic trajectories:
  - **Odom-only recovery:** Feed a known smooth trajectory (e.g., constant velocity circle) as between-factors. Verify recovered poses match ground truth within 1cm.
  - **Prior correction:** Inject a drifting odom sequence, then add a prior factor at the true pose. Verify the smoother pulls the estimate back toward truth.
  - **Marginalization:** Feed 1000 factors, verify the graph doesn't grow unbounded (check that old keys are marginalized — this is the FixedLagSmoother's job).
  - **Range factor:** Set head and arm 1.5m apart with a 1.0m range factor. Verify the penalty pulls them toward 1.0m.
  - **Two-chain test:** Head and arm chains with independent odom, connected by a visual between-factor. Verify cross-chain consistency.
  - Run in container: `python3 -m pytest src/gtsam_tracker/test/test_factor_graph.py -v`.

- [x] **C.6** Implement `umeyama.py` (SVD-based 3D-3D alignment, pure numpy):
  - **`umeyama(src_points, dst_points) -> tuple[np.ndarray, np.ndarray]`**
  - Returns `(T_4x4, covariance_6x6)`.
  - See V6 plan §3.1 for reference implementation.
  - Test: generate known R, t, apply to random points + small noise, verify recovery within tolerance.

---

## Verification Gate

Phase 1 is complete when ALL of the following are true:
1. `sensor_fusion_msgs` builds and all three message types are importable in Python.
2. `python3 -m pytest src/keyframe_buffer/test/test_cloud_utils.py -v` passes (all tests green).
3. `python3 -m pytest src/gtsam_tracker/test/test_se3_helpers.py -v` passes.
4. `python3 -m pytest src/gtsam_tracker/test/test_factor_graph.py -v` passes.
5. Both `keyframe_buffer` and `gtsam_tracker` packages build with `colcon build` (even though nodes don't exist yet — just package structure).

---

## Dependency Graph

```
Task A (sensor_fusion_msgs) ──────────────────┐
                                               ├──► Phase 2 Task C (GTSAM node)
Task B (cloud_utils) ─────┬────────────────────┤
                          ├──► Phase 2 Task A (keyframe buffer)
Task C (se3 + factor_graph)┤
                          ├──► Phase 3 Task A (TSDF fusion)
                          └──► Phase 3 Task B (SIFT node)
```

Tasks A, B, C can all proceed in parallel after Phase 0.
