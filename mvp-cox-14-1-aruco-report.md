# mvp-cox.14.1: Native OpenVINS ArUco Pipeline Investigation Report

Date: 2026-05-24

---

## AC1: Does native ArUco estimate rigid marker poses, or only corner/landmark features?

**Only individual corner landmarks.** No rigid marker pose estimation exists anywhere in the native OpenVINS ArUco pipeline.

Proof sources:

1. `docker_ws/src/open_vins/ov_core/src/track/TrackAruco.h:37-41`
   > "We track the corners of the tag as compared to the pose of the tag or any other corners."
   > "The actual size of the tags do not matter since we do not recover the pose and instead just use this for re-detection and tracking of the four corners of the tag."

2. Zero occurrences of `solvePnP`, `tag_side_length`, `tag_size`, or `estimatePose` in the entire `docker_ws/src/open_vins/` tree. The `marker_pose` matches found refer to the *external* `UpdaterMarkerPose` system (mvp-cox Phase 1 feature) which consumes `/head/marker_pose/observation` messages from a separate Python node -- completely independent of the native ArUco tracker.

3. Each marker corner is stored as an independent SLAM landmark with its own feature ID. Four corners per tag, each individually triangulated and updated through the EKF. No geometric constraint enforces the corners belong to the same rigid body.

---

## AC2: Exact files/functions for detection, feature IDs, update, and visualization

### ArUco Detection
- **`docker_ws/src/open_vins/ov_core/src/track/TrackAruco.cpp:31-55`** — `feed_new_camera()` dispatches to per-camera tracking
- **`docker_ws/src/open_vins/ov_core/src/track/TrackAruco.cpp:59-171`** — `perform_tracking()` does the actual OpenCV ArUco detection:
  - Line 98: `aruco_detector.detectMarkers(img0, corners[cam_id], ids_aruco[cam_id], rejects[cam_id])` (OpenCV >= 4.7)
  - Line 100: `cv::aruco::detectMarkers(...)` (older OpenCV)
- **`docker_ws/src/open_vins/ov_core/src/track/TrackAruco.h:59`** — Dictionary is hardcoded: `cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_1000)`
- **`docker_ws/src/open_vins/ov_core/src/track/TrackAruco.h:60`** — Corner refinement: `cv::aruco::CORNER_REFINE_SUBPIX`

### Feature ID Assignment
- **`docker_ws/src/open_vins/ov_core/src/track/TrackAruco.cpp:147`** — ID formula for each corner:
  ```cpp
  size_t tmp_id = (size_t)ids_aruco[cam_id].at(i) + n * max_tag_id;
  ```
  Where `n ∈ {0,1,2,3}` (4 corners per tag). Example: tag ID 7 with `max_tag_id = 1024` produces feature IDs {7, 1031, 2055, 3079}.
- **`docker_ws/src/open_vins/ov_core/src/track/TrackBase.cpp:33-34`** — Non-aruco feature IDs start after all possible ArUco IDs:
  ```cpp
  currid = 4 * (size_t)numaruco + 1;
  ```
  This prevents ID collision: regular tracking features get IDs > `4 * max_aruco_features`.

### ArUco Feature Routing into the State
- **`docker_ws/src/open_vins/ov_msckf/src/core/VioManager.cpp:149-153`** — `trackARUCO` is created when `use_aruco: true`
- **`docker_ws/src/open_vins/ov_msckf/src/core/VioManager.cpp:931-933`** — Images fed to `trackARUCO` after VIO initialization
- **`docker_ws/src/open_vins/ov_msckf/src/core/VioManager.cpp:1019-1020`** — ArUco features pulled from database for SLAM initialization:
  ```cpp
  feats_slam = trackARUCO->get_feature_database()->features_containing(state->margtimestep(), false, true);
  ```
- **`docker_ws/src/open_vins/ov_msckf/src/core/VioManager.cpp:1076-1083`** — ArUco feature count is accounted for when adding new SLAM features (they don't count against `max_slam_features` quota)
- **`docker_ws/src/open_vins/ov_msckf/src/core/VioManager.cpp:1106-1109`** — Active ArUco features for existing SLAM landmarks are pulled from `trackARUCO->get_feature_database()`

### SLAM Update (ArUco-specific handling)
- **`docker_ws/src/open_vins/ov_msckf/src/update/UpdaterSLAM.cpp:43-44`** — Constructor takes separate `_options_aruco` with independent sigma_pix and chi2_multiplier
- **`docker_ws/src/open_vins/ov_msckf/src/update/UpdaterSLAM.cpp:159-161`** — Feature representation selection:
  ```cpp
  auto feat_rep = ((int)feat.featid < state->_options.max_aruco_features) ? state->_options.feat_rep_aruco : state->_options.feat_rep_slam;
  ```
- **`docker_ws/src/open_vins/ov_msckf/src/update/UpdaterSLAM.cpp:225-232`** — Measurement noise and chi2 threshold use ArUco-specific values for features with ID < `max_aruco_features`
- **`docker_ws/src/open_vins/ov_msckf/src/update/UpdaterSLAM.cpp:393, 409`** — Same distinction in the update path
- **`docker_ws/src/open_vins/ov_msckf/src/update/UpdaterSLAM.cpp:411-412, 422-424`** — Debug prints specific to ArUco features during chi2 rejection/acceptance

### Marginalization Protection
- **`docker_ws/src/open_vins/ov_msckf/src/state/StateHelper.cpp:637`** — ArUco landmarks are NEVER marginalized:
  ```cpp
  if ((*it0).second->should_marg && (int)(*it0).first > 4 * state->_options.max_aruco_features) {
  ```
  This explicitly skips marginalization for any feature ID ≤ `4 * max_aruco_features`.
- **`docker_ws/src/open_vins/ov_msckf/src/core/VioManager.cpp:1125`** — Comment: "We do *NOT* marginalize out our aruco tags landmarks"

### Visualization
- **`docker_ws/src/open_vins/ov_msckf/src/ros/ROS2Visualizer.cpp:127-128`** — Publisher: `pub_points_aruco` on topic `points_aruco`
- **`docker_ws/src/open_vins/ov_msckf/src/ros/ROS2Visualizer.cpp:1162-1166`** — Publishes ArUco landmarks as `PointCloud2`:
  ```cpp
  std::vector<Eigen::Vector3d> feats_aruco = _app->get_features_ARUCO();
  sensor_msgs::msg::PointCloud2 cloud_ARUCO = ROSVisualizerHelper::get_ros_pointcloud(_node, feats_aruco);
  pub_points_aruco->publish(cloud_ARUCO);
  ```
- **`docker_ws/src/open_vins/ov_msckf/src/core/VioManagerHelper.cpp:440-460`** — `get_features_ARUCO()`: iterates all `_features_SLAM`, filters `featid <= 4 * max_aruco_features`, transforms to global frame
- **`docker_ws/src/open_vins/ov_msckf/src/core/VioManagerHelper.cpp:420`** — `get_features_SLAM()` explicitly skips ArUco features: `if ((int)f.first <= 4 * state->_options.max_aruco_features) continue;` — thus SLAM cloud and ArUco cloud are disjoint
- **`docker_ws/src/open_vins/ov_msckf/src/ros/ROS1Visualizer.cpp:60-61, 697-699`** — Same pattern for ROS1 (topic `points_aruco`)

### Configuration
- **`docker_ws/src/open_vins/ov_msckf/src/core/VioManagerOptions.h:539-543`** — Parameters: `use_aruco` (bool, defaults true), `downsize_aruco` (bool, half-resolution detection)
- **`docker_ws/src/open_vins/ov_msckf/src/state/StateOptions.h:79-80`** — `max_aruco_features = 1024` (parameter name `num_aruco` in YAML)
- **`docker_ws/src/open_vins/ov_msckf/src/state/StateOptions.h:92`** — `feat_rep_aruco` separate representation (e.g., `ANCHORED_MSCKF_INVERSE_DEPTH` in rpng_aruco config)
- **`docker_ws/src/open_vins/config/rpng_aruco/estimator_config.yaml:92-105`** — The one working config with `use_aruco: true`, `up_aruco_sigma_px: 2.0`, `up_aruco_chi2_multipler: 10`

### Compile-time Gate
- **`docker_ws/src/open_vins/ov_msckf/CMakeLists.txt:19-24`** — `ENABLE_ARUCO_TAGS` option, ON by default. Requires OpenCV contrib modules (aruco).
- **`docker_ws/src/open_vins/ov_core/CMakeLists.txt:18-23`** — Same flag for ov_core.

---

## AC3: Can same marker observed by head and arm create a common frame without additional graph logic?

**No.** Each OpenVINS instance (head `ov_msckf`, arm `ov_msckf_arm`) runs as an independent ROS2 node with its own:
- `State` object (position, orientation, clones, feature map in its own global frame)
- `TrackAruco` instance (separate feature database, separate feature ID space)
- Independent SLAM feature estimation

The head instance's ArUco landmarks are expressed in the head's global frame origin (set at head VIO initialization). The arm instance's ArUco landmarks are expressed in the arm's global frame origin. These frames are unrelated unless an external mechanism (like the existing `UpdaterMarkerPose` consuming absolute marker poses) ties them together.

Even when both see the same physical tag:
- Head estimates corners 7, 1031, 2055, 3079 in frame G_head
- Arm estimates corners 7, 1031, 2055, 3079 in frame G_arm
- No path connects G_head ↔ G_arm internally

The features are assigned the same numerical IDs in both processes (derived from tag ID and corner index), but they are completely separate data structures with no shared memory or IPC.

**The native ArUco path provides zero inter-instance communication or frame alignment.**

---

## AC4: Minimum code changes required to expose camera-to-marker observations if native ArUco is reused

To go from "corner landmarks" to "marker pose observations" using the native ArUco detector, the minimum changes are:

1. **Add `tag_side_length` parameter** to `VioManagerOptions` and estimator config YAML. Currently no tag geometry is known, so solvePnP cannot run.

2. **Add `solvePnP` call in `TrackAruco::perform_tracking()`** (TrackAruco.cpp, after line 101). For each detected tag:
   - Define known 3D corner coordinates in marker frame: `[(0,0,0), (side,0,0), (side,side,0), (0,side,0)]`
   - Call `cv::solvePnP(obj_points, img_corners, K, dist, rvec, tvec)` to get camera→marker pose
   - Compose into IMU frame: `T_I_M = T_I_C * T_C_M`

3. **Construct and publish `MarkerPoseObservation` messages**. This requires:
   - Adding a ROS publisher in `TrackAruco` or in `VioManager`
   - Including marker ID, timestamp, T_C_M pose with covariance, camera_id
   - Using the existing `sensor_fusion_msgs::msg::MarkerPoseObservation` message format

4. **Wire into existing `UpdaterMarkerPose` pipeline**. The simplest approach is to publish on the same topic that `ROS2Visualizer::callback_marker_pose()` subscribes to (currently `/head/marker_pose/observation`), reusing all existing update infrastructure (VioManager.cpp:211-293, UpdaterMarkerPose).

5. **Handle the double-booking problem**: If native ArUco `use_aruco: true` is also enabled, the same tag corners would be both SLAM landmarks AND marker pose measurements, potentially double-counting information. You'd need either:
   - Disable native `use_aruco` and use only the new solvePnP path, or
   - Disable ArUco corner SLAM initialization while keeping detection active

**Estimated effort**: ~200 lines of C++ in TrackAruco.cpp + ~30 lines config additions. The majority of the work is the solvePnP → MarkerPoseObservation pipeline, not reusing the existing update machinery.

---

## AC5: Recommendation

**Build a separate marker graph frontend (mvp-cox.14.3 path).** Do not enable native `use_aruco` for this purpose.

Rationale:

| Criterion | Native `use_aruco` only | Native detection + solvePnP | Separate marker frontend |
|-----------|------------------------|-----------------------------|--------------------------|
| Provides marker poses | **No** (corner landmarks only) | Yes | Yes |
| Reuses existing update path | N/A | Yes (UpdaterMarkerPose) | Yes (UpdaterMarkerPose) |
| Reuses existing detection | Yes | Yes | Independent |
| Marker graph logic | Impossible | Needs new code | Clean separation |
| Dictionary flexibility | Hardcoded DICT_6X6_1000 | Must patch same file | Free choice |
| Risk of double-counting corners | N/A | High | Zero |
| Maintainability | N/A | Tangled with VIO internals | Clean standalone node |

The separate marker frontend approach is superior because:
1. It cleanly separates marker detection/pose estimation from the VIO estimator
2. It can use any OpenCV ArUco dictionary, any tag size, any number of markers
3. It emits `MarkerPoseObservation` messages that the existing `UpdaterMarkerPose` already consumes
4. It can observe multiple markers in one frame and publish all observations for the marker graph estimator (mvp-cox.14.4)
5. It avoids the feature ID namespace collision and double-counting problems

**Implementation plan**: Create a new C++ ROS2 node (e.g., `aruco_marker_pose_frontend`) that:
- Subscribes to camera images and camera_info
- Uses OpenCV ArUco directly (no dependency on OpenVINS internals)
- Computes full 6-DOF marker pose via solvePnP
- Publishes `MarkerPoseObservation` messages on the existing topic
- Can optionally publish a marker graph edge message when multiple markers are visible in one frame

This frontend can then replace the Python `aruco_marker_pose_node.py` while providing the richer observations needed for multi-marker graph fusion.
