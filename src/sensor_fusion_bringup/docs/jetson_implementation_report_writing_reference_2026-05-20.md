# Jetson Implementation Report Writing Reference

Date: 2026-05-20

This document is an internal writing reference for the semester project implementation section. It is not the final report text. Use it to decide what to write, which repository files to cite, which figures to include, and which implementation details must be explained.

## Scope Boundary

The semester implementation section should focus on the implemented Jetson-side system:

- dual Intel RealSense D435i setup on the Jetson;
- modified OpenVINS for both head and arm VIO;
- fixed ArUco marker ID0 updates and `marker_map` anchoring;
- dynamic ArUco marker ID2 arm pose measurement, update, and guarded reanchor pipeline;
- calibrated static transform between marker ID2 and the arm D435i;
- raw D435i color pointcloud publishing;
- optional Jetson-side `marker_map` pointcloud republishing;
- launch/config parameters, ROS topics, TF frames, odometry/state outputs, and validation.

Mention the x86 PC only as a downstream/offload consumer. Do not describe x86 code structure, algorithms, or launch files. Experimental or offload files such as `x86_raw_pointcloud_marker_map.launch.py` should only be described as downstream integration context unless the x86 authors confirm them as final.

## Recommended Semester Report Structure

Write the implementation chapter in this order so an examiner can follow the system from hardware to estimation to outputs.

### 1. Runtime Environment

Explain that the Jetson Orin Nano runs the ROS 2 Jazzy system inside Docker with direct RealSense device access and CycloneDDS peer networking.

Content to include:

- Jetson Orin Nano, Ubuntu 24.04 / JetPack 6.x / L4T 36.4.7 context.
- ROS 2 Jazzy inside Docker.
- D435i firmware version recorded as 5.17.0.10.
- Docker service uses host networking, privileged device access, NVIDIA runtime, `/dev`, `/run/udev`, and X11 mounts.
- DDS peer configuration disables multicast and explicitly lists the Jetson/x86 peer addresses.
- State that the runtime setup was important because the D435i IMU must work reliably inside the container.

Files to cite:

- `docker_ws/docker-deployment/docker-compose.yml`
- `docker_ws/docker-deployment/cyclonedds_peer.xml`
- `docker_ws/docker-deployment/README_d435i_jetson_jazzy.md`
- `docker_ws/docker-deployment/start_realsense_camera.sh`

Figure/table suggestions:

- Small runtime deployment diagram: Jetson host, Docker container, two D435i USB devices, CycloneDDS link to downstream PC.
- Table of runtime assumptions: ROS distribution, Docker network mode, DDS implementation, camera firmware, RealSense serial numbers.

### 2. Integrated Jetson Architecture

Describe the main live Jetson launch as the integration point for the actual system.

Content to include:

- `dynamic_id2_arm_update_live.launch.py` is the main integrated live launch.
- It includes:
  - `head_d435i_openvins_phase2.launch.py`
  - `arm_d435i_openvins_phase2.launch.py`
  - head marker pose launch
  - arm marker pose launch
  - `dynamic_arm_pose_measurement_node.py`
  - optional head-derived preview
  - optional RViz
  - optional bag recording
  - optional raw and transformed pointcloud publishing
- The head and arm phase-2 launches start one RealSense node and one modified OpenVINS instance per camera.
- `dual_d435i.launch.py` is a simpler RealSense-only/raw dual-camera launch. Do not call it the main OpenVINS marker-update launch.
- Explain why two cameras are used:
  - head camera observes the environment and marker ID2 on the arm/prosthesis;
  - arm camera moves with the prosthesis and provides a local viewpoint;
  - both need to be placed into the common `marker_map` frame for downstream multiview perception.

Files to cite:

- `docker_ws/multi_cam_localization/sensor_fusion_bringup/launch/dynamic_id2_arm_update_live.launch.py`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/launch/head_d435i_openvins_phase2.launch.py`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/launch/arm_d435i_openvins_phase2.launch.py`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/launch/dual_d435i.launch.py`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/package.xml`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/CMakeLists.txt`

Figure suggestions:

- Full Jetson ROS architecture graph showing RealSense nodes, OpenVINS nodes, marker nodes, dynamic measurement node, TF, odometry, raw pointclouds, and optional pointcloud republishers.
- A simplified data-flow diagram for the report body, with detailed topic table in an appendix.

### 3. Camera Calibration

Explain how the D435i RGB camera and IMU parameters enter OpenVINS.

Content to include:

- Both D435i cameras are treated as monocular RGB + IMU VIO sensors.
- Calibration includes RGB intrinsics, radial-tangential distortion, camera-IMU extrinsics, camera-IMU time shift, and IMU noise parameters.
- OpenVINS uses:
  - `estimator_config.yaml` for estimator behavior;
  - `kalibr_imucam_chain.yaml` for camera intrinsics, distortion, `T_cam_imu`, and time shift;
  - `kalibr_imu_chain.yaml` for IMU noise and update rate.
- Mention that OpenVINS is configured to allow online refinement of camera extrinsics, intrinsics, and time offset, but the initialization comes from Kalibr files.

Known calibration values to summarize:

| Camera | Serial | Intrinsics `[fx, fy, cx, cy]` | Time shift `cam_imu` | IMU topic |
|---|---:|---|---:|---|
| Head D435i | `336222071386` | `[599.174, 599.268, 324.972, 250.519]` | `0.0133486 s` | `/head/d435i_head/imu` |
| Arm D435i | `310622071850` | `[599.014, 599.579, 330.436, 244.517]` | `0.0088163 s` | `/arm/d435i_arm/imu` |

Known IMU noise values:

| Camera | Accel noise density | Accel random walk | Gyro noise density | Gyro random walk | Rate |
|---|---:|---:|---:|---:|---:|
| Head | `0.00927884` | `0.000280483` | `0.00198414` | `1.75444e-05` | `200 Hz` |
| Arm | `0.0108051` | `0.000385498` | `0.00256066` | `1.50778e-05` | `200 Hz` |

Files to cite:

- `docker_ws/multi_cam_localization/sensor_fusion_bringup/config/openvins/head_d435i_336222071386/estimator_config.yaml`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/config/openvins/head_d435i_336222071386/kalibr_imucam_chain.yaml`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/config/openvins/head_d435i_336222071386/kalibr_imu_chain.yaml`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/config/openvins/arm_d435i_310622071850/estimator_config.yaml`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/config/openvins/arm_d435i_310622071850/kalibr_imucam_chain.yaml`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/config/openvins/arm_d435i_310622071850/kalibr_imu_chain.yaml`

Calibration artifacts to use as figures or appendix evidence:

- `docker_ws/calibration/head/d435i_336222071386/camera_intrinsics/head_camera_intrinsics_ros1-report-cam.pdf`
- `docker_ws/calibration/head/d435i_336222071386/camera_intrinsics_from_dynamic/head_rgb_imu_dynamic_20260429_131443_ros1-report-cam.pdf`
- `docker_ws/calibration/head/d435i_336222071386/camera_imu_extrinsics_inflated10x/head_rgb_imu_dynamic_20260429_131443_ros1_inflated10x-report-imucam.pdf`
- `docker_ws/calibration/head/d435i_336222071386/imu_noise/acceleration.png`
- `docker_ws/calibration/head/d435i_336222071386/imu_noise/gyro.png`
- `docker_ws/calibration/arm/d435i_310622071850/camera_intrinsics_from_dynamic/arm_rgb_imu_dynamic_20260505_090147_ros1-report-cam.pdf`
- `docker_ws/calibration/arm/d435i_310622071850/camera_imu_extrinsics_inflated10x/arm_rgb_imu_dynamic_20260505_090147_ros1_inflated10x-report-imucam.pdf`
- `docker_ws/calibration/arm/d435i_310622071850/imu_noise/acceleration.png`
- `docker_ws/calibration/arm/d435i_310622071850/imu_noise/gyro.png`
- `docker_ws/calibration/head/d435i_336222071386/aprilgrid_6x6_80mm_0p3.yaml`
- `docker_ws/calibration/arm/d435i_310622071850/aprilgrid_6x6_80mm_0p3.yaml`

Figure suggestions:

- Photo or screenshot of Aprilgrid calibration target.
- One Kalibr reprojection/error plot per camera.
- One IMU Allan/noise plot per camera, or a combined table if space is limited.
- Diagram showing `T_cam_imu` convention for the D435i RGB optical frame and IMU frame.

### 4. Marker Map And Frames

Explain the marker-based common reference frame.

Content to include:

- Static marker ID0 defines the common global reference `marker_map`.
- ID0 is listed in `marker_fixed_ids` and can be used for fixed marker EKF updates and reanchoring.
- Dynamic marker ID2 is physically attached to the arm/prosthesis/D435i setup.
- ID2 is not a global fixed marker; it moves with the arm.
- Therefore, ID2 must not be added to `marker_fixed_ids`.
- The correct design is:
  - ID0: fixed reference for the map;
  - ID2: observed by the head camera and converted into an arm pose measurement using the calibrated ID2-to-arm-D435i transform.

Core frame chains:

- Head estimator chain: `marker_map -> head_imu -> head_cam0 -> head_d435i_head_color_optical_frame`
- Arm estimator chain: `marker_map -> arm_imu -> arm_cam0 -> arm_d435i_arm_color_optical_frame`
- Dynamic measurement chain: `marker_map -> head_imu -> head camera optical -> arm_marker_2 -> arm D435i optical -> arm_imu`

Marker config facts:

- ArUco dictionary: `DICT_6X6_1000`
- ID0 marker size: `0.100 m`
- ID2 marker size: `0.100 m`
- Head marker node publishes fixed ID0 observations and dynamic ID2 observations.
- Arm marker node uses ID0 for fixed marker observations.
- Phase-2 OpenVINS launches use `marker_fixed_ids: "0"` for both head and arm.

Files to cite:

- `docker_ws/multi_cam_localization/sensor_fusion_bringup/config/markers/head_aruco_map.yaml`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/config/markers/arm_aruco_map.yaml`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/launch/head_d435i_openvins_phase2.launch.py`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/launch/arm_d435i_openvins_phase2.launch.py`

Figure suggestions:

- Physical marker placement photo: ID0 fixed in the workspace, ID2 attached to arm/prosthesis.
- TF tree diagram with `marker_map` as the root.
- Transform-chain diagram separating fixed ID0 logic from dynamic ID2 logic.

### 5. ID2 To Arm D435i Fixed Transform Calibration

Explain how the static transform from the arm-mounted ID2 marker to the arm D435i was estimated.

Content to include:

- ID2 is rigidly mounted on the arm/prosthesis assembly.
- The calibration estimates a fixed transform between `arm_marker_2` and the arm D435i optical/IMU frames.
- This transform is required because the head camera observes ID2, but the estimator needs a measurement of the arm frame, typically `arm_imu`, in `marker_map`.
- The calibrated file provides:
  - `T_armcam_marker`
  - `T_armimu_marker`
  - residual covariance estimates
  - sample count and robust inlier statistics
- Clearly explain transform direction in words:
  - `T_armcam_marker` maps marker-frame points into the arm camera optical frame.
  - `T_armimu_marker` maps marker-frame points into the arm IMU frame.

Known final calibration values:

- Marker ID: `2`
- Marker frame: `arm_marker_2`
- Parent camera frame: `arm_d435i_arm_color_optical_frame`
- Parent IMU frame: `arm_imu`
- Source bag: `bags/openvins_tests/phase2_live/dynamic_id2_arm_update_live_20260519_113936`
- Samples: `167`
- Inliers: `166`
- Outliers: `1`
- Median translation residual: `0.003815 m`
- P95 translation residual: `0.010397 m`
- Median rotation residual: `1.005 deg`
- P95 rotation residual: `2.349 deg`
- `T_armcam_marker` translation: approximately `[-0.0960, -0.1566, -0.0344] m`

Files to cite:

- `docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/arm_marker_id2_extrinsic_calibration_commands.md`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/config/markers/arm_marker_extrinsics.yaml`
- `docker_ws/calibration/arm/d435i_310622071850/arm_marker_id2_candidate_20260519_113936/arm_marker_extrinsics.yaml`
- `docker_ws/calibration/arm/d435i_310622071850/arm_marker_id2_candidate_20260519_113936/report/arm_marker_extrinsic_summary.yaml`
- `docker_ws/calibration/arm/d435i_310622071850/arm_marker_id2_candidate_20260519_113936/report/arm_marker_extrinsic_residuals.csv`

Figure/table suggestions:

- Transform-chain diagram for `head camera -> ID2 -> arm camera -> arm IMU`.
- Table containing `T_armcam_marker` and `T_armimu_marker` translation plus compact rotation representation.
- Residual plot from `arm_marker_extrinsic_residuals.csv`; if no plot exists, create one showing translation and rotation residual distributions.

### 6. Why OpenVINS Was Used

Keep this conceptual and report-level.

Content to include:

- VIO combines camera feature tracking with IMU integration.
- IMU propagation gives high-rate short-term motion prediction.
- Visual updates correct IMU drift using tracked image features.
- OpenVINS is suitable because it provides a mature MSCKF-based VIO implementation, camera/IMU calibration support, ROS integration, covariance-aware state estimation, and extensible update modules.
- Pure marker tracking was not enough because markers may be intermittently visible and because camera/IMU motion estimation is needed between marker observations.
- VIO alone can drift over time, so fixed marker ID0 updates and marker-map reanchoring were added.
- Dynamic ID2 updates provide a way for the head camera to constrain the arm estimator when the head camera observes the arm-mounted marker.

Figure suggestion:

- Conceptual pipeline: IMU propagation + visual feature/MSCKF update + fixed ID0 correction + dynamic ID2 arm correction.

### 7. Modified OpenVINS

Describe the modifications by behavior, not line-by-line.

Content to include:

- A marker-enabled ROS 2 entrypoint was added: `run_subscribe_msckf_marker`.
- Fixed marker path:
  - consumes `MarkerPoseObservation`;
  - accepts only IDs in `marker_fixed_ids`;
  - validates frame IDs, stability, hard gates, covariance, time tolerance;
  - performs bounded EKF correction;
  - can initialize or reanchor the estimator into `marker_map`.
- Common reference frame:
  - both head and arm OpenVINS instances publish in `marker_map`;
  - `marker_map_locked` reports whether the estimator has locked onto the marker-map frame.
- Dynamic arm path:
  - consumes `DynamicArmPoseObservation` in the arm OpenVINS instance;
  - supports measurement-only and active update modes;
  - supports guarded dynamic initial-lock and reanchor;
  - gates by chi-square, translation, rotation, update interval, recent fixed-marker updates, sample count, time span, velocity, and scatter.
- Jetson timing stability changes:
  - lower OpenVINS feature/update load;
  - cap stale camera queue at 3 frames;
  - drop stale camera frames;
  - treat incomplete IMU propagation coverage as a warning instead of a fatal assertion path.

Modified OpenVINS files to cite:

- `docker_ws/src/open_vins/ov_msckf/cmake/ROS2.cmake`
- `docker_ws/src/open_vins/ov_msckf/src/run_subscribe_msckf_marker.cpp`
- `docker_ws/src/open_vins/ov_msckf/src/core/VioManager.cpp`
- `docker_ws/src/open_vins/ov_msckf/src/core/VioManager.h`
- `docker_ws/src/open_vins/ov_msckf/src/core/VioManagerOptions.h`
- `docker_ws/src/open_vins/ov_msckf/src/ros/ROS2Visualizer.cpp`
- `docker_ws/src/open_vins/ov_msckf/src/ros/ROS2Visualizer.h`
- `docker_ws/src/open_vins/ov_msckf/src/state/Propagator.cpp`
- `docker_ws/src/open_vins/ov_msckf/src/update/UpdaterMarkerPose.cpp`
- `docker_ws/src/open_vins/ov_msckf/src/update/UpdaterMarkerPose.h`
- `docker_ws/src/open_vins/ov_msckf/src/update/UpdaterDynamicArmPose.cpp`
- `docker_ws/src/open_vins/ov_msckf/src/update/UpdaterDynamicArmPose.h`
- `docker_ws/src/open_vins/ov_msckf/src/test_dynamic_arm_pose_updater.cpp`

Use this search as the traceability method:

```bash
rg "UpdaterMarkerPose|UpdaterDynamicArmPose|marker_map_locked|dynamic_arm|marker_fixed_ids|reanchor|camera queue|stale camera|unable to propagate" docker_ws/src/open_vins
```

Figure suggestions:

- State-estimator block diagram showing OpenVINS baseline state and added marker update modules.
- Mode diagram for fixed ID0 lock/reanchor vs dynamic ID2 update/reanchor.

### 8. Dynamic ID2 Pipeline

Describe the full Jetson pipeline for converting a head-camera observation of ID2 into an arm OpenVINS measurement.

Pipeline:

1. Head marker node observes dynamic marker ID2 in the head camera optical frame.
2. It publishes `/head/marker_pose/dynamic_observation`.
3. `dynamic_arm_pose_measurement_node.py` matches this observation with the head OpenVINS pose from `/ov_msckf/odomimu`.
4. The node uses the head camera calibration and calibrated ID2-to-arm-D435i extrinsic to compute the arm target pose in `marker_map`.
5. It publishes `/arm/marker_pose/dynamic_arm_pose_observation`.
6. Arm OpenVINS consumes the measurement and either logs, updates, reports would-reanchor, or actively reanchors depending on launch mode.

Important modes:

| Mode | Behavior |
|---|---|
| `observe` | Publish/check measurements only; no arm estimator mutation. |
| `update` | Run bounded dynamic EKF updates; no dynamic initial-lock/reanchor. |
| `would_reanchor` | Report initial-lock/reanchor decisions without mutating state. |
| `active` | Default mode: bounded updates plus guarded dynamic initial-lock/reanchor. |

Files to cite:

- `docker_ws/multi_cam_localization/sensor_fusion_bringup/scripts/dynamic_arm_pose_measurement_node.py`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/config/dynamic_id2_arm_update.yaml`
- `docker_ws/multi_cam_localization/sensor_fusion_msgs/msg/DynamicMarkerObservation.msg`
- `docker_ws/multi_cam_localization/sensor_fusion_msgs/msg/DynamicArmPoseObservation.msg`
- `docker_ws/src/open_vins/ov_msckf/src/update/UpdaterDynamicArmPose.cpp`
- `docker_ws/src/open_vins/ov_msckf/src/core/VioManager.cpp`

Figure suggestions:

- Dedicated dynamic ID2 node/topic diagram:
  - `/head/marker_pose/dynamic_observation`
  - `/ov_msckf/odomimu`
  - `dynamic_arm_pose_measurement_node.py`
  - `/arm/marker_pose/dynamic_arm_pose_observation`
  - `/ov_msckf_arm/dynamic_arm_update/status`
  - `/ov_msckf_arm/odomimu`

### 9. Pointcloud Publishing

Explain pointclouds as a Jetson output and optional Jetson-side transformed output.

Content to include:

- Raw D435i color pointcloud publishing is implemented on the Jetson through the RealSense nodes.
- Raw pointclouds are in the RealSense depth optical frames:
  - `/head/d435i_head/depth/color/points`
  - `/arm/d435i_arm/depth/color/points`
  - `head_d435i_head_depth_optical_frame`
  - `arm_d435i_arm_depth_optical_frame`
- Optional Jetson-side `marker_map` republishers are implemented using `pointcloud_to_frame_node`.
- Optional transformed topics:
  - `/head/d435i_head/points_marker_map`
  - `/arm/d435i_arm/points_marker_map`
  - status topics with `/status` suffix.
- The new split is:
  - raw pointcloud generation can stay enabled;
  - Jetson-side `marker_map` republishers can be disabled;
  - the downstream x86 PC can subscribe to raw clouds, TF, odometry, and lock topics and do transform/merge/filter work off the Jetson.

Important implementation details:

- In head/arm phase-2 launches, transformed `points_marker_map` nodes start only when both `enable_pointclouds` and `enable_marker_map_pointclouds` evaluate true.
- `enable_pointclouds:=true` alone may preserve the older Jetson marker-map republisher behavior because `enable_marker_map` defaults true in `dynamic_id2_arm_update.yaml`.
- Raw offload tests should explicitly set `enable_marker_map_pointclouds:=false`.
- `pointcloud_to_frame_node` uses sensor-data/best-effort input QoS, reliable output QoS, TF lookup, optional marker-map lock gating, max-rate limiting, source-range filtering, and voxel downsampling.
- Set `pointcloud_max_range_m:=0.0` and `pointcloud_voxel_leaf_m:=0.0` to avoid Jetson-side filtering/downsampling in raw offload mode.

Files to cite:

- `docker_ws/multi_cam_localization/sensor_fusion_bringup/src/pointcloud_to_frame_node.cpp`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/launch/head_d435i_openvins_phase2.launch.py`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/launch/arm_d435i_openvins_phase2.launch.py`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/config/dynamic_id2_arm_update.yaml`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/d435i_openvins_pointcloud_implementation_report_2026-05-14.md`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/d435i_color_pointcloud_marker_map_validation.md`

Figure suggestions:

- Side-by-side diagram:
  - Jetson marker-map pointcloud mode: Jetson publishes raw and transformed clouds.
  - x86 raw offload interface: Jetson publishes raw clouds, TF, odometry, and lock status only.
- RViz screenshot showing raw head/arm clouds and/or transformed `marker_map` clouds.

### 10. Validation And Limitations

Separate validation claims by subsystem.

Validation categories:

| Validation area | What to show | Evidence to use |
|---|---|---|
| OpenVINS-only validation | Odometry, path, TF stability with pointclouds off. | `phase2_dual_openvins_tf_validation.md`, head/arm live trial docs, bag replay if available. |
| Static ID0 validation | `marker_map_locked`, fixed marker EKF updates, reanchor behavior. | `phase2_openvins_marker_validation.md`, `marker_correction_validation.md`, covariance docs. |
| Dynamic ID2 measurement validation | ID2 detections and dynamic arm pose measurements are produced. | `phase2_dynamic_id2_marker_calibration_validation.md`, `dynamic_id2_arm_update_live_validation_commands.md`. |
| Dynamic ID2 active validation | Arm OpenVINS receives overlapping odom, dynamic measurement, and status topics. | Need overlap of `/ov_msckf_arm/odomimu`, `/arm/marker_pose/dynamic_arm_pose_observation`, and `/ov_msckf_arm/dynamic_arm_update/status`. |
| Pointcloud publishing validation | Raw pointcloud rates; optional transformed topic headers and status. | `d435i_color_pointcloud_marker_map_validation.md`. |
| x86 interface validation | Only prove Jetson publishes raw clouds, TF, odometry/state, and lock information needed downstream. | Topic list, bag metadata, `ros2 topic hz`, `ros2 topic info`, RViz screenshots. |

Validation docs to cite:

- `docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/d435i_openvins_pointcloud_implementation_report_2026-05-14.md`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/d435i_color_pointcloud_marker_map_validation.md`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/dynamic_id2_arm_update_parameters.md`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/dynamic_id2_arm_update_live_validation_commands.md`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/phase2_dynamic_id2_marker_calibration_validation.md`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/phase2_dual_openvins_tf_validation.md`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/phase2_openvins_marker_validation.md`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/marker_correction_validation.md`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/marker_covariance_bag_analysis.md`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/marker_covariance_empirical_validation.md`

Limitations to state carefully:

- Do not conclude that drift is caused by the implemented method until the test command, initialization, ID0 visibility, ID2 fixed-marker exclusion, and Jetson-safe parameters are confirmed.
- Formal dynamic ID2 active validation should use a bag or live log where the arm odometry, dynamic arm pose observation, and dynamic arm update status topics overlap in time.
- x86-side fusion, segmentation, collision checking, and prediction are outside this Jetson implementation section.

## Jetson To X86 Interface Table

Use this table in the report or appendix. Keep it interface-level only.

| Topic/frame | Message type | Published by Jetson node | Intended x86 use | Notes |
|---|---|---|---|---|
| `/tf` | `tf2_msgs/TFMessage` | OpenVINS, RealSense, marker nodes | Live transforms | Needed for common-frame fusion. |
| `/tf_static` | `tf2_msgs/TFMessage` | RealSense/static TF | Static optical/calibration frames | Needed for pointcloud transforms. |
| `/ov_msckf/odomimu` | `nav_msgs/Odometry` | head OpenVINS | Head pose/state | `header.frame_id=marker_map`, `child_frame_id=head_imu`. Linear twist is published in local/child frame. Angular twist is corrected IMU gyro, not an estimated angular velocity state. |
| `/ov_msckf_arm/odomimu` | `nav_msgs/Odometry` | arm OpenVINS | Arm pose/state | `header.frame_id=marker_map`, `child_frame_id=arm_imu`; same twist caveat. |
| `/head/d435i_head/depth/color/points` | `sensor_msgs/PointCloud2` | head RealSense | Raw head cloud | Raw depth optical frame. |
| `/arm/d435i_arm/depth/color/points` | `sensor_msgs/PointCloud2` | arm RealSense | Raw arm cloud | Raw depth optical frame. |
| `/head/marker_pose/observation` | `sensor_fusion_msgs/MarkerPoseObservation` | head marker node | ID0 reference evidence | Fixed markers only. |
| `/head/marker_pose/dynamic_observation` | `sensor_fusion_msgs/DynamicMarkerObservation` | head marker node | ID2 diagnostics/interface | Dynamic marker in head camera frame. |
| `/arm/marker_pose/dynamic_arm_pose_observation` | `sensor_fusion_msgs/DynamicArmPoseObservation` | dynamic measurement node | Arm pose measurement/status context | Consumed by Jetson arm OpenVINS. |
| `/ov_msckf/marker_map_locked` | `std_msgs/Bool` | head OpenVINS | Lock status | Transient local/reliable. |
| `/ov_msckf_arm/marker_map_locked` | `std_msgs/Bool` | arm OpenVINS | Lock status | Transient local/reliable. |
| `/head/d435i_head/points_marker_map` | `sensor_msgs/PointCloud2` | optional Jetson republisher | Optional already-transformed head cloud | Jetson marker-map mode only, not x86 raw offload. |
| `/arm/d435i_arm/points_marker_map` | `sensor_msgs/PointCloud2` | optional Jetson republisher | Optional already-transformed arm cloud | Jetson marker-map mode only. |
| `/head/d435i_head/points_marker_map/status` | `std_msgs/String` | optional Jetson republisher | Debug/status | Reports `published`, `published_unlocked`, `rate_limited`, TF/lock reasons. |
| `/arm/d435i_arm/points_marker_map/status` | `std_msgs/String` | optional Jetson republisher | Debug/status | Same as head. |

Odometry caveat for downstream prediction:

- Pose is published in `marker_map`.
- The odometry child frame is the IMU frame.
- The linear twist fields are local/body-frame values as published by the OpenVINS visualizer.
- The angular twist fields are corrected IMU gyro values from propagation, not a separately estimated angular velocity state.
- Downstream prediction should transform/use these fields consistently rather than assuming ROS parent-frame twist semantics.

## Launch Modes Table

Use `dynamic_id2_arm_update_live.launch.py` as the main launch unless explicitly validating the simpler RealSense-only launch.

| Mode | Arguments | Purpose |
|---|---|---|
| OpenVINS-only baseline | `enable_pointclouds:=false start_preview:=false start_rviz:=false` | VIO/marker stability without pointcloud load. |
| Jetson marker-map pointcloud mode | `enable_pointclouds:=true enable_marker_map_pointclouds:=true` | Raw clouds plus Jetson-transformed `points_marker_map`. |
| x86 raw offload interface | `enable_pointclouds:=true enable_marker_map_pointclouds:=false pointcloud_decimation_enable:=false pointcloud_max_range_m:=0.0 pointcloud_voxel_leaf_m:=0.0 pointcloud_decimation_magnitude:=1 start_preview:=false start_rviz:=false enable_pointcloud_neon_fix:=false` | Jetson publishes raw clouds/TF/state only. |
| Dynamic observe | `mode:=observe` | Measurement/status only. |
| Dynamic update | `mode:=update` | Bounded dynamic EKF updates, no dynamic reanchor. |
| Dynamic would-reanchor | `mode:=would_reanchor` | Report initial-lock/reanchor decisions without mutation. |
| Dynamic active | `mode:=active` | Default: bounded updates plus guarded initial-lock/reanchor. |
| Calibration recording | `mode:=observe record_bag:=true` | Safe ID2 calibration/validation recording. |

Recommended command skeleton:

```bash
ros2 launch sensor_fusion_bringup dynamic_id2_arm_update_live.launch.py <arguments>
```

Drift/performance warning:

- `enable_pointclouds:=true` without `enable_marker_map_pointclouds:=false` preserves the old Jetson marker-map pointcloud republisher behavior because `pointcloud.enable_marker_map` defaults true in `dynamic_id2_arm_update.yaml`.
- This adds Jetson CPU/TF/pointcloud load and may affect timing and drift comparisons.

## Parameter Table Plan

Separate parameter tables by source and purpose.

### A. Config Defaults From `dynamic_id2_arm_update.yaml`

| Group | Parameter | Value | Why it matters |
|---|---|---:|---|
| Launch | `mode` | `active` | Default dynamic behavior: updates plus guarded lock/reanchor. |
| Launch | `start_cameras` | `true` | Starts live RealSense nodes through included launches. |
| Launch | `start_preview` | `true` | Optional visual preview; disable for performance tests. |
| Launch | `start_rviz` | `false` | RViz off by default. |
| Launch | `record_bag` | `false` | Recording is opt-in. |
| Launch | `marker_detection_rate_hz` | `15.0` | Marker detection rate. |
| Launch | `hold_back_imu_for_frames` | `true` | Keeps RealSense IMU/image order more chronological for OpenVINS. |
| Pointcloud | `enable` | `false` | Pointclouds are opt-in due to Jetson load. |
| Pointcloud | `enable_marker_map` | `true` | When pointclouds are enabled, Jetson marker-map republishers remain enabled unless overridden. |
| Pointcloud | `max_rate_hz` | `10.0` | Optional transformed cloud rate limit. |
| Pointcloud | `voxel_leaf_m` | `0.02` | Optional Jetson voxel downsample. |
| Pointcloud | `max_range_m` | `2.0` | Optional source-frame range filter. |
| Pointcloud | `decimation_enable` | `true` | RealSense decimation filter default for pointcloud mode. |
| Pointcloud | `decimation_magnitude` | `3` | RealSense decimation magnitude. |
| Pointcloud | `enable_neon_fix` | `false` | Legacy delayed pointcloud enable fallback. |
| Pointcloud | `require_marker_map_locked` | `false` | Optional transformed clouds are gated by TF only unless changed. |

### B. RealSense Parameters From Head/Arm Phase-2 Launches

| Parameter | Head value | Arm value | Notes |
|---|---|---|---|
| Serial | `_336222071386` | `_310622071850` | Physical camera selection. |
| Color profile | `640x480x30` | `640x480x30` | RGB stream used by OpenVINS. |
| Depth profile | `640x480x15` | `640x480x15` | Enabled only when pointclouds are enabled. |
| Gyro | `200 Hz` | `200 Hz` | D435i IMU input. |
| Accel | `200 Hz` | `200 Hz` | D435i IMU input. |
| `unite_imu_method` | `2` | `2` | Unified IMU topic. |
| Infra streams | `false` | `false` | Not used. |

### C. OpenVINS Tuned Estimator Parameters

Values are active in both head and arm `estimator_config.yaml`.

| Parameter | Value | Purpose |
|---|---:|---|
| `track_frequency` | `21.0` | Process subset of 30 Hz RGB stream. |
| `num_pts` | `200` | Feature budget. |
| `fast_threshold` | `25` | Avoid heavier low-threshold feature extraction. |
| `min_px_dist` | `15` | Keep features spaced and reduce cost. |
| `max_clones` | `8` | Jetson-safe sliding window. |
| `max_slam` | `25` | Jetson-safe SLAM feature budget. |
| `max_slam_in_update` | `25` | Batch size for SLAM updates. |
| `max_msckf_in_update` | `25` | Jetson-safe MSCKF update budget. |
| `num_opencv_threads` | `2` | Reduce CPU contention between two OpenVINS instances. |
| `use_klt` | `true` | KLT feature tracking. |
| `try_zupt` | `false` | Disabled for normal live VIO. |
| `use_aruco` | `false` | Built-in OpenVINS ArUco tracking not used; external marker nodes are used. |

### D. Fixed ID0 Marker Update Parameters

Values are set in both head and arm phase-2 launches.

| Parameter | Value |
|---|---:|
| `use_marker_pose_updates` | `true` |
| `marker_fixed_ids` | `"0"` |
| `marker_time_tolerance_s` | `0.05` |
| `marker_chi2_gate` | `16.81` |
| `marker_noise_multiplier` | `1.0` |
| `marker_max_update_translation_m` | `0.25` |
| `marker_max_update_rotation_deg` | `25.0` |
| `marker_reset_translation_m` | `0.50` |
| `marker_reset_rotation_deg` | `20.0` |
| `marker_reset_min_samples` | `5` |
| `marker_reset_window_s` | `0.50` |
| `marker_reset_min_sample_dt_s` | `0.10` |
| `marker_reset_max_velocity_mps` | `2.0` |
| `marker_reset_min_velocity_std_mps` | `0.05` |
| `marker_reset_bias_gyro_std` | `0.02` |
| `marker_reset_bias_accel_std` | `0.20` |

### E. Dynamic ID2 Parameters

Values come from `dynamic_id2_arm_update.yaml` and are forwarded to the arm OpenVINS launch by `dynamic_id2_arm_update_live.launch.py`.

| Parameter | Value |
|---|---:|
| `dynamic_arm_marker_id` | `2` |
| `dynamic_arm_global_frame_id` | `marker_map` |
| `dynamic_arm_target_frame` | `arm_imu` |
| `dynamic_arm_source_camera_frame` | `head_d435i_head_color_optical_frame` |
| `dynamic_arm_marker_frame` | `arm_marker_2` |
| `dynamic_arm_time_tolerance_s` | `0.05` |
| `dynamic_arm_noise_multiplier` | `4.0` |
| `dynamic_arm_chi2_gate` | `16.81` |
| `dynamic_arm_max_update_translation_m` | `0.35` |
| `dynamic_arm_max_update_rotation_deg` | `15.0` |
| `dynamic_arm_min_update_interval_s` | `0.10` |
| `dynamic_arm_skip_after_fixed_marker_s` | `0.50` |
| `dynamic_arm_reanchor_min_samples` | `5` |
| `dynamic_arm_reanchor_window_s` | `2.0` |
| `dynamic_arm_reanchor_min_sample_dt_s` | `0.50` |
| `dynamic_arm_reanchor_max_velocity_mps` | `2.0` |
| `dynamic_arm_reanchor_max_sample_translation_std_m` | `0.12` |
| `dynamic_arm_reanchor_max_sample_rotation_std_deg` | `8.0` |
| `dynamic_arm_reanchor_trigger_translation_m` | `0.75` |
| `dynamic_arm_reanchor_trigger_rotation_deg` | `20.0` |
| `dynamic_arm_reanchor_cooldown_s` | `5.0` |
| `dynamic_arm_reanchor_skip_after_fixed_marker_s` | `3.0` |
| `dynamic_arm_reanchor_covariance_multiplier` | `2.0` |

Mode-dependent values:

| Mode | `dynamic_arm_measurement_only` | `dynamic_arm_allow_initial_lock` | `dynamic_arm_allow_reanchor` | `dynamic_arm_reanchor_measurement_only` |
|---|---|---|---|---|
| `observe` | `true` | `false` | `false` | `true` |
| `update` | `false` | `false` | `false` | `true` |
| `would_reanchor` | `true` | `true` | `true` | `true` |
| `active` | `false` | `true` | `true` | `false` |

### F. Launch-Time Pointcloud Override Tables

Jetson marker-map pointcloud testing:

| Parameter | Value |
|---|---|
| `enable_pointclouds` | `true` |
| `enable_marker_map_pointclouds` | `true` |
| `pointcloud_max_rate_hz` | default `10.0` unless overridden |
| `pointcloud_voxel_leaf_m` | default `0.02` unless overridden |
| `pointcloud_max_range_m` | default `2.0` unless overridden |
| `pointcloud_decimation_enable` | default `true` unless overridden |
| `pointcloud_decimation_magnitude` | default `3` unless overridden |

x86 raw offload interface:

| Parameter | Value |
|---|---|
| `enable_pointclouds` | `true` |
| `enable_marker_map_pointclouds` | `false` |
| `pointcloud_decimation_enable` | `false` |
| `pointcloud_max_range_m` | `0.0` |
| `pointcloud_voxel_leaf_m` | `0.0` |
| `pointcloud_decimation_magnitude` | `1` |
| `start_preview` | `false` |
| `start_rviz` | `false` |
| `enable_pointcloud_neon_fix` | `false` |

## Suggested Figures, Graphs, And Tables

| Figure/table | What it should show | Where to place it | Existing source or action |
|---|---|---|---|
| Runtime deployment diagram | Jetson, Docker container, two D435i cameras, DDS peer link | Runtime Environment | Draw from `docker-compose.yml` and `cyclonedds_peer.xml`. |
| Integrated ROS architecture | RealSense, OpenVINS, marker nodes, dynamic measurement node, pointcloud outputs, TF | Integrated Jetson Architecture | Draw manually from launch files. |
| TF tree | `marker_map`, head/arm IMU frames, camera frames, optical frames | Marker Map And Frames | Capture from RViz or draw manually. |
| Physical setup photo | Head D435i, arm D435i/prosthesis, ID0, ID2 | Marker Map And Frames | Add photo/CAD if available. |
| Calibration target image | Aprilgrid/Kalibr target | Camera Calibration | Use existing calibration reports or add a photo. |
| Kalibr reprojection/error plots | Calibration quality for head/arm | Camera Calibration | Use Kalibr PDFs listed above. |
| IMU noise plots | Allan/noise results for both IMUs | Camera Calibration | Use `imu_noise/acceleration.png` and `imu_noise/gyro.png`. |
| ID2 extrinsic transform chain | Head camera observes ID2; fixed transform gives arm pose | ID2 Calibration / Dynamic ID2 Pipeline | Draw manually; include residual table. |
| ID2 residual plot | Translation/rotation residual distribution | ID2 Calibration | Generate from `arm_marker_extrinsic_residuals.csv` if no plot exists. |
| OpenVINS update concept | IMU propagation, visual update, ID0 correction, ID2 correction | Modified OpenVINS | Draw conceptual block diagram. |
| Dynamic ID2 topic pipeline | `/head/marker_pose/dynamic_observation` to arm OpenVINS | Dynamic ID2 Pipeline | Draw from launch/config/message files. |
| Pointcloud mode comparison | Jetson marker-map mode vs x86 raw offload mode | Pointcloud Publishing | Draw two-lane architecture diagram. |
| Jetson-to-x86 interface table | Topic, type, publisher, intended x86 use | Interface appendix or pointcloud section | Use table in this document. |
| Launch modes table | Commands/arguments and purposes | Implementation details or appendix | Use table in this document. |
| Parameter tables | Runtime, RealSense, OpenVINS, marker, pointcloud parameters | Appendix or implementation details | Use table plan in this document. |

## Missing Or Unclear Information To Gather

Gather these before writing final claims:

- Final demo launch command and exact launch-time overrides used during the evaluated run.
- Confirmation that ID2 remains excluded from `marker_fixed_ids` in the evaluated launch.
- Photos or CAD images showing ID0 and ID2 physical placement.
- RViz screenshots of the TF tree and both camera poses in `marker_map`.
- Topic-rate evidence for:
  - `/ov_msckf/odomimu`
  - `/ov_msckf_arm/odomimu`
  - raw head/arm pointclouds
  - optional transformed pointclouds if used.
- Bag or log proving overlap of:
  - `/ov_msckf_arm/odomimu`
  - `/arm/marker_pose/dynamic_arm_pose_observation`
  - `/ov_msckf_arm/dynamic_arm_update/status`
- CPU/load measurements for:
  - OpenVINS-only baseline;
  - Jetson marker-map pointcloud mode;
  - x86 raw offload interface mode.
- Drift/reanchor evaluation plots:
  - pose trajectory over time;
  - marker-map lock state;
  - fixed marker update status;
  - dynamic arm update status.
- Confirmation from x86 authors of which downstream/offload implementation details they will document.

## Drift And Performance Cautions

Before writing that the system drifted, check whether the test used the intended Jetson-safe setup.

Confirm:

- The system was launched with `dynamic_id2_arm_update_live.launch.py`.
- OpenVINS was initialized with enough camera/IMU excitation.
- Marker ID0 was visible when anchoring/reanchoring was expected.
- ID2 was not added to `marker_fixed_ids`.
- Jetson-safe OpenVINS settings were active:
  - `track_frequency=21.0`
  - `num_pts=200`
  - `fast_threshold=25`
  - `min_px_dist=15`
  - `max_clones=8`
  - `max_slam=25`
  - `max_msckf_in_update=25`
  - `num_opencv_threads=2`
- Raw pointcloud offload flags were set correctly when comparing performance:
  - `enable_marker_map_pointclouds:=false`
  - `pointcloud_decimation_enable:=false`
  - `pointcloud_max_range_m:=0.0`
  - `pointcloud_voxel_leaf_m:=0.0`
  - `pointcloud_decimation_magnitude:=1`
  - `start_preview:=false`
  - `start_rviz:=false`
  - `enable_pointcloud_neon_fix:=false`

Important warning:

- `enable_pointclouds:=true` alone is not the same as x86 raw offload mode.
- Because `enable_marker_map` defaults true in `dynamic_id2_arm_update.yaml`, enabling pointclouds without explicitly disabling `enable_marker_map_pointclouds` starts the old Jetson marker-map republisher behavior and adds extra Jetson load.

## Source List For Writing

Use these files as primary local evidence.

Runtime and deployment:

- `.agents/AGENTS.md`
- `.agents/future_tasks.md`
- `.agents/phase2_openvins_handoff_status.md`
- `docker_ws/docker-deployment/docker-compose.yml`
- `docker_ws/docker-deployment/cyclonedds_peer.xml`
- `docker_ws/docker-deployment/Dockerfile`
- `docker_ws/docker-deployment/start_realsense_camera.sh`
- `docker_ws/docker-deployment/README_d435i_jetson_jazzy.md`

Bringup package:

- `docker_ws/multi_cam_localization/sensor_fusion_bringup/package.xml`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/CMakeLists.txt`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/launch/dynamic_id2_arm_update_live.launch.py`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/launch/head_d435i_openvins_phase2.launch.py`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/launch/arm_d435i_openvins_phase2.launch.py`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/launch/dual_d435i.launch.py`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/config/dynamic_id2_arm_update.yaml`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/config/d435i_cameras.yaml`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/src/pointcloud_to_frame_node.cpp`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/scripts/aruco_marker_pose_node.py`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/scripts/dynamic_arm_pose_measurement_node.py`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/scripts/calibrate_arm_marker_extrinsic.py`

Message definitions:

- `docker_ws/multi_cam_localization/sensor_fusion_msgs/msg/MarkerPoseObservation.msg`
- `docker_ws/multi_cam_localization/sensor_fusion_msgs/msg/DynamicMarkerObservation.msg`
- `docker_ws/multi_cam_localization/sensor_fusion_msgs/msg/DynamicArmPoseObservation.msg`
- `docker_ws/multi_cam_localization/sensor_fusion_msgs/package.xml`
- `docker_ws/multi_cam_localization/sensor_fusion_msgs/CMakeLists.txt`

Marker and calibration config:

- `docker_ws/multi_cam_localization/sensor_fusion_bringup/config/markers/head_aruco_map.yaml`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/config/markers/arm_aruco_map.yaml`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/config/markers/arm_marker_extrinsics.yaml`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/config/openvins/head_d435i_336222071386/estimator_config.yaml`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/config/openvins/head_d435i_336222071386/kalibr_imucam_chain.yaml`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/config/openvins/head_d435i_336222071386/kalibr_imu_chain.yaml`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/config/openvins/arm_d435i_310622071850/estimator_config.yaml`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/config/openvins/arm_d435i_310622071850/kalibr_imucam_chain.yaml`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/config/openvins/arm_d435i_310622071850/kalibr_imu_chain.yaml`
- `docker_ws/calibration/head/d435i_336222071386/`
- `docker_ws/calibration/arm/d435i_310622071850/`

OpenVINS modified files:

- `docker_ws/src/open_vins/ov_msckf/cmake/ROS2.cmake`
- `docker_ws/src/open_vins/ov_msckf/src/run_subscribe_msckf_marker.cpp`
- `docker_ws/src/open_vins/ov_msckf/src/core/VioManager.cpp`
- `docker_ws/src/open_vins/ov_msckf/src/core/VioManager.h`
- `docker_ws/src/open_vins/ov_msckf/src/core/VioManagerOptions.h`
- `docker_ws/src/open_vins/ov_msckf/src/ros/ROS2Visualizer.cpp`
- `docker_ws/src/open_vins/ov_msckf/src/ros/ROS2Visualizer.h`
- `docker_ws/src/open_vins/ov_msckf/src/state/Propagator.cpp`
- `docker_ws/src/open_vins/ov_msckf/src/update/UpdaterMarkerPose.cpp`
- `docker_ws/src/open_vins/ov_msckf/src/update/UpdaterMarkerPose.h`
- `docker_ws/src/open_vins/ov_msckf/src/update/UpdaterDynamicArmPose.cpp`
- `docker_ws/src/open_vins/ov_msckf/src/update/UpdaterDynamicArmPose.h`
- `docker_ws/src/open_vins/ov_msckf/src/test_dynamic_arm_pose_updater.cpp`

Existing documentation:

- `docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/d435i_openvins_pointcloud_implementation_report_2026-05-14.md`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/d435i_color_pointcloud_marker_map_validation.md`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/arm_marker_id2_extrinsic_calibration_commands.md`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/dynamic_id2_arm_update_parameters.md`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/dynamic_id2_arm_update_live_validation_commands.md`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/dynamic_id2_arm_update_live_bag_recommendations.md`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/phase2_dynamic_id2_marker_calibration_validation.md`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/phase2_dual_openvins_tf_validation.md`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/phase2_openvins_marker_validation.md`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/marker_correction_validation.md`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/marker_covariance_bag_analysis.md`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/marker_covariance_empirical_validation.md`

## Writing Checklist

Before drafting final semester prose:

- Decide which validation run is the official evaluated run.
- Record the exact launch command and arguments.
- Confirm ID0 and ID2 marker visibility/placement for that run.
- Confirm final `arm_marker_extrinsics.yaml` values are the values used during the run.
- Confirm active dynamic ID2 validation has overlapping arm odom, dynamic arm pose observation, and dynamic update status.
- Capture or draw all figures listed in the figure table.
- Convert long parameter lists into compact report tables; move full values to appendix if needed.
- Keep x86 discussion at the interface level only.
- Keep workspace/git-status notes out of the semester report. The semester report should describe committed or validated implementation behavior, not temporary local workspace state.

