# Jetson D435i OpenVINS Marker Pointcloud Deep Reference Report

Date: 2026-05-20

This is a detailed technical reference report for the Jetson-side implementation of the dual Intel RealSense D435i, modified OpenVINS, ArUco marker, dynamic ID2 arm update, and pointcloud publishing system. It is intended as source material for writing a polished semester project implementation section.

## How To Use This Reference

This document is not the final semester report text. It is deliberately more detailed than the final implementation section should be, so that this chat, another chat, or a human writer can extract accurate explanations, parameter values, file references, tables, and figure ideas.

The labels below are used throughout the report:

- Use in semester report: material that can be rewritten directly into the final implementation chapter.
- Reference detail / appendix material: useful technical detail that may be too long for the main report body, but can support an appendix, methods subsection, or examiner question.
- Internal caution / not final evidence yet: information that should not be stated as a final validated result unless the required validation evidence is available.

## Scope

Use in semester report:

The implementation section should focus on the Jetson-side system. The Jetson runs the two D435i cameras, the modified OpenVINS instances, the fixed-marker and dynamic-marker measurement pipelines, the `marker_map` common frame, and the pointcloud publishing interface. The x86 PC is a downstream consumer of Jetson topics, not part of the Jetson implementation itself.

In scope:

- dual Intel RealSense D435i setup on the Jetson;
- ROS 2 Jazzy and Docker runtime environment;
- head and arm RealSense launch configuration;
- camera and IMU calibration used by OpenVINS;
- fixed ArUco marker ID0 as the static `marker_map` reference;
- dynamic ArUco marker ID2 attached to the arm/prosthesis;
- calibrated static transform between ID2 and the arm D435i;
- modified OpenVINS marker updates, marker-map lock, and reanchor behavior;
- dynamic ID2 measurement, update, and guarded arm reanchor pipeline;
- raw D435i color pointcloud publishing;
- optional Jetson-side `marker_map` pointcloud republishing;
- Jetson-published topics, frames, message types, launch modes, and parameters;
- validation evidence and known limitations.

Out of scope:

- x86 implementation details;
- x86 segmentation, collision checking, or prediction algorithms;
- x86 launch-file structure;
- unconfirmed experimental x86 code as final implementation evidence.

Internal caution / not final evidence yet:

Experimental or uncommitted x86 integration files, including files such as `x86_raw_pointcloud_marker_map.launch.py`, should be mentioned only as downstream integration context unless the x86 authors confirm that they are final evidence for their own section.

## Executive Technical Summary

Use in semester report:

The Jetson implementation provides a live ROS 2 system for localizing two Intel RealSense D435i cameras in a common marker-referenced coordinate frame. One D435i is mounted as the head/environment camera and one D435i is mounted on or near the prosthesis/arm. Each camera provides RGB images and IMU data to a separate OpenVINS instance. The OpenVINS estimators are modified so that a static ArUco marker, ID0, can anchor and correct the VIO estimates in a shared `marker_map` frame.

The implementation also supports a dynamic marker, ID2, mounted rigidly on the arm/prosthesis assembly. Unlike ID0, ID2 is not a fixed world marker and must not be listed in `marker_fixed_ids`. Instead, the head camera observes ID2, a Jetson measurement node combines that observation with the current head OpenVINS pose and a calibrated ID2-to-arm-D435i extrinsic transform, and the resulting arm pose measurement is published to the arm OpenVINS instance. Arm OpenVINS can then either observe this measurement, apply bounded EKF updates, report would-reanchor decisions, or actively perform guarded initial-lock/reanchor behavior depending on the selected launch mode.

For perception, the Jetson can publish raw color pointclouds from both D435i cameras. It can also optionally transform and republish those pointclouds into `marker_map` on the Jetson. A newer interface mode disables the Jetson-side transformed pointcloud republishers while keeping raw pointclouds, TF, odometry, and marker-map lock information available for an x86 PC to transform, merge, filter, and process downstream.

## Runtime Environment And Deployment

Use in semester report:

The runtime environment is built around a Jetson Orin Nano running ROS 2 Jazzy inside Docker. The Docker container has direct access to RealSense devices, GPU/NVIDIA runtime support, host networking, and CycloneDDS peer networking. This matters because the D435i IMU and multi-camera streams must be visible inside the container with stable timestamps and predictable communication to downstream machines.

Reference detail / appendix material:

The main Docker service is defined in:

- `docker_ws/docker-deployment/docker-compose.yml`

The service uses:

- `network_mode: "host"`;
- `privileged: true`;
- NVIDIA runtime;
- `ipc: host`;
- device mounts including `/dev:/dev` and `/run/udev:/run/udev:ro`;
- the workspace mounted into the container;
- CycloneDDS configuration mounted at `/tmp/cyclonedds_peer.xml`;
- ROS environment variables including `ROS_DOMAIN_ID=0`, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, and `CYCLONEDDS_URI=file:///tmp/cyclonedds_peer.xml`.

CycloneDDS peer networking is configured in:

- `docker_ws/docker-deployment/cyclonedds_peer.xml`

The DDS file sets domain ID 0, disables multicast, selects network interface address `10.42.0.2`, and lists peers `10.42.0.1` and `10.42.0.2`. In the report, this can be described as an explicit peer setup for communication between the Jetson and downstream PC instead of relying on multicast discovery.

The RealSense setup notes are documented in:

- `docker_ws/docker-deployment/README_d435i_jetson_jazzy.md`

Important runtime facts from that document:

- host: Ubuntu 24.04 / JetPack 6.x / L4T 36.4.7;
- container: ROS 2 Jazzy;
- D435i firmware: 5.17.0.10;
- RealSense host-side patching was required for reliable D435i IMU use on Jetson;
- only one RealSense node should be run per physical camera;
- important topics include RGB image, depth image, pointcloud, accel, gyro, and unified IMU.

Internal caution / not final evidence yet:

The `realsense_camera` Docker service entrypoint uses `start_realsense_camera.sh`, which launches the simpler `dual_d435i.launch.py`. That service is useful as a raw camera entrypoint, but the main integrated OpenVINS + marker + dynamic ID2 system is `dynamic_id2_arm_update_live.launch.py`.

Figure idea:

Draw a runtime diagram with four blocks: Jetson host, Docker ROS 2 container, two D435i USB devices, and downstream x86 PC over CycloneDDS peer networking.

## Integrated Jetson Launch Architecture

Use in semester report:

The main integrated live launch file is:

- `docker_ws/multi_cam_localization/sensor_fusion_bringup/launch/dynamic_id2_arm_update_live.launch.py`

This launch file is the integration point for the actual Jetson system. It includes the per-camera RealSense/OpenVINS launches, starts the head and arm marker pose pipelines, starts the dynamic arm pose measurement node, and optionally starts preview, RViz, bag recording, and pointcloud publishing.

The included per-camera launches are:

- `docker_ws/multi_cam_localization/sensor_fusion_bringup/launch/head_d435i_openvins_phase2.launch.py`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/launch/arm_d435i_openvins_phase2.launch.py`

The head launch starts:

- the head RealSense node in namespace `/head` with node name `d435i_head`;
- the head modified OpenVINS node in namespace `/ov_msckf`;
- optional head `pointcloud_to_frame_node` for `marker_map` pointcloud republishing.

The arm launch starts:

- the arm RealSense node in namespace `/arm` with node name `d435i_arm`;
- the arm modified OpenVINS node in namespace `/ov_msckf_arm`;
- optional arm `pointcloud_to_frame_node` for `marker_map` pointcloud republishing;
- dynamic ID2 update parameters that are consumed by the arm OpenVINS instance.

Reference detail / appendix material:

`dynamic_id2_arm_update_live.launch.py` reads:

- `docker_ws/multi_cam_localization/sensor_fusion_bringup/config/dynamic_id2_arm_update.yaml`

It uses that file to set defaults for launch mode, pointcloud behavior, dynamic measurement settings, and dynamic OpenVINS update/reanchor settings. Launch arguments can override these values.

The simpler dual camera launch is:

- `docker_ws/multi_cam_localization/sensor_fusion_bringup/launch/dual_d435i.launch.py`

This launch starts the two RealSense camera nodes using `d435i_cameras.yaml`. It is a simpler RealSense-only/raw dual-camera launch, not the main OpenVINS marker-update system. Phrase this carefully in the semester report.

Use in semester report:

The system uses two D435i cameras because the task needs multiple viewpoints for assistive prosthesis perception. The head camera provides a stable environment-facing viewpoint and can observe the arm-mounted marker ID2. The arm camera moves with the prosthesis and provides local sensing near the hand/arm. Both camera trajectories are estimated in the same `marker_map` frame so downstream perception can combine information from both viewpoints.

Figure idea:

Draw a ROS architecture diagram with these blocks:

- `/head/d435i_head` RealSense;
- `/arm/d435i_arm` RealSense;
- `/ov_msckf/run_subscribe_msckf_marker`;
- `/ov_msckf_arm/run_subscribe_msckf_marker`;
- head and arm marker pose nodes;
- `dynamic_arm_pose_measurement_node.py`;
- optional pointcloud-to-`marker_map` nodes;
- `/tf`, `/tf_static`, odometry, marker observations, raw pointclouds, and optional transformed pointclouds.

## Dual D435i RealSense Configuration

Use in semester report:

Each D435i is launched as a separate RealSense node with a fixed serial number. The head camera serial is `336222071386`, and the arm camera serial is `310622071850`. Both cameras publish RGB images and a unified IMU stream for OpenVINS. Depth and pointcloud generation are optional because they add Jetson load.

Reference detail / appendix material:

Head launch RealSense settings:

| Parameter | Value |
|---|---|
| Namespace | `/head` |
| Node name | `d435i_head` |
| Serial | `_336222071386` |
| TF prefix | `head_` |
| Color profile | `640x480x30` |
| Gyro FPS | `200` |
| Accel FPS | `200` |
| `unite_imu_method` | `2` |
| Infra streams | disabled |
| Depth profile | `640x480x15` when pointclouds are enabled |

Arm launch RealSense settings:

| Parameter | Value |
|---|---|
| Namespace | `/arm` |
| Node name | `d435i_arm` |
| Serial | `_310622071850` |
| TF prefix | `arm_` |
| Color profile | `640x480x30` |
| Gyro FPS | `200` |
| Accel FPS | `200` |
| `unite_imu_method` | `2` |
| Infra streams | disabled |
| Depth profile | `640x480x15` when pointclouds are enabled |

Both phase-2 launches expose:

- `enable_pointclouds`;
- `enable_marker_map_pointclouds`;
- `pointcloud_decimation_enable`;
- `pointcloud_max_rate_hz`;
- `pointcloud_voxel_leaf_m`;
- `pointcloud_max_range_m`;
- `pointcloud_decimation_magnitude`;
- `pointcloud_require_marker_map_locked`;
- `enable_pointcloud_neon_fix`.

Use in semester report:

The `hold_back_imu_for_frames` setting is enabled by default through the integrated dynamic launch. This asks the RealSense node to publish IMU and image messages in a more chronological order, which helps OpenVINS under Jetson load.

## Camera And IMU Calibration

Use in semester report:

OpenVINS requires camera intrinsics, camera distortion, camera-IMU extrinsics, camera-IMU time offset, and IMU noise parameters. These values were generated from Kalibr-style calibration artifacts and stored in the OpenVINS configuration folders for each D435i.

Head OpenVINS config folder:

- `docker_ws/multi_cam_localization/sensor_fusion_bringup/config/openvins/head_d435i_336222071386/`

Arm OpenVINS config folder:

- `docker_ws/multi_cam_localization/sensor_fusion_bringup/config/openvins/arm_d435i_310622071850/`

The key files are:

- `estimator_config.yaml`;
- `kalibr_imucam_chain.yaml`;
- `kalibr_imu_chain.yaml`.

Reference detail / appendix material:

Camera intrinsics and time shifts:

| Camera | Intrinsics `[fx, fy, cx, cy]` | Distortion model | Time shift cam-IMU |
|---|---|---|---:|
| Head D435i | `[599.1739928442613, 599.2682460042912, 324.9719463276537, 250.51904788461366]` | `radtan` | `0.01334857234167497 s` |
| Arm D435i | `[599.0144910569156, 599.5793593987496, 330.4358247492389, 244.51650425749176]` | `radtan` | `0.008816270957429255 s` |

Distortion coefficients:

| Camera | Distortion coefficients |
|---|---|
| Head D435i | `[0.11030606589572783, -0.20736097595170666, -0.002276220265004869, 0.00184355291590457]` |
| Arm D435i | `[0.10660040555500379, -0.19725984022276483, -0.0017422990584042355, 0.0022020513377890858]` |

IMU noise parameters:

| Camera | Accel noise density | Accel random walk | Gyro noise density | Gyro random walk | Update rate |
|---|---:|---:|---:|---:|---:|
| Head D435i | `0.009278837334054291` | `0.0002804825905759594` | `0.0019841370058405537` | `1.7544399415266354e-05` | `200 Hz` |
| Arm D435i | `0.010805145755125` | `0.0003854976741129608` | `0.0025606617446109967` | `1.507779944779771e-05` | `200 Hz` |

OpenVINS estimator config enables online calibration refinement for:

- camera extrinsics;
- camera intrinsics;
- camera time offset.

However, the starting calibration still comes from the Kalibr files. In the final report, explain this as using offline calibration as a strong initial estimate with OpenVINS allowed to refine some parameters during estimation.

Calibration artifacts for figures:

- `docker_ws/calibration/head/d435i_336222071386/camera_intrinsics/head_camera_intrinsics_ros1-report-cam.pdf`
- `docker_ws/calibration/head/d435i_336222071386/camera_intrinsics_from_dynamic/head_rgb_imu_dynamic_20260429_131443_ros1-report-cam.pdf`
- `docker_ws/calibration/head/d435i_336222071386/camera_imu_extrinsics_inflated10x/head_rgb_imu_dynamic_20260429_131443_ros1_inflated10x-report-imucam.pdf`
- `docker_ws/calibration/head/d435i_336222071386/imu_noise/acceleration.png`
- `docker_ws/calibration/head/d435i_336222071386/imu_noise/gyro.png`
- `docker_ws/calibration/arm/d435i_310622071850/camera_intrinsics_from_dynamic/arm_rgb_imu_dynamic_20260505_090147_ros1-report-cam.pdf`
- `docker_ws/calibration/arm/d435i_310622071850/camera_imu_extrinsics_inflated10x/arm_rgb_imu_dynamic_20260505_090147_ros1_inflated10x-report-imucam.pdf`
- `docker_ws/calibration/arm/d435i_310622071850/imu_noise/acceleration.png`
- `docker_ws/calibration/arm/d435i_310622071850/imu_noise/gyro.png`

Figure idea:

Include one calibration target or Kalibr reprojection figure, one camera/IMU transform diagram, and one compact calibration table. If report space is tight, put full calibration tables in an appendix.

## Marker Map, ID0, ID2, And Frames

Use in semester report:

The system uses ArUco markers to tie VIO estimates to a shared physical coordinate frame. Marker ID0 is a static marker used as the global reference. It defines or anchors `marker_map`. Marker ID2 is attached to the arm/prosthesis assembly and is therefore dynamic. Because ID2 moves with the arm, it must not be listed as a fixed marker in OpenVINS.

Reference detail / appendix material:

Marker configuration files:

- `docker_ws/multi_cam_localization/sensor_fusion_bringup/config/markers/head_aruco_map.yaml`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/config/markers/arm_aruco_map.yaml`

Common marker configuration:

| Item | Value |
|---|---|
| ArUco dictionary | `DICT_6X6_1000` |
| Static marker ID | `0` |
| Static marker size | `0.100 m` |
| Dynamic marker ID | `2` |
| Dynamic marker size | `0.100 m` |
| Global frame | `marker_map` |

The head marker configuration includes both:

- fixed marker ID0 as `marker_0`;
- dynamic marker ID2 as `arm_marker_2`.

The arm marker configuration includes fixed marker ID0.

Use in semester report:

The key distinction is that ID0 is a world reference, while ID2 is a moving measurement target. ID0 can be used to lock or reanchor OpenVINS to `marker_map`; ID2 must be converted into a dynamic arm pose measurement instead.

Important frame chains:

| Chain | Meaning |
|---|---|
| `marker_map -> head_imu -> head_cam0 -> head_d435i_head_color_optical_frame` | Head OpenVINS pose and calibration frames. |
| `marker_map -> arm_imu -> arm_cam0 -> arm_d435i_arm_color_optical_frame` | Arm OpenVINS pose and calibration frames. |
| `marker_map -> head_imu -> head optical frame -> arm_marker_2 -> arm optical frame -> arm_imu` | Dynamic ID2 measurement chain used to infer arm pose from head observation. |

Internal caution / not final evidence yet:

Do not write that ID2 is a fixed marker. Do not add ID2 to `marker_fixed_ids`. The phase-2 OpenVINS launches intentionally use:

```text
marker_fixed_ids: "0"
```

Figure idea:

Draw ID0 as a fixed coordinate reference on the workspace and ID2 as moving with the arm. This is one of the most important conceptual figures for the examiner.

## ID2 To Arm D435i Extrinsic Calibration

Use in semester report:

The dynamic marker pipeline requires a known rigid transform between marker ID2 and the arm D435i. The head camera observes ID2, but the arm estimator needs a pose measurement for the arm camera or arm IMU frame. Therefore, a fixed extrinsic calibration was estimated between `arm_marker_2`, the arm camera optical frame, and the arm IMU frame.

Primary output file:

- `docker_ws/multi_cam_localization/sensor_fusion_bringup/config/markers/arm_marker_extrinsics.yaml`

Calibration command documentation:

- `docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/arm_marker_id2_extrinsic_calibration_commands.md`

Reference detail / appendix material:

Final accepted calibration facts from `arm_marker_extrinsics.yaml`:

| Item | Value |
|---|---|
| Marker ID | `2` |
| Marker frame | `arm_marker_2` |
| Marker size | `0.1 m` |
| Parent camera frame | `arm_d435i_arm_color_optical_frame` |
| Parent IMU frame | `arm_imu` |
| Source bag | `bags/openvins_tests/phase2_live/dynamic_id2_arm_update_live_20260521_144725` |
| Estimation method | `robust_se3_mean_after_medoid_outlier_rejection` |
| Sample count | `573` |
| Inlier count | `561` |
| Outlier count | `12` |
| Median translation residual | `0.006033279830274279 m` |
| RMS translation residual | `0.007253446250778667 m` |
| P95 translation residual | `0.011798788046482767 m` |
| Median rotation residual | `1.42912217196374 deg` |
| RMS rotation residual | `1.9609495049982064 deg` |
| P95 rotation residual | `3.887523943237699 deg` |

The `T_armcam_marker` translation is approximately:

```text
[-0.1006131481, -0.1583371972, -0.0338905368] m
```

Transform convention:

- `T_armcam_marker` maps marker-frame points into the arm camera optical frame.
- `T_armimu_marker` maps marker-frame points into the arm IMU frame.

Calibration artifact paths:

- `docker_ws/calibration/arm/d435i_310622071850/arm_marker_id2_candidate_20260521_144725/arm_marker_extrinsics.yaml`
- `docker_ws/calibration/arm/d435i_310622071850/arm_marker_id2_candidate_20260521_144725/report/arm_marker_extrinsic_summary.yaml`
- `docker_ws/calibration/arm/d435i_310622071850/arm_marker_id2_candidate_20260521_144725/report/arm_marker_extrinsic_residuals.csv`

Internal caution / not final prose:

The 2026-05-21 replacement-mount calibration supersedes the accepted
2026-05-19 calibration from `dynamic_id2_arm_update_live_20260519_113936`. The
older result had fewer samples (`166` inliers) but slightly lower residuals.
The newer result has many more synchronized samples (`561` inliers) and still
meets the documented acceptance criteria. If this is discussed in the semester
report, present only the final 2026-05-21 calibration unless there is a reason
to explain the broken/replaced mount history.

Figure idea:

Draw the transform chain:

```text
head camera observes ID2
        |
        v
T_headcam_marker
        |
        v
T_armcam_marker^{-1}
        |
        v
arm D435i optical frame
        |
        v
arm_imu
```

If there is enough space, include a residual plot generated from `arm_marker_extrinsic_residuals.csv`.

## OpenVINS Baseline Explanation

Use in semester report:

OpenVINS is used as the base VIO estimator. VIO estimates camera/IMU motion by propagating the state with IMU measurements and correcting that state using visual feature observations. The IMU provides high-rate short-term motion prediction, while visual feature tracking provides drift correction relative to the observed scene.

At report level, describe OpenVINS as estimating:

- IMU pose;
- IMU velocity;
- gyroscope and accelerometer biases;
- camera-IMU calibration parameters;
- a sliding window of past camera poses or clones;
- feature constraints through MSCKF updates.

Why OpenVINS was suitable:

- it is a mature open-source VIO system;
- it supports monocular camera + IMU operation;
- it has ROS integration;
- it maintains covariance, which is useful for gated measurement updates;
- it can be extended with custom update modules;
- it provides odometry, path, TF, and state outputs needed by the rest of the system.

Why VIO alone was not enough:

- VIO can drift over time;
- monocular VIO depends on good feature tracking and IMU excitation;
- marker observations provide absolute or semi-absolute corrections in the workspace;
- fixed ID0 can anchor the estimate to `marker_map`;
- dynamic ID2 can provide cross-camera information about the arm pose when seen by the head camera.

Reference detail / appendix material:

The OpenVINS outputs used by the Jetson system include:

- `/ov_msckf/odomimu`
- `/ov_msckf/poseimu`
- `/ov_msckf/pathimu`
- `/ov_msckf/marker_map_locked`
- `/ov_msckf_arm/odomimu`
- `/ov_msckf_arm/poseimu`
- `/ov_msckf_arm/pathimu`
- `/ov_msckf_arm/marker_map_locked`
- `/ov_msckf_arm/dynamic_arm_update/status`

Odometry caveat:

The modified ROS 2 visualizer publishes odometry with `header.frame_id=marker_map` and child frame `head_imu` or `arm_imu`. The linear twist fields are local/body-frame values. The angular twist fields are corrected IMU gyro values from propagation, not a separately estimated angular velocity state. Downstream users should not assume the twist is parent-frame velocity without checking and transforming it.

## Modified OpenVINS Implementation

Use in semester report:

OpenVINS was modified so that marker observations can correct and anchor the VIO estimates in the common `marker_map` frame. A new marker-enabled executable, `run_subscribe_msckf_marker`, is launched for both the head and arm estimators. This executable uses the modified visualizer and VIO manager paths to consume marker pose observations and dynamic arm pose observations.

Reference detail / appendix material:

Modified-file traceability from the search:

```bash
rg "UpdaterMarkerPose|UpdaterDynamicArmPose|marker_map_locked|dynamic_arm|marker_fixed_ids|reanchor|camera queue|stale camera|unable to propagate" docker_ws/src/open_vins
```

Important modified OpenVINS files:

| File | Role |
|---|---|
| `docker_ws/src/open_vins/ov_msckf/cmake/ROS2.cmake` | Adds marker/dynamic updater sources, message dependencies, executable/test build wiring. |
| `docker_ws/src/open_vins/ov_msckf/src/run_subscribe_msckf_marker.cpp` | Marker-enabled ROS 2 entrypoint. |
| `docker_ws/src/open_vins/ov_msckf/src/core/VioManagerOptions.h` | Parses fixed marker and dynamic arm update parameters. |
| `docker_ws/src/open_vins/ov_msckf/src/core/VioManager.cpp` | Applies marker measurements, dynamic arm measurements, lock/reanchor logic, and reset behavior. |
| `docker_ws/src/open_vins/ov_msckf/src/core/VioManager.h` | Declares marker/dynamic arm methods and histories. |
| `docker_ws/src/open_vins/ov_msckf/src/ros/ROS2Visualizer.cpp` | Publishes odometry, TF, marker-map lock, dynamic status, and processes marker/dynamic queues. |
| `docker_ws/src/open_vins/ov_msckf/src/ros/ROS2Visualizer.h` | Declares marker/dynamic subscriptions and status publishing. |
| `docker_ws/src/open_vins/ov_msckf/src/update/UpdaterMarkerPose.cpp` | Fixed marker EKF update implementation. |
| `docker_ws/src/open_vins/ov_msckf/src/update/UpdaterMarkerPose.h` | Fixed marker EKF update types and options. |
| `docker_ws/src/open_vins/ov_msckf/src/update/UpdaterDynamicArmPose.cpp` | Dynamic arm pose EKF update implementation. |
| `docker_ws/src/open_vins/ov_msckf/src/update/UpdaterDynamicArmPose.h` | Dynamic arm pose EKF update types and options. |
| `docker_ws/src/open_vins/ov_msckf/src/state/Propagator.cpp` | Propagation handling for incomplete IMU coverage. |
| `docker_ws/src/open_vins/ov_msckf/src/test_dynamic_arm_pose_updater.cpp` | Tests for dynamic arm updater behavior and defaults. |

Fixed ID0 update behavior:

- consumes `sensor_fusion_msgs/MarkerPoseObservation`;
- accepts only marker IDs in `marker_fixed_ids`;
- validates frame IDs, covariance, hard gate status, marker stability, and timestamp tolerance;
- computes an innovation between the marker-derived pose and current IMU state;
- gates using chi-square, translation, and rotation limits;
- applies a bounded EKF update;
- can initialize or reanchor the estimator into `marker_map`.

Dynamic ID2 update behavior:

- consumes `sensor_fusion_msgs/DynamicArmPoseObservation`;
- checks marker ID, source camera frame, marker frame, target frame, covariance, stability, gates, and timestamp tolerance;
- supports measurement-only mode;
- supports bounded EKF update mode;
- supports guarded initial-lock and reanchor mode;
- skips or limits updates after recent fixed-marker updates;
- uses sample windows, velocity fit, translation scatter, rotation scatter, trigger thresholds, and cooldown gates for reanchor decisions.

Jetson timing and robustness changes:

- OpenVINS tracking frequency and feature budgets were reduced for Jetson-safe dual-estimator operation.
- The ROS camera queue is capped at 3 frames and stale camera frames are dropped.
- The propagation path warns when complete IMU coverage is unavailable instead of treating that case as a fatal assertion path.
- `hold_back_imu_for_frames` is used in RealSense launch configuration to improve image/IMU ordering.

Figure idea:

Draw OpenVINS baseline with two added update paths: fixed ID0 marker updates and dynamic ID2 arm pose updates.

## Dynamic ID2 Measurement, Update, And Reanchor Pipeline

Use in semester report:

The dynamic ID2 pipeline lets the head camera provide arm pose information to the arm OpenVINS estimator. This is necessary because ID2 is mounted on the arm and moves with it. The head camera detects ID2, and the Jetson converts that detection into a pose observation of the arm frame in `marker_map`.

Pipeline:

1. The head marker node detects marker ID2 in the head camera optical frame.
2. It publishes `/head/marker_pose/dynamic_observation`.
3. `dynamic_arm_pose_measurement_node.py` subscribes to the dynamic marker observation and to the head OpenVINS odometry on `/ov_msckf/odomimu`.
4. The measurement node matches the marker observation with the corresponding head pose.
5. It composes the head pose, head camera calibration, ID2 observation, ID2-to-arm-D435i extrinsic, and arm camera/IMU calibration.
6. It publishes `/arm/marker_pose/dynamic_arm_pose_observation`.
7. The arm OpenVINS instance consumes that observation and behaves according to the selected mode.

Primary implementation file:

- `docker_ws/multi_cam_localization/sensor_fusion_bringup/scripts/dynamic_arm_pose_measurement_node.py`

Message definitions:

- `docker_ws/multi_cam_localization/sensor_fusion_msgs/msg/DynamicMarkerObservation.msg`
- `docker_ws/multi_cam_localization/sensor_fusion_msgs/msg/DynamicArmPoseObservation.msg`

Reference detail / appendix material:

`DynamicMarkerObservation` represents the pose of the dynamic marker in the observing camera frame. Its pose convention is `T_camera_marker`, mapping marker-frame points into camera-frame points.

`DynamicArmPoseObservation` represents the pose of the target arm frame in `marker_map`. Its covariance order is `x, y, z, roll, pitch, yaw` in `marker_map`. It also carries quality metadata such as reprojection error, marker distance, view angle, marker area, geometry score, head pose match timing, and covariance fallback flags.

Dynamic modes:

| Mode | Behavior |
|---|---|
| `observe` | Publish and check dynamic measurements only; no arm estimator mutation. |
| `update` | Apply bounded dynamic EKF updates; no dynamic initial-lock/reanchor. |
| `would_reanchor` | Report initial-lock/reanchor decisions without mutating state. |
| `active` | Default mode: bounded dynamic updates plus guarded dynamic initial-lock/reanchor. |

Use in semester report:

The safest way to explain the modes is that `observe` is for diagnostics and calibration, `update` is for normal bounded corrections, `would_reanchor` is for validating reanchor decisions, and `active` is the full live behavior.

Internal caution / not final evidence yet:

Formal validation of the active dynamic ID2 path should include a run or bag where these topics overlap in time:

- `/ov_msckf_arm/odomimu`
- `/arm/marker_pose/dynamic_arm_pose_observation`
- `/ov_msckf_arm/dynamic_arm_update/status`

Without that overlap, the report can claim the measurement pipeline and configuration exist, but should be careful about claiming fully validated active update/reanchor performance.

Figure idea:

Draw a dedicated dynamic pipeline diagram:

```text
/head/marker_pose/dynamic_observation
          +
/ov_msckf/odomimu
          |
          v
dynamic_arm_pose_measurement_node.py
          |
          v
/arm/marker_pose/dynamic_arm_pose_observation
          |
          v
/ov_msckf_arm dynamic arm updater
```

## Raw Pointcloud Publishing And Optional Marker-Map Republishers

Use in semester report:

The Jetson can publish raw color pointclouds from both D435i cameras. These raw pointclouds are the preferred Jetson output for x86 raw offload mode because the Jetson avoids extra transform, filtering, and downsampling work. The Jetson also implements optional `marker_map` pointcloud republishers for local testing or RViz visualization.

Raw pointcloud topics:

| Camera | Topic | Raw frame |
|---|---|---|
| Head D435i | `/head/d435i_head/depth/color/points` | `head_d435i_head_depth_optical_frame` |
| Arm D435i | `/arm/d435i_arm/depth/color/points` | `arm_d435i_arm_depth_optical_frame` |

Optional Jetson-transformed topics:

| Camera | Topic | Output frame |
|---|---|---|
| Head D435i | `/head/d435i_head/points_marker_map` | `marker_map` |
| Arm D435i | `/arm/d435i_arm/points_marker_map` | `marker_map` |
| Head status | `/head/d435i_head/points_marker_map/status` | status string |
| Arm status | `/arm/d435i_arm/points_marker_map/status` | status string |

Primary implementation file:

- `docker_ws/multi_cam_localization/sensor_fusion_bringup/src/pointcloud_to_frame_node.cpp`

Reference detail / appendix material:

The optional pointcloud transform node:

- subscribes to a raw `sensor_msgs/PointCloud2`;
- uses SensorDataQoS/best-effort input with a small queue;
- composes TF from the target frame to camera pose frame and from color optical frame to cloud source frame;
- optionally waits for `marker_map_locked`;
- optionally rate-limits output;
- optionally filters points by source-frame range;
- optionally voxel-downsamples;
- publishes transformed `sensor_msgs/PointCloud2` in `marker_map`;
- publishes JSON-like status strings such as `published`, `published_unlocked`, `rate_limited`, `tf_unavailable`, and `marker_map_not_locked`.

Use in semester report:

The key design split is:

- Jetson marker-map pointcloud mode: Jetson publishes raw clouds and transformed `marker_map` clouds.
- x86 raw offload mode: Jetson publishes raw clouds, TF, odometry/state, and lock information; downstream x86 subscribes and performs transform/merge/filter work.

Internal caution / not final evidence yet:

`enable_pointclouds:=true` alone is not the same as x86 raw offload mode. Since `pointcloud.enable_marker_map` defaults true in `dynamic_id2_arm_update.yaml`, enabling pointclouds without explicitly setting `enable_marker_map_pointclouds:=false` preserves the older Jetson marker-map republisher behavior and adds Jetson load.

Figure idea:

Use a two-column diagram:

- left: Jetson marker-map pointcloud mode;
- right: x86 raw pointcloud offload interface mode.

## Jetson To X86 Interface

Use in semester report:

The x86 PC is treated as a downstream consumer. The Jetson publishes the data needed for x86-side pointcloud fusion, filtering, segmentation, collision checking, and future pose/trajectory prediction. The Jetson report should document the interface, not the x86 implementation.

| Topic/frame | Message type | Published by Jetson node | Intended x86 use | Notes |
|---|---|---|---|---|
| `/tf` | `tf2_msgs/TFMessage` | OpenVINS, RealSense, marker nodes | Live transforms | Needed for common-frame fusion. |
| `/tf_static` | `tf2_msgs/TFMessage` | RealSense/static TF | Static optical/calibration frames | Needed for pointcloud transforms. |
| `/ov_msckf/odomimu` | `nav_msgs/Odometry` | head OpenVINS | Head pose/state | `header.frame_id=marker_map`, `child_frame_id=head_imu`. Linear twist is local/child-frame velocity; angular twist is corrected IMU gyro, not an estimated angular velocity state. |
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

Reference detail / appendix material:

Downstream x86 use should be described at the interface level only:

- subscribe to raw head and arm pointclouds;
- subscribe to `/tf` and `/tf_static`;
- subscribe to OpenVINS odometry/state topics;
- optionally subscribe to marker lock/status topics;
- transform raw pointclouds into `marker_map`;
- merge head and arm pointclouds;
- range-filter and prepare clouds for segmentation/collision checking;
- use pose, local velocity, angular-rate information, and covariance carefully for prediction.

Do not describe x86 code structure or x86 algorithms in the Jetson implementation section.

## Launch Modes

Use in semester report:

The integrated launch supports multiple modes so the same system can be used for baseline VIO tests, marker update tests, pointcloud tests, x86 offload tests, and ID2 calibration/validation.

Main command skeleton:

```bash
ros2 launch sensor_fusion_bringup dynamic_id2_arm_update_live.launch.py <arguments>
```

Launch modes:

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

Internal caution / not final evidence yet:

If drift or poor timing was observed in a test, first verify which launch arguments were used. A test launched with pointclouds enabled but without `enable_marker_map_pointclouds:=false` may have included extra Jetson-side pointcloud transform load.

## Parameter Tables

### Config Defaults From `dynamic_id2_arm_update.yaml`

Use in semester report:

`dynamic_id2_arm_update.yaml` defines the default integrated workflow. These defaults are used unless launch arguments override them.

| Group | Parameter | Value | Meaning |
|---|---|---:|---|
| Launch | `mode` | `active` | Default dynamic ID2 behavior. |
| Launch | `start_cameras` | `true` | Start live RealSense cameras. |
| Launch | `start_preview` | `true` | Start optional preview unless overridden. |
| Launch | `start_rviz` | `false` | RViz off by default. |
| Launch | `record_bag` | `false` | Bag recording opt-in. |
| Launch | `marker_detection_rate_hz` | `15.0` | Marker detection rate. |
| Launch | `hold_back_imu_for_frames` | `true` | Improve RealSense IMU/image ordering. |
| Pointcloud | `enable` | `false` | Pointclouds off by default. |
| Pointcloud | `enable_marker_map` | `true` | Jetson marker-map republishers enabled when pointclouds are enabled unless overridden. |
| Pointcloud | `max_rate_hz` | `10.0` | Optional transformed cloud rate limit. |
| Pointcloud | `voxel_leaf_m` | `0.02` | Optional voxel leaf size. |
| Pointcloud | `max_range_m` | `2.0` | Optional source range filter. |
| Pointcloud | `decimation_enable` | `true` | RealSense decimation default for pointcloud mode. |
| Pointcloud | `decimation_magnitude` | `3` | RealSense decimation magnitude. |
| Pointcloud | `enable_neon_fix` | `false` | Legacy delayed pointcloud enable fallback. |
| Pointcloud | `require_marker_map_locked` | `false` | Optional transformed clouds only require TF by default. |

### RealSense Stream Parameters

| Parameter | Head D435i | Arm D435i |
|---|---|---|
| Serial | `_336222071386` | `_310622071850` |
| Color profile | `640x480x30` | `640x480x30` |
| Depth profile | `640x480x15` when enabled | `640x480x15` when enabled |
| Gyro FPS | `200` | `200` |
| Accel FPS | `200` | `200` |
| Unified IMU | `unite_imu_method=2` | `unite_imu_method=2` |
| Infra streams | disabled | disabled |

### Jetson-Safe OpenVINS Estimator Parameters

Use in semester report:

Both head and arm OpenVINS configurations use a reduced feature/update profile to make two OpenVINS instances more stable on the Jetson.

| Parameter | Value | Reason |
|---|---:|---|
| `track_frequency` | `21.0` | Process subset of 30 Hz RGB stream. |
| `num_pts` | `200` | Feature budget for Jetson. |
| `fast_threshold` | `25` | Avoid heavier low-threshold extraction path. |
| `min_px_dist` | `15` | Space features and reduce update cost. |
| `max_clones` | `8` | Jetson-safe sliding window. |
| `max_slam` | `25` | Jetson-safe SLAM feature budget. |
| `max_slam_in_update` | `25` | Update batch limit. |
| `max_msckf_in_update` | `25` | MSCKF update batch limit. |
| `num_opencv_threads` | `2` | Reduce CPU contention between estimators. |
| `use_klt` | `true` | KLT feature tracking. |
| `try_zupt` | `false` | ZUPT disabled for normal live VIO. |
| `use_aruco` | `false` | External marker nodes handle ArUco measurements. |

### Fixed ID0 Marker Update Parameters

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

### Dynamic ID2 Measurement And Update Parameters

Measurement node parameters:

| Parameter | Value |
|---|---:|
| `dynamic_observation_topic` | `/head/marker_pose/dynamic_observation` |
| `head_pose_topic` | `/ov_msckf/odomimu` |
| `dynamic_arm_pose_observation_topic` | `/arm/marker_pose/dynamic_arm_pose_observation` |
| `dynamic_arm_measurement_status_topic` | `/arm/marker_pose/dynamic_arm_measurement/status` |
| `marker_id` | `2` |
| `target_frame` | `arm_imu` |
| `max_head_pose_dt_s` | `0.05` |
| `head_pose_buffer_seconds` | `5.0` |
| `max_pose_covariance_trace` | `10.0` |
| `max_reprojection_error_px` | `3.0` |
| `max_marker_distance_m` | `2.0` |
| `max_view_angle_deg` | `75.0` |
| `min_marker_area_px2` | `800.0` |
| `min_geometry_score` | `0.35` |

Arm OpenVINS dynamic update parameters:

| Parameter | Value |
|---|---:|
| `use_dynamic_arm_pose_updates` | `true` |
| `dynamic_arm_pose_topic` | `/arm/marker_pose/dynamic_arm_pose_observation` |
| `dynamic_arm_status_topic` | `/ov_msckf_arm/dynamic_arm_update/status` |
| `dynamic_arm_global_frame_id` | `marker_map` |
| `dynamic_arm_target_frame` | `arm_imu` |
| `dynamic_arm_source_camera_frame` | `head_d435i_head_color_optical_frame` |
| `dynamic_arm_marker_frame` | `arm_marker_2` |
| `dynamic_arm_marker_id` | `2` |
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

Mode-dependent dynamic settings:

| Mode | `dynamic_arm_measurement_only` | `dynamic_arm_allow_initial_lock` | `dynamic_arm_allow_reanchor` | `dynamic_arm_reanchor_measurement_only` |
|---|---|---|---|---|
| `observe` | `true` | `false` | `false` | `true` |
| `update` | `false` | `false` | `false` | `true` |
| `would_reanchor` | `true` | `true` | `true` | `true` |
| `active` | `false` | `true` | `true` | `false` |

### Pointcloud Override Parameters

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

## Validation Evidence And Remaining Limitations

Use in semester report:

Validation should be split by subsystem so the report does not overclaim one result based on a different test.

Validation breakdown:

| Area | What to validate | Evidence sources |
|---|---|---|
| OpenVINS-only | Odometry, path, and TF stability with pointcloud load disabled. | `phase2_dual_openvins_tf_validation.md`, head/arm live trial docs. |
| Static ID0 update/reanchor | Marker-map lock, fixed marker EKF updates, reanchor behavior. | `phase2_openvins_marker_validation.md`, `marker_correction_validation.md`. |
| Dynamic ID2 measurement | Head detects ID2 and measurement node publishes arm pose observations. | `phase2_dynamic_id2_marker_calibration_validation.md`, dynamic validation commands. |
| Dynamic ID2 active update/reanchor | Arm OpenVINS consumes dynamic observations and reports accepted/rejected update/reanchor decisions. | Requires overlapping odom, dynamic observation, and status topics. |
| Pointcloud publishing | Raw pointcloud topics publish at expected rates; optional `points_marker_map` topics have correct headers/status. | `d435i_color_pointcloud_marker_map_validation.md`. |
| Jetson-to-x86 interface | Jetson publishes raw clouds, TF, odometry/state, and marker-map lock topics needed downstream. | Topic lists, bag metadata, `ros2 topic hz`, `ros2 topic info`. |

Existing validation and support docs:

- `docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/d435i_openvins_pointcloud_implementation_report_2026-05-14.md`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/d435i_color_pointcloud_marker_map_validation.md`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/dynamic_id2_arm_update_parameters.md`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/dynamic_id2_arm_update_live_validation_commands.md`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/dynamic_id2_arm_update_live_bag_recommendations.md`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/phase2_dynamic_id2_marker_calibration_validation.md`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/phase2_dual_openvins_tf_validation.md`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/phase2_openvins_marker_validation.md`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/marker_correction_validation.md`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/marker_covariance_bag_analysis.md`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/marker_covariance_empirical_validation.md`

Internal caution / not final evidence yet:

If the final report includes a claim about active dynamic ID2 reanchoring, it should be backed by explicit status messages from `/ov_msckf_arm/dynamic_arm_update/status` and synchronized evidence from the arm odometry and dynamic arm observation topics.

## Drift And Performance Cautions

Use in semester report:

OpenVINS drift or performance issues should be interpreted carefully. Before concluding that the implementation itself drifted, verify that the intended launch and tuning were used and that the estimator had the required initialization and marker observations.

Checklist before attributing drift:

- Was the integrated launch `dynamic_id2_arm_update_live.launch.py` used?
- Was OpenVINS initialized with sufficient camera/IMU excitation?
- Was marker ID0 visible when anchoring or reanchoring was expected?
- Was ID2 kept out of `marker_fixed_ids`?
- Were the Jetson-safe OpenVINS settings active?
- Were pointclouds disabled or configured in the intended offload mode?
- Was `enable_marker_map_pointclouds:=false` explicitly set for raw offload tests?
- Were preview and RViz disabled during timing-sensitive tests?

Jetson-safe OpenVINS settings to confirm:

| Parameter | Intended value |
|---|---:|
| `track_frequency` | `21.0` |
| `num_pts` | `200` |
| `fast_threshold` | `25` |
| `min_px_dist` | `15` |
| `max_clones` | `8` |
| `max_slam` | `25` |
| `max_msckf_in_update` | `25` |
| `num_opencv_threads` | `2` |

Important pointcloud warning:

`enable_pointclouds:=true` without `enable_marker_map_pointclouds:=false` can start the old Jetson marker-map pointcloud republisher behavior, because `enable_marker_map` defaults true in the dynamic config. This adds more Jetson load and can affect comparisons between drift/performance tests.

## Suggested Figures, Graphs, And Tables

Use in semester report:

These figures would make the implementation section easier for a robotics examiner to follow.

| Figure/table | What it should show | Where to use it | Existing source or action |
|---|---|---|---|
| Runtime deployment diagram | Jetson host, Docker container, two D435i devices, DDS link to x86 | Runtime environment | Draw from Docker/CycloneDDS files. |
| Integrated ROS architecture | RealSense, OpenVINS, marker nodes, dynamic node, pointcloud topics, TF | Architecture | Draw from launch files. |
| TF/frame tree | `marker_map`, head/arm IMU frames, camera frames, optical frames | Marker map section | Capture from RViz or draw manually. |
| Physical setup photo | Head camera, arm camera, ID0, ID2 | Marker setup | Need photo/CAD if not already available. |
| Calibration target and plots | Aprilgrid, Kalibr reprojection, IMU Allan/noise plots | Calibration | Use calibration PDFs and `imu_noise/*.png`. |
| ID2 extrinsic chain | Head camera observes ID2 and infers arm pose | ID2 calibration/dynamic pipeline | Draw manually. |
| ID2 residual plot/table | Translation and rotation residuals | ID2 calibration | Generate from residual CSV if needed. |
| OpenVINS update concept | IMU propagation, visual update, ID0 update, ID2 update | OpenVINS modifications | Draw conceptual diagram. |
| Dynamic ID2 pipeline | Dynamic observation to dynamic arm pose observation to arm OpenVINS | Dynamic pipeline | Draw from topics and nodes. |
| Pointcloud mode comparison | Jetson marker-map mode vs x86 raw offload mode | Pointcloud section | Draw two-lane diagram. |
| Jetson-to-x86 interface table | Topics, types, publishers, downstream use | Interface or appendix | Use table in this report. |
| Parameter tables | RealSense, OpenVINS, marker, dynamic, pointcloud settings | Appendix | Use tables in this report. |

Reference detail / appendix material:

For the final report, avoid placing every parameter in the main body. Use the main body to explain why each subsystem exists and move long parameter tables to an appendix or supplementary section.

## Missing Information Checklist

Internal caution / not final evidence yet:

Gather or confirm these before writing final validated claims:

- exact launch command used for the final evaluated/demo run;
- final launch-time overrides for pointcloud and dynamic mode;
- confirmation that ID2 was not in `marker_fixed_ids`;
- photos or CAD images showing ID0 and ID2 physical placement;
- RViz screenshot of the TF tree in `marker_map`;
- topic-rate evidence for head/arm odometry and pointclouds;
- CPU/load evidence for OpenVINS-only, Jetson marker-map pointcloud mode, and x86 raw offload mode;
- bag/log evidence with overlapping dynamic ID2 active topics;
- drift/reanchor plots if claiming quantitative improvement;
- confirmation from x86 authors about what downstream implementation details they will document.

## Primary Source List

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

Existing docs:

- `docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/jetson_implementation_report_writing_reference_2026-05-20.md`
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

## Extraction Guide For The Semester Report

Use in semester report:

Recommended implementation-section order:

1. Start with the Jetson runtime and hardware setup.
2. Explain the integrated launch architecture.
3. Explain the two D435i streams and why two cameras are needed.
4. Explain camera/IMU calibration.
5. Explain `marker_map`, ID0, ID2, and frame conventions.
6. Explain why OpenVINS was used and what was modified.
7. Explain static ID0 update/reanchor behavior.
8. Explain dynamic ID2 measurement and arm update/reanchor behavior.
9. Explain raw pointcloud publishing and the x86 raw offload interface.
10. Summarize validation and limitations.

Suggested main-report length:

- Runtime and architecture: 1 to 1.5 pages.
- Calibration and marker setup: 1 to 1.5 pages.
- OpenVINS modifications: 1.5 to 2.5 pages.
- Dynamic ID2 pipeline: 1 to 1.5 pages.
- Pointcloud interface and validation: 1 to 1.5 pages.
- Parameters and long source tables: appendix.

Final warning for report writing:

Keep implementation claims tied to validated Jetson behavior. Use this reference for details, but do not copy internal cautions as if they were final results.
