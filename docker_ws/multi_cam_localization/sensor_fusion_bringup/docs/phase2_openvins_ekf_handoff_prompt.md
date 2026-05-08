# Phase 2 OpenVINS EKF Handoff Prompt

Use this prompt in a new chat after the Phase 1 baseline is committed and
pushed.

```text
We have completed Phase 1 external marker covariance validation for the 100 mm
head marker setup in the assistive_multiview_prosthesis repo.

Please read:
- .agents/AGENTS.md
- .agents/marker_pose_covariance_plan.md
- .agents/future_tasks.md
- docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/marker_covariance_bag_analysis.md
- docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/marker_covariance_empirical_validation.md
- docker_ws/bags/openvins_tests/head_marker_covariance/analysis_phase1_marker_covariance/analysis_report.md

Also inspect the current Phase 1 marker integration:
- docker_ws/multi_cam_localization/sensor_fusion_bringup/scripts/aruco_marker_pose_node.py
- docker_ws/multi_cam_localization/sensor_fusion_bringup/scripts/marker_quality_monitor.py
- docker_ws/multi_cam_localization/sensor_fusion_bringup/scripts/analyze_marker_covariance_bags.py
- docker_ws/multi_cam_localization/sensor_fusion_bringup/launch/head_d435i_openvins.launch.py
- docker_ws/multi_cam_localization/sensor_fusion_bringup/launch/launch/head_marker_pose.launch.py
- docker_ws/multi_cam_localization/sensor_fusion_bringup/config/markers/head_aruco_map.yaml
- docker_ws/multi_cam_localization/sensor_fusion_bringup/config/openvins/head_d435i_336222071386/estimator_config.yaml
- docker_ws/multi_cam_localization/sensor_fusion_bringup/config/openvins/head_d435i_336222071386/kalibr_imucam_chain.yaml

Inspect likely OpenVINS Phase 2 integration points:
- docker_ws/src/open_vins/ov_msckf/src/run_subscribe_msckf.cpp
- docker_ws/src/open_vins/ov_msckf/src/core/VioManager.h
- docker_ws/src/open_vins/ov_msckf/src/core/VioManager.cpp
- docker_ws/src/open_vins/ov_msckf/src/core/VioManagerHelper.cpp
- docker_ws/src/open_vins/ov_msckf/src/core/VioManagerOptions.h
- docker_ws/src/open_vins/ov_msckf/src/state/State.h
- docker_ws/src/open_vins/ov_msckf/src/state/State.cpp
- docker_ws/src/open_vins/ov_msckf/src/state/StateHelper.h
- docker_ws/src/open_vins/ov_msckf/src/state/StateHelper.cpp
- docker_ws/src/open_vins/ov_msckf/src/state/Propagator.h
- docker_ws/src/open_vins/ov_msckf/src/state/Propagator.cpp
- docker_ws/src/open_vins/ov_msckf/src/update/UpdaterHelper.h
- docker_ws/src/open_vins/ov_msckf/src/update/UpdaterHelper.cpp
- docker_ws/src/open_vins/ov_msckf/src/update/UpdaterMSCKF.h
- docker_ws/src/open_vins/ov_msckf/src/update/UpdaterMSCKF.cpp
- docker_ws/src/open_vins/ov_msckf/src/update/UpdaterSLAM.h
- docker_ws/src/open_vins/ov_msckf/src/update/UpdaterSLAM.cpp
- docker_ws/src/open_vins/ov_msckf/src/update/UpdaterZeroVelocity.h
- docker_ws/src/open_vins/ov_msckf/src/update/UpdaterZeroVelocity.cpp
- docker_ws/src/open_vins/ov_msckf/src/ros/ROS2Visualizer.h
- docker_ws/src/open_vins/ov_msckf/src/ros/ROS2Visualizer.cpp
- docker_ws/src/open_vins/ov_core/src/types/PoseJPL.h
- docker_ws/src/open_vins/ov_core/src/types/IMU.h
- docker_ws/src/open_vins/ov_core/src/utils/quat_ops.h
- docker_ws/src/open_vins/ov_core/src/track/TrackAruco.h
- docker_ws/src/open_vins/ov_core/src/track/TrackAruco.cpp

Important Phase 1 result:
- The external marker covariance model is conservative on stationary 100 mm
  marker repeatability.
- No covariance config tuning was applied.
- Live sanity checking confirmed the expected Phase 1 limitation: when VIO
  drifts and marker ID 0 is reacquired, `/head/marker_pose/ov_corrected_odom`
  snaps back to the marker-consistent pose, but OpenVINS internal covariance
  keeps growing and the velocity estimate can remain/increase inconsistent.
- Phase 1 must remain as the committed baseline.
- Do not modify OpenVINS internals until we agree on a Phase 2 implementation
  plan.

Task:
Create a detailed implementation plan for Phase 2: integrating marker
observations into OpenVINS as a principled EKF update or reanchor/reset path.
The plan should inspect the OpenVINS codebase first, identify the correct
estimator/state/update interfaces, propose how to construct marker measurement
residuals and R_marker from the Phase 1 covariance model, define
gating/rejection behavior, and explain how pose, velocity, and covariance
should be handled without covariance hacking.

Desired Phase 2 behavior:
- Every accepted observation of a known fixed marker should be able to update
  the OpenVINS EKF state, not merely an external corrected odom wrapper.
- Marker ID 0 should define the common reference frame at initialization or
  reanchor, so future head and arm D435i instances can share `marker_map`.
- When VIO is healthy, valid marker updates should improve global pose
  consistency and reduce/maintain appropriate state uncertainty.
- When VIO has drifted, marker reacquisition should recover pose and handle
  internal uncertainty/velocity consistently rather than producing repeated
  external snaps.
- The design should preserve the Phase 1 external baseline and avoid adding the
  future arm-mounted marker ID 1 as a fixed map landmark.

Important Phase 2 requirement:
Do not assume a marker pose update alone is sufficient. Explicitly investigate
whether velocity is corrected through EKF cross-correlations, needs a
principled reset/reinitialize path, or needs a separate supported velocity
handling strategy when VIO drift has made the internal velocity inconsistent.

Do not code until the Phase 2 implementation plan is clear.
```
