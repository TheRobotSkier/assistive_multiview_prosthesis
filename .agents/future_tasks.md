# Future Tasks

## A. Arm D435i Setup From Calibration Output

Calibration files are available at:
`calibration/arm/d435i_310622071850/`

Create the arm setup by mirroring the head D435i configuration, after
inspecting the current launch/config patterns.

Planned values:
- Namespace: `arm`
- Camera name: `d435i_arm`
- RealSense serial: `_310622071850`
- Mount location: `prosthetic_arm`

Likely deliverables:
- `multi_cam_localization/sensor_fusion_bringup/config/openvins/arm_d435i_310622071850/estimator_config.yaml`
- `multi_cam_localization/sensor_fusion_bringup/config/openvins/arm_d435i_310622071850/kalibr_imucam_chain.yaml`
- `multi_cam_localization/sensor_fusion_bringup/config/openvins/arm_d435i_310622071850/kalibr_imu_chain.yaml`
- `multi_cam_localization/sensor_fusion_bringup/config/markers/arm_aruco_map.yaml`
- `multi_cam_localization/sensor_fusion_bringup/launch/arm_d435i_openvins.launch.py`
- `multi_cam_localization/sensor_fusion_bringup/launch/arm_marker_pose.launch.py`

The arm OpenVINS node must not collide with the head OpenVINS node. Consider a
distinct OpenVINS namespace or topic prefix such as `/ov_msckf_arm`, but inspect
existing launch files before deciding.

## B. OpenVINS Internal Marker Update / EKF Reanchor

Investigate modifying OpenVINS internals so marker updates can directly correct
the EKF state, including pose, velocity, and covariance. Start this only after
Phase 1 external marker correction is validated and committed.

Use `.agents/marker_pose_covariance_plan.md` as the starting point for marker
measurement covariance, innovation gating, and EKF update design.

## C. VIO Health Monitor And Reset/Reinitialize Behavior

Improve logic that marks VIO invalid when covariance, velocity, workspace
bounds, or marker innovation become unreasonable. This can remain external
first, then potentially move into OpenVINS after the behavior is trusted.

## D. Two-Camera Point-Cloud Fusion

Use `marker_map` or another common reference frame to align head and arm D435i
point clouds. Avoid fusing clouds directly in drifting raw OpenVINS `global`
frames.
