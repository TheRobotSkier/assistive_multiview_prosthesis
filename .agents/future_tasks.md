# Future Tasks

## Current Phase 2 Status

The 100 mm marker covariance bag analysis has been completed. The Phase 1
external covariance model was conservative on stationary repeatability, and no
covariance config tuning was applied.

Live sanity checking confirmed the expected Phase 1 limit: external corrected
odom can snap back to marker ID 0 after VIO drift, but OpenVINS internal
uncertainty and velocity are not repaired by the external correction layer.

Phase 2 OpenVINS-internal marker update/reset code has been implemented locally
and passed the low-memory Docker/Jazzy build plus launch/message smoke checks.

Use the current Phase 2 handoff note before continuing:
`.agents/phase2_openvins_handoff_status.md`.

Important next gate:
- `docker_ws/src/open_vins` has been converted locally from the broken gitlink
  into a lean vendored source tree so Phase 2 OpenVINS edits can be committed
  with the parent repo
- do not commit generated `build_overlay/`, `install_overlay/`, or
  `log_overlay/` artifacts
- continue to Phase 2 launch and bag behavior validation before final commit

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
the Phase 1 baseline is committed/pushed.

Phase 2 target behavior:
- accepted observations of known fixed markers should become OpenVINS EKF
  updates, not only an external corrected-odom wrapper
- marker ID 0 should define the shared `marker_map` reference frame at startup
  or reanchor, so later head and arm D435i instances can publish point clouds in
  a common frame
- when VIO is healthy, marker updates should improve global pose consistency and
  keep state uncertainty appropriately bounded
- after VIO drift, marker reacquisition should repair pose while handling
  covariance and velocity consistently enough to avoid the repeated Phase 1
  snap-back behavior

Use `.agents/marker_pose_covariance_plan.md` as the starting point for marker
measurement covariance, innovation gating, EKF update design, and velocity
handling. Do not assume a pose-only update is sufficient if the internal VIO
velocity has already become inconsistent.

## C. VIO Health Monitor And Reset/Reinitialize Behavior

Improve logic that marks VIO invalid when covariance, velocity, workspace
bounds, or marker innovation become unreasonable. This can remain external
first, then potentially move into OpenVINS after the behavior is trusted.

## D. Two-Camera Point-Cloud Fusion

Use `marker_map` or another common reference frame to align head and arm D435i
point clouds. Avoid fusing clouds directly in drifting raw OpenVINS `global`
frames.

Planned final marker layout:
- Fixed world/common reference marker: 6x6 marker ID 0, 100 mm x 100 mm, in
  front of the user and visible to the head D435i.
- Arm-mounted marker: 6x6 marker ID 1, 100 mm x 100 mm, rigidly attached to
  the prosthetic arm with a fixed transform to the arm D435i.
- Optional extra marker: 6x6 marker ID 2, 100 mm x 100 mm, for calibration or
  redundancy experiments.

Important modeling boundary:
- Do not add the arm-mounted marker ID 1 as a fixed `marker_map` landmark in
  the current Phase 1 marker node. The current marker map assumes listed
  markers are stationary in the map. ID 1 moves with the arm and needs a
  separate dynamic-marker/multiview fusion path.

Future multiview fusion concept:
- Head D435i updates its pose from VIO and fixed marker ID 0.
- Arm D435i updates its pose from VIO and fixed marker ID 0 when visible.
- When the head D435i sees arm marker ID 1, use the head pose uncertainty,
  marker detection uncertainty, and calibrated `T_arm_camera_marker_1` to
  update or constrain the arm D435i pose.

## E. Arm Marker To Arm D435i Extrinsic Calibration

Create a calibration script for the fixed transform between arm-mounted marker
ID 1 and the arm D435i.

One possible calibration procedure:
- Place fixed marker ID 0 where both cameras can observe it.
- Move the prosthetic arm through multiple poses.
- Save frames when the head D435i sees marker ID 0 and marker ID 1, and the
  arm D435i sees marker ID 0.
- Estimate the rigid transform between marker ID 1 and the arm D435i from
  repeated simultaneous observations.
- Report residuals and uncertainty so the transform can be used in later
  multiview fusion.

## F. Arm D435i Trajectory Prediction With Uncertainty

Investigate a prediction node that runs on each arm D435i image or synchronized
state update and predicts the arm camera feature/pose trajectory over a short
horizon, for example one second.

The output should include:
- predicted poses along the horizon
- uncertainty for each predicted pose
- enough timing metadata to fuse predictions with marker/VIO updates

This is after Phase 1 covariance validation and likely after the first
two-camera fusion prototype.
