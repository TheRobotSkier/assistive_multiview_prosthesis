# Future Tasks

## Current Phase 2 Status

As of 2026-05-11, the Phase 2 head marker EKF path and arm D435i Phase 2
bringup have been committed and pushed. Latest local commit seen by Codex:

```text
374216c Add arm D435i Phase 2 OpenVINS bringup
```

The 100 mm marker covariance bag analysis has been completed. Phase 2
OpenVINS-internal marker update/reset code has been implemented locally and
passed the low-memory Docker/Jazzy build, launch/message smoke checks, live
head-camera testing, arm setup smoke checks, and pre-init raw replay validation
for both head and arm D435i cameras.

Validated Phase 2 head D435i replay bag:

```text
docker_ws/bags/openvins_tests/phase2_live/head_marker_phase2_100mm_preinit_20260510_122723
```

Fresh replay of only raw image/camera-info/IMU topics initialized OpenVINS,
published `poseimu`, `odomimu`, and `pathimu` in `marker_map`, produced `4`
marker-map resets and `1212` accepted marker-0 EKF updates, and did not reproduce
the live Propagator assertion.

Validated Phase 2 arm D435i replay bag:

```text
docker_ws/bags/openvins_tests/phase2_live/arm_marker_phase2_100mm_preinit_20260510_142201
```

Fresh replay of only raw arm image/camera-info/IMU topics initialized OpenVINS,
published `/ov_msckf_arm/poseimu`, `/ov_msckf_arm/odomimu`, and
`/ov_msckf_arm/pathimu` in `marker_map`, produced `3` marker-map resets and
`1664` accepted marker-0 EKF updates, and did not reproduce the live hard
Propagator assertion.

Use the current Phase 2 handoff note before continuing:
`.agents/phase2_openvins_handoff_status.md`.

Repository hygiene:
- generated `build_overlay/`, `install_overlay/`, and `log_overlay/` artifacts
  should stay out of commits
- ROS bags should stay local validation artifacts unless a separate data-sharing
  decision is made

Near-term next work:
- validate the per-instance OpenVINS TF frame fix in live dual-camera operation
  and with fresh head/arm raw replays
- add a defensive OpenVINS timing/crash guard around the Propagator assertion
  observed once during live testing

## A. Arm D435i Setup From Calibration Output

Calibration files are available at:
`docker_ws/calibration/arm/d435i_310622071850/`

Arm Phase 2 setup has been implemented by mirroring the head D435i
configuration where appropriate.

Planned values:
- Namespace: `arm`
- Camera name: `d435i_arm`
- RealSense serial: `_310622071850`
- Mount location: `prosthetic_arm`

Implemented deliverables:
- `multi_cam_localization/sensor_fusion_bringup/config/openvins/arm_d435i_310622071850/estimator_config.yaml`
- `multi_cam_localization/sensor_fusion_bringup/config/openvins/arm_d435i_310622071850/kalibr_imucam_chain.yaml`
- `multi_cam_localization/sensor_fusion_bringup/config/openvins/arm_d435i_310622071850/kalibr_imu_chain.yaml`
- `multi_cam_localization/sensor_fusion_bringup/config/markers/arm_aruco_map.yaml`
- `multi_cam_localization/sensor_fusion_bringup/launch/arm_d435i_openvins_phase2.launch.py`
- `multi_cam_localization/sensor_fusion_bringup/launch/arm_marker_pose_phase2.launch.py`
- `multi_cam_localization/sensor_fusion_bringup/docs/arm_phase2_openvins_marker_validation.md`
- `multi_cam_localization/sensor_fusion_bringup/docs/arm_phase2_openvins_live_trial_and_recording.md`

The arm OpenVINS node uses `/ov_msckf_arm` to avoid topic collisions with head
OpenVINS. The Phase 2 launch now uses per-instance OpenVINS frames so arm TF can
run beside head TF: `marker_map -> head_imu -> head_cam0` and
`marker_map -> arm_imu -> arm_cam0`.

## B. OpenVINS Internal Marker Update / EKF Reanchor

Phase 2 head D435i internal marker update/reanchor is implemented and validated.
Keep future edits here focused on behavior improvements found during head or arm
validation.

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

Known robustness follow-up:
- A live run once hit the OpenVINS `Propagator.cpp` assertion comparing requested
  propagation duration against summed IMU dt. The new validation bag did not
  reproduce it, and raw timestamps were monotonic, so this looks like a transient
  live sensor/timing disruption rather than a marker EKF bug. Consider replacing
  the hard assert with a logged guard/drop path after the arm setup is validated,
  or sooner if the crash recurs.

## C. VIO Health Monitor And Reset/Reinitialize Behavior

Improve logic that marks VIO invalid when covariance, velocity, workspace
bounds, or marker innovation become unreasonable. This can remain external
first, then potentially move into OpenVINS after the behavior is trusted.

## D. Two-Camera Point-Cloud Fusion

Use `marker_map` or another common reference frame to align head and arm D435i
point clouds. Avoid fusing clouds directly in drifting raw OpenVINS `global`
frames.

Immediate prerequisite:
- Validate the new per-instance OpenVINS TF frame support for multiple live
  instances. The intended TF tree exposes distinct head and arm IMU/camera
  frames while preserving the shared `marker_map`.

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
