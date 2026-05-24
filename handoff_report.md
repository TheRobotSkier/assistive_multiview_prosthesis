# OpenVINS Marker/IMU Fusion Handoff

## Source Branch

Work should be based on the Jetson/OpenVINS code, not the host-only branch:

```text
worktree: /home/asger/Drive/AAU/P8/grasping/multiview_prosthesis-jetson-openvins
branch: inspect/jetson-openvins
tracking: origin/jetson_docker
commit inspected: 68f0f89f2936e6a38211c30ddc7094d67c665c23
```

The host branch `pc-fusion-hourly` should only need changes if the Jetson work changes the current host-facing topic/frame contract.

## Parent Epic

The implementation plan is captured in beads under:

```text
mvp-cox - EPIC: Make Jetson OpenVINS less IMU-dominated
```

Use `bd show mvp-cox` and `bd show mvp-cox.<n>` for the full acceptance criteria and design notes.

## Child Beads

```text
mvp-cox.1  Design OpenVINS experiment profile interface
mvp-cox.2  Add fixed marker update diagnostics topic
mvp-cox.3  Implement strong fixed-marker EKF trust profile
mvp-cox.4  Make first marker-map lock easier and diagnosable
mvp-cox.5  Add configurable IMU bias handling on marker reset
mvp-cox.6  Add OpenVINS online calibration experiment profile
mvp-cox.7  Add conservative ZUPT experiment profile
mvp-cox.8  Investigate and implement IMU optical-frame convention variant
mvp-cox.9  Add tests for marker reset and profile behavior
mvp-cox.10 Run Jetson validation matrix for OpenVINS experiments
mvp-cox.11 Verify host-side compatibility for OpenVINS experiment profiles
mvp-cox.12 Document operator commands for OpenVINS profiles
mvp-cox.13 Compose all_changes OpenVINS experiment profile
```

## Native ArUco / Marker Graph Track

The beads database also includes a second epic under the same parent for the
user's proposed marker-chain synchronization architecture:

```text
mvp-cox.14   EPIC: Evaluate native OpenVINS ArUco marker-graph fusion
mvp-cox.14.1 Verify native OpenVINS ArUco semantics
mvp-cox.14.2 Add native ArUco landmark-only experiment profile
mvp-cox.14.3 Implement C++ ArUco pose frontend compatible with MarkerPoseObservation
mvp-cox.14.4 Implement multi-marker graph estimator for head/arm camera sync
mvp-cox.14.5 Feed marker graph constraints back into OpenVINS or TF outputs
mvp-cox.14.6 Test and validate native ArUco marker graph path
```

Important architectural point: simply setting `use_aruco: true` is not assumed
to solve camera synchronization. The current native OpenVINS ArUco path appears
to treat marker corners as SLAM landmarks inside each independent OpenVINS
instance. That can help or hurt VIO, and should be tested as
`mvp-cox.14.2`, but it does not by itself build a shared marker graph or infer
`cam1 -> marker1 -> marker2 -> marker3 -> cam2`.

The marker-chain idea makes sense as a separate graph estimator. It needs
camera-to-marker observations with IDs and covariance, then it can add
marker-to-marker edges whenever a camera sees multiple markers in one frame and
query transforms through connected components. The graph should be able to
consume either the current Python observations or a future C++ ArUco pose
frontend, so implementation can proceed without removing the Python path first.

## Suggested Delegation Order

Start with `mvp-cox.1`. It defines the profile/launch interface that every other implementation task should use.

Then these can proceed mostly in parallel:

```text
mvp-cox.2 diagnostics
mvp-cox.3 strong fixed-marker EKF profile
mvp-cox.4 easier first marker lock
mvp-cox.5 marker reset bias policy
mvp-cox.6 online calibration profile
mvp-cox.7 ZUPT profile
mvp-cox.8 IMU frame/convention variant
```

For the native-ArUco/marker-graph track, start with:

```text
mvp-cox.14.1 native OpenVINS ArUco semantics investigation
```

Then `mvp-cox.14.2`, `mvp-cox.14.3`, and `mvp-cox.14.4` can proceed in
parallel once the semantics are clear. `mvp-cox.14.5` depends on the graph, and
`mvp-cox.14.6` is the validation gate.

After those land:

```text
mvp-cox.13 compose all_changes profile
mvp-cox.9 add tests
mvp-cox.10 run Jetson validation matrix
mvp-cox.11 verify host compatibility
mvp-cox.12 write operator runbook
```

## Current Code Architecture

The live Jetson setup uses:

```text
RealSense D435i color + combined IMU
-> ov_msckf/run_subscribe_msckf_marker
-> /ov_msckf/odomimu and /ov_msckf_arm/odomimu
```

Fixed ArUco marker ID0 uses a separate path:

```text
aruco_marker_pose_node.py
-> /head/marker_pose/observation or /arm/marker_pose/observation
-> ROS2Visualizer marker queue
-> VioManager::feed_measurement_marker()
-> UpdaterMarkerPose EKF update or VioManager::reset_to_marker_map()
```

Built-in OpenVINS ArUco feature tracking is disabled in the live configs:

```text
use_aruco: false
```

So marker ID0 is not merely a high-quality visual feature. It is an external absolute pose measurement. The problem is that it is still a gated correction inside an always-IMU-propagated VIO estimator.

## Main Failure Hypothesis

The current implementation always feeds IMU samples:

```text
ROS2Visualizer::callback_inertial()
  -> _app->feed_measurement_imu(message)
  -> visualize_odometry(message.timestamp)
```

Marker observations are only processed after camera updates and only if they pass timing, fixed-ID, stability, frame, covariance, chi-square, and reset gates.

This can produce the observed behavior:

```text
marker visible
-> Python detector publishes observation
-> OpenVINS drops/rejects/weakly applies it
-> IMU propagation remains dominant
-> odom continues drifting
```

The most suspicious technical issue is still IMU frame/calibration convention:

```text
ROS IMU wm/am copied directly into OpenVINS
kalibr_imu_chain has identity R_IMUtoACC/R_IMUtoGYRO
kalibr_imucam_chain has near-identity T_cam_imu
online calibration is disabled
```

Do not introduce undocumented sign flips. If `mvp-cox.8` implements an IMU-frame variant, document exact frame direction and validation evidence.

## Compatibility Target

Keep these host-facing interfaces stable unless a bead explicitly proves otherwise:

```text
/ov_msckf/odomimu
/ov_msckf_arm/odomimu
/head/marker_pose/observation
/arm/marker_pose/observation
/ov_msckf/marker_map_locked
/ov_msckf_arm/marker_map_locked
marker_map
head_imu
arm_imu
head_cam0
arm_cam0
```

The host `pc-fusion-hourly` side currently relays OpenVINS odom/TF. It does not perform marker correction itself.

## Jetson Access Rule

All Jetson access must go through the project queue. Do not run bare `ssh`, `scp`, `rsync`, or Jetson make targets directly.

For commands that SSH or start containers, wrap with:

```text
scripts/jetson_run.sh <command>
```

Use:

```text
queue_name="jetson"
timeout_seconds=300
```

Use shorter timeouts for quick status checks and data sync.
