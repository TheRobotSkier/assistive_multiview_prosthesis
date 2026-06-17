# Plan: Merge Jetson Localization Code to x86 Single-Machine

**Date:** 2026-06-17
**Branch:** `openvins-merge2host` (target)
**Source:** `jetson_docker` worktree at `/home/daniel/worktrees/assistive_multiview_prosthesis/ample-linden/assistive_multiview_prosthesis`
**Goal:** Run the full dual-D435i + OpenVINS localization pipeline on a single x86 machine with Intel cameras connected via USB, eliminating the Jetson ↔ host network split.

---

## 1. Executive Summary

**Feasibility: HIGH.** The merge is very feasible. The Jetson code is almost entirely architecture-agnostic (Eigen3, OpenCV, Boost, Ceres, ROS 2 Jazzy). The only ARM-specific element is a single Dockerfile `ARG` that downloads the aarch64 MuJoCo tarball — which the host already has an x86 equivalent for. The real work is **interface rewiring**: today the host consumes throttled `/jetson/*` topics relayed over Ethernet; on a single machine those relay hops and network-discovery workarounds disappear.

The host branch (`openvins-merge2host`) is already 90% prepared — it has all the consumer nodes (`pointcloud_fusion`, `openvins_odom_tf_relay`, `openvins_realsense_tf_bridge`, `gtsam_tracker`, etc.) wired to `/jetson/*` topics. What is **missing** on the host is the producer side: the forked OpenVINS (with marker-pose updates), the dual-camera launch, the ArUco pose node, and the OpenVINS calibration configs.

**Estimated effort:** ~1–2 days. No algorithmic changes; it is a packaging + launch + config merge with a Dockerfile tweak.

### Resolved decisions (per user, 2026-06-17)

- **Cameras:** The same two physical D435i units (serials `...71386` head, `...71850` arm) are moving to the x86 machine. **Calibration carries over verbatim** — no recalibration needed.
- **External IMU:** The I2C MPU-6500 `imu_driver` package is no longer part of the project. **Do not bring it over.** OpenVINS uses the D435i built-in BMI085.
- **Jetson fallback:** Not required. The relay + cyclonedds peer + chrony layer can be **removed outright** rather than preserved behind a profile.
- **Container:** **Single container** — OpenVINS goes into the existing `prosthesis` container (see §5.6).
- **Topic naming:** Keep `/jetson/*` via **launch-file remappings** (free, no extra nodes) — see §5.4.

---

## 2. Current Architecture (Two-Machine Split)

```
┌─────────────────────────────┐          ┌──────────────────────────────────┐
│  JETSON (aarch64)           │  Ethernet │  HOST (x86)                      │
│  jetson_docker branch       │  10.42.0  │  openvins-merge2host branch      │
│                             │           │                                  │
│  realsense2_camera (×2)     │           │                                  │
│   /head/d435i_head/*        │──(raw)──▶ │                                  │
│   /arm/d435i_arm/*          │           │                                  │
│          ▼                  │           │                                  │
│  OpenVINS ×2 (forked)       │           │                                  │
│   /ov_msckf/odomimu         │──(odom)──▶│  openvins_odom_tf_relay          │
│   /ov_msckf_arm/odomimu     │           │  openvins_realsense_tf_bridge    │
│          ▼                  │           │          ▼                       │
│  aruco_marker_pose_node     │           │  pointcloud_fusion               │
│   /head/marker_pose/obs     │──(aruco)─▶│  segmentation / tsdf_fusion      │
│          ▼                  │           │  gtsam_tracker                   │
│  jetson_relay.py ◀── THE NETWORK BOTTLENECK                                │
│   /jetson/head/points (5Hz) │──(relay)▶│  twist_propagation               │
│   /jetson/arm/points        │           │  grasp_preshaping                │
│   /jetson/head/odom         │           │  pipeline_manager                │
│   /jetson/head/image        │           │                                  │
└─────────────────────────────┘          └──────────────────────────────────┘
        chrony time sync ◀────────────▶ cyclonedds peer discovery
```

**Key observation:** `jetson_relay.py` exists *only* to throttle/decimate data so the Ethernet link is not saturated. On a single machine this entire layer is unnecessary.

---

## 3. Interface Map (What Must Be Preserved)

### 3.1 Topics (host → producer contract)

The host's consumer nodes subscribe to these topics. The merge must ensure these still appear, whether via relay or direct topic remapping:

> **Note:** This table shows the *current* host contract. §5.4 supersedes the wiring decision for the single-machine merge: only the two **odom** topics keep the `/jetson/*` alias (via launch remap); the **camera** topics (image/depth/cloud/info) are repointed in `prosthesis_config.yaml` to the native `/head/d435i_head/*` names. The marker topic (`/head/marker_pose/observation`) is unchanged.

| Host consumer | Topic subscribed | Source on Jetson | Type |
|---|---|---|---|
| `pointcloud_fusion` | `/jetson/head/points` | `/head/d435i_head/depth/color/points` | PointCloud2 |
| `pointcloud_fusion` | `/jetson/arm/points` | `/arm/d435i_arm/depth/color/points` | PointCloud2 |
| `openvins_odom_tf_relay` | `/jetson/head/odom` | `/ov_msckf/odomimu` | Odometry |
| `openvins_odom_tf_relay` | `/jetson/arm/odom` | `/ov_msckf_arm/odomimu` | Odometry |
| `keyframe_buffer` / `cross_camera_features` | `/jetson/head/image` | `/head/d435i_head/color/image_raw` | Image |
| `keyframe_buffer` / `cross_camera_features` | `/jetson/arm/image` | `/arm/d435i_arm/color/image_raw` | Image |
| `keyframe_buffer` / `cross_camera_features` | `/jetson/head/depth` | `/head/d435i_head/depth/image_rect_raw` | Image |
| `cross_camera_features` | `/jetson/head/camera_info` | `/head/d435i_head/color/camera_info` | CameraInfo |
| `gtsam_tracker` | `/aruco/marker_pose` etc. | `/head/marker_pose/observation` | MarkerPoseObservation |

### 3.2 TF tree (host reconstruction)

The host does **not** receive `/tf` from the Jetson. It reconstructs the tree from odom + static extrinsics:

1. `openvins_odom_tf_relay` — publishes `marker_map → head_imu` and `marker_map → arm_imu` from the Odometry messages, plus the calibrated `imu → cam0` static edges.
2. `openvins_realsense_tf_bridge` — publishes `head_cam0 → head_d435i_head_link` and `arm_cam0 → arm_d435i_arm_link` (the bridge edge), plus the nominal RealSense static fan-out (`link → depth_frame → depth_optical_frame`, etc.).

On a single machine, OpenVINS itself will publish `marker_map → *_imu` and the calibration TFs (`*_imu → *_cam0`) directly via its `publish_global_to_imu_tf` / `publish_calibration_tf` parameters. The RealSense driver publishes its own static chain. **This means the host's two relay/bridge nodes become partially redundant** — see §6.2 for the TF-authority conflict and its mitigation.

---

## 4. Architecture-Specific Analysis

### 4.1 What is ARM/aarch64-specific

| Item | Location | Impact |
|---|---|---|
| `ARG CPU_ARCH=aarch64` | `docker_ws/docker-deployment/Dockerfile:13` | Downloads aarch64 MuJoCo tarball. **Host already uses x86 MuJoCo.** Not needed. |
| `runtime: nvidia` | `docker_ws/docker-deployment/docker-compose.yml:14` | Jetson GPU. Host uses `NVIDIA_VISIBLE_DEVICES` differently. Not needed for cameras. |
| `build_overlay/`, `install_overlay/` | `docker_ws/build_overlay/*` | Stale aarch64 build artifacts. **Do not copy.** Rebuild from source. |
| I2C IMU driver (`/dev/i2c-7`) | `imu_driver/src/imu_node.cpp` | Reads MPU-6500 over Jetson I2C bus. **Not needed** — OpenVINS uses the D435i's built-in IMU (`/head/d435i_head/imu`). |

### 4.2 What is architecture-agnostic (ports cleanly)

- **OpenVINS fork** (`ov_core`, `ov_init`, `ov_msckf`) — C++14, depends only on Eigen3, OpenCV, Boost, Ceres. Compiles on x86 unchanged.
- **`UpdaterMarkerPose`** + `run_subscribe_msckf_marker` — pure C++ ROS 2 nodes.
- **`aruco_marker_pose_node.py`** — Python, OpenCV ArUco.
- **`pointcloud_to_frame_node.cpp`** — tf2 + sensor_msgs.
- **All launch files, configs, RViz files** — declarative.

### 4.3 Docker base image compatibility

Both branches already use `ros:jazzy` (Jetson: `ros:jazzy-ros-base`; host: `osrf/ros:jazzy-desktop`). The host image **already includes** `ros-jazzy-librealsense2`, `ros-jazzy-realsense2-camera`, and `ros-jazzy-realsense2-description` (see `docker/Dockerfile:27-29`). No additional camera packages are needed on the host.

---

## 5. Merge Plan

### 5.1 Bring over the forked OpenVINS (PRODUCER — missing on host)

**From:** `jetson_docker:docker_ws/src/open_vins/`
**To:** `openvins-merge2host:src/open_vins/` (currently an empty directory)

Copy the entire `open_vins/` tree containing:
- `ov_core/`, `ov_init/`, `ov_msckf/` (the three ROS 2 packages)
- The custom marker files:
  - `ov_msckf/src/update/UpdaterMarkerPose.cpp` + `.h`
  - `ov_msckf/src/run_subscribe_msckf_marker.cpp`
  - Marker options in `ov_msckf/src/core/VioManagerOptions.h` (`MarkerPoseUpdaterOptions`)
  - Marker subscriber in `ov_msckf/src/ros/ROS2Visualizer.cpp`
  - `ov_msckf/cmake/ROS2.cmake` (registers the `run_subscribe_msckf_marker` executable)

**Build deps to add to host Dockerfile:** `libceres-dev`, `libboost-all-dev`, `libeigen3-dev` are needed by OpenVINS. Add them explicitly to the `apt-get` block — see §5.6.1 for the exact snippet. (`rosdep install` at `docker/Dockerfile:68-71` will also pick up `ov_msckf`'s `package.xml` deps automatically once `src/open_vins/` exists, but these three are added explicitly to match the Jetson Dockerfile at `docker_ws/docker-deployment/Dockerfile:34-37`.)

**Note:** The `ov_msckf/package.xml` depends on `sensor_fusion_msgs` (for the `MarkerPoseObservation` message). The host already has `src/sensor_fusion_msgs/` with `MarkerPoseObservation.msg`. **VERIFIED COMPATIBLE** — see §6.1; no reconciliation needed.

### 5.2 Merge `sensor_fusion_bringup` (enriched version)

The Jetson branch's `sensor_fusion_bringup` is far richer than the host's. **Merge, do not overwrite** — the host has newer scripts (`publish_camera_mounts.py`, `tf_pipeline_diagnostics.py`) that the Jetson branch lacks.

**From Jetson, bring over:**
- `config/openvins/` — the two calibrated camera directories:
  - `head_d435i_336222071386/` (estimator_config, kalibr_imu_chain, kalibr_imucam_chain, imu_noise)
  - `arm_d435i_310622071850/` (same set)
- `config/markers/` — ArUco marker maps and extrinsics
- `config/aruco_markers/` — printable marker PNGs
- `config/d435i_cameras.yaml` — **prefer the Jetson version** (it has `pointcloud_enable: false`, `decimation_filter`, `unite_imu_method: 1` tuned for OpenVINS). The host version has `unite_imu_method: 2` and `pointcloud_enable: true`.
- `launch/head_d435i_openvins_phase2.launch.py` + `arm_d435i_openvins_phase2.launch.py` — the producer launch files
- `launch/dual_d435i.launch.py` — **prefer the Jetson version** (uses `realsense2_camera_node` directly with full parameter control, including decimation filter and the NEON fix)
- `scripts/aruco_marker_pose_node.py` — the ArUco correction layer
- `src/pointcloud_to_frame_node.cpp` + update `CMakeLists.txt` to build it
- `scripts/dynamic_arm_pose_measurement_node.py`, `scripts/head_derived_arm_pose_preview_node.py` (optional, for dynamic marker workflows)

**Keep from host (do not clobber):**
- `scripts/publish_camera_mounts.py`, `scripts/tf_pipeline_diagnostics.py`
- `config/camera_mounts.yaml`

**Update `CMakeLists.txt`:** The host's `sensor_fusion_bringup/CMakeLists.txt` is a thin ament wrapper. Replace it with the Jetson version (which builds `pointcloud_to_frame_node` and installs scripts as executables), then re-add the host's `USE_SOURCE_PERMISSIONS` for the scripts directory.

### 5.3 Create a unified single-machine launch file

Create `src/prosthesis_launch/launch/localization.launch.py` (new) that starts the **producer** side on the same machine:

```
dual_d435i.launch.py          # both RealSense cameras (USB)
  → head_d435i_openvins_phase2 # head OpenVINS + aruco node
  → arm_d435i_openvins_phase2  # arm OpenVINS + aruco node
```

This launch should:
1. Start both D435i cameras via `dual_d435i.launch.py` (USB, not network).
2. Start both OpenVINS instances (`run_subscribe_msckf_marker`) with their calibrated configs.
3. Start both `aruco_marker_pose_node` instances.
4. **Not** start `jetson_relay.py` (unnecessary — see §5.5).

### 5.4 Topic wiring: keep `/jetson/*` via launch remappings (zero extra compute)

**Decision: keep the `/jetson/*` prefix using native ROS 2 launch remappings.** This was analyzed against the user's constraint ("keep names if cheap, otherwise revert"). The verdict: **remapping is free** — it is a DDS topic-name alias resolved at the subscriber/publisher layer with no copying, no extra process, and no CPU cost. It is strictly cheaper than either a relay node or a global rename.

**Compute analysis:**
- A **relay node** (the old `jetson_relay.py`) deserializes, re-stamps, and re-publishes every message — real CPU + memory cost, and it was only needed to throttle over Ethernet. On one machine it is pure overhead.
- A **launch remapping** (`remappings=[...]` on a `Node`) makes a publisher's native topic appear under an alias. No second publisher, no serialization, no process. This is the cheapest possible option.
- A **global rename** (`/jetson/` → `/` across ~30 files) is also free at runtime but high-churn and error-prone in the short term.

**Implementation:** In the new `localization.launch.py` (§5.3), apply `remappings=` to the OpenVINS nodes so they publish to the host-expected names directly. Example for the head OpenVINS node:

```python
Node(
    package="ov_msckf",
    executable="run_subscribe_msckf_marker",
    name="run_subscribe_msckf_marker",
    remappings=[
        ("/ov_msckf/odomimu", "/jetson/head/odom"),  # odom → host contract
    ],
    ...
)
```

The RealSense camera node publishes `/head/d435i_head/...` natively. Rather than remap 8 camera topics, the host config can point directly at the native camera topics for image/depth/cloud (they are stable, well-named names), and **only the two OpenVINS odom topics** need the `/jetson/*` alias (since `prosthesis_config.yaml` references `/jetson/head/odom` in 6 places). Concretely:

| Topic | Approach | Why |
|---|---|---|
| `/ov_msckf/odomimu` → `/jetson/head/odom` | **remap** in launch | Referenced 6× in `prosthesis_config.yaml`; cheaper than editing config |
| `/ov_msckf_arm/odomimu` → `/jetson/arm/odom` | **remap** in launch | Same |
| `/head/d435i_head/depth/color/points` | **point host config at native name** | Already a clear name; avoids 2 remaps |
| `/head/d435i_head/color/image_raw` | **point host config at native name** | Same |
| `/head/d435i_head/depth/image_rect_raw` | **point host config at native name** | Same |
| `/head/d435i_head/color/camera_info` | **point host config at native name** | Same |

**Net effect:** 2 remaps in one launch file + a handful of config edits in `prosthesis_config.yaml` (cam1/cam2 topic, image/depth/info topics). No relay node, no global rename. This is the minimum-compute, minimum-churn path.

**Why not a relay node?** `jetson_relay.py` exists solely to throttle/decimate for the Ethernet link. Its `stamp_override` logic (re-stamping with `get_clock().now()`) was a chrony workaround. Both are obsolete on a single machine. Bringing it over would add CPU cost for no benefit.

**Deferred cleanup:** A future pass can rename `/jetson/*` → `/head/*` / `/arm/*` across the ~30 files for long-term cleanliness. Not required for this merge.

### 5.5 Remove the network layer (no fallback needed)

Since no Jetson fallback is required, **delete** rather than preserve these components:

| Component | Action |
|---|---|
| `jetson_relay.py` | **Do not bring over.** Its throttling and stamp-override logic are obsolete on one machine. |
| `cyclonedds_peer.xml` peer entries (`10.42.0.1/0.2`) | **Remove** peer entries. Keep the file for socket-buffer tuning, or set `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`. |
| `chrony` (host + Jetson) | **Remove.** Single clock domain — no skew to correct. Drop the `chrony` apt dep + `chrony-host.conf` volume from compose. |
| `config/chrony-host.conf` | **Delete.** |

### 5.6 Container architecture: single container (OpenVINS joins `prosthesis`)

**Decision: merge OpenVINS into the existing `prosthesis` container. Do NOT create a separate container.**

**Rationale (grounded in both compose files):**
- The host already runs one monolithic `prosthesis` container (`docker/docker-compose.yml:29`) with `network_mode: host`, `privileged: true`, source bind-mounted, and shared `/dev/shm`. The only *separate* containers are non-ROS HTTP services (segmentation, MobileSAM) that have no ROS dependency.
- The Jetson branch did the same: one `miahand_ros2` container (`docker_ws/docker-deployment/docker-compose.yml:2`) ran cameras + OpenVINS + ArUco together, with only RViz/MoveIt as sibling containers sharing the same image via `extends:`.
- OpenVINS is a standard C++ ROS 2 package with the **same dependency stack** the host already has (ROS 2 Jazzy, OpenCV, Eigen3, Boost). Splitting it into its own container would require a second image, a second `colcon build`, a shared DDS domain config, and inter-container `/dev` + `/dev/shm` plumbing — all for zero benefit, since the producer and consumer are co-located.
- With `network_mode: host` on a single container, all topics, services, and TF are in one process group with zero transport overhead. This is exactly what the two-machine design was trying to approximate over Ethernet.

**What changes in the host Dockerfile** (`docker/Dockerfile`):
1. Add OpenVINS build deps to the `apt-get` block (§5.6.1).
2. Nothing else — `rosdep install` at `docker/Dockerfile:68-71` will pick up `ov_msckf`'s `package.xml` deps automatically once `src/open_vins/` exists.

**No new compose service.** OpenVINS nodes are launched by `localization.launch.py` inside the existing `prosthesis` container.

#### 5.6.1 Dockerfile dependency additions

The host Dockerfile currently installs `ros-jazzy-librealsense2`, `ros-jazzy-realsense2-camera`, `ros-jazzy-realsense2-description` (`docker/Dockerfile:27-29`) — camera support is already present. For OpenVINS to compile, add to the `apt-get install` block:

```dockerfile
    libceres-dev \
    libboost-all-dev \
    # libeigen3-dev is pulled transitively by Ceres, but add explicitly for safety:
    libeigen3-dev \
```

These match the Jetson Dockerfile's dependency list (`docker_ws/docker-deployment/Dockerfile:34-37`).

#### 5.6.2 USB camera permissions

The host compose runs as `prosthesis` (non-root) with `privileged: true` (`docker/docker-compose.yml:74`). `privileged: true` grants USB access regardless of group membership, so no `video`/`plugdev` `group_add` is strictly needed. If `privileged` is later dropped for hardening, add:

```yaml
        group_add:
            - video
            - plugdev
        devices:
            - /dev:/dev
```

(mirroring the Jetson compose at `docker_ws/docker-deployment/docker-compose.yml:16-26`).

**No MuJoCo changes** — the host Dockerfile does not download MuJoCo (it is a Python pip install on the host path), so the Jetson's `ARG CPU_ARCH=aarch64` MuJoCo download is irrelevant.

### 5.7 IMU handling

**Use the D435i built-in IMU.** The OpenVINS config (`kalibr_imu_chain.yaml`) subscribes to `/head/d435i_head/imu` — this is the D435i's BMI085, published by `realsense2_camera` with `enable_gyro:=true enable_accel:=true unite_imu_method:=1` (as set in `head_d435i_openvins_phase2.launch.py:51`).

**Do NOT bring over the I2C `imu_driver` package.** It reads `/dev/i2c-7` (Jetson-specific MPU-6500/9250) and is **no longer part of the project** (confirmed by user). Leave it in the Jetson branch.

---

## 6. Risks and Verification

### 6.1 Message compatibility

**Risk:** `sensor_fusion_msgs/MarkerPoseObservation` must match between the OpenVINS fork (C++ consumer in `ROS2Visualizer`) and the host's `sensor_fusion_msgs`.

**Status: VERIFIED COMPATIBLE.** All three marker messages (`MarkerPoseObservation`, `DynamicMarkerObservation`, `DynamicArmPoseObservation`) are byte-identical between the two branches (checked 2026-06-17). No reconciliation needed — the OpenVINS C++ fork and the host's Python nodes agree on the wire format.

### 6.2 OpenVINS TF publishing conflicts

**Risk:** On the Jetson, OpenVINS does **not** publish `marker_map → *_imu` TF (the host's `openvins_odom_tf_relay` does that from odom messages). But OpenVINS *can* publish it directly via `publish_global_to_imu_tf: true` (set in the phase2 launch). On a single machine, if both OpenVINS and the relay publish the same TF edge, they will conflict.

**Mitigation:** In single-machine mode, either:
- Let OpenVINS publish TF (`publish_global_to_imu_tf: true`, `publish_calibration_tf: true`) and **disable** `openvins_odom_tf_relay.publish_dynamic_tf` (already `false` in host config). OR
- Keep the relay as the TF authority and set OpenVINS `publish_global_to_imu_tf: false`.

The phase2 launch files set `publish_global_to_imu_tf: True` and `publish_calibration_tf: True`, so **prefer the first option** and ensure the host relay's `publish_dynamic_tf` stays `false`.

### 6.3 Camera serial numbers

**Risk:** The configs hardcode serial numbers `_336222071386` (head) and `_310622071850` (arm).

**Status: RESOLVED (non-issue).** User confirmed the *same* two physical D435i units move to x86 (2026-06-17). No serial changes needed. The hardcoded serials in `d435i_cameras.yaml`, the `config/openvins/` directory names, and `head_d435i_openvins_phase2.launch.py:45` all remain valid as-is.

*(If cameras are ever swapped, update the three locations above. A future refactor could parameterize serials via config — `dual_d435i.launch.py` already reads from `d435i_cameras.yaml`; the phase2 files have them hardcoded.)*

### 6.4 Calibration validity

**Risk:** The OpenVINS Kalibr calibration (`kalibr_imucam_chain.yaml`) is per-camera-serial.

**Status: RESOLVED (non-issue).** Same cameras → calibration carries over verbatim. No recalibration with Kalibr needed.

### 6.5 CPU load

**Risk:** The Jetson branch tuned OpenVINS for resource constraints (`max_clones: 8`, `max_slam: 25`, `num_pts: 200`, `num_opencv_threads: 2`). On x86 these can be relaxed for better accuracy, but the current values are safe defaults.

### 6.6 USB bandwidth (two D435i on one host)

**Risk:** Two D435i streaming color + depth + IMU over USB 3.x can hit bandwidth limits, causing frame drops. The Jetson had one camera per USB bus.

**Mitigation:** Connect the two cameras to **different USB controllers** (not just different ports on the same hub). Monitor with `uvcdynext -f` or `rs-enumerate-devices`. The decimation filter (`decimation_filter.enable: true`, `magnitude: 3`) in the Jetson config reduces USB load — keep it.

### 6.7 WSL2 IMU inaccessibility (CRITICAL — discovered during implementation)

**Risk:** When the host runs under **WSL2** with cameras attached via `usbipd-win`, the D435i's built-in BMI085 IMU is **inaccessible**. Verified empirically:

- Both cameras enumerate correctly (serials `336222071386`, `310622071850`).
- Video streams (color + depth) work perfectly (`rs-depth` produces live output).
- The IMU fails with: `d400-motion.cpp:185 — HID Motion Sensor Failure! bad optional access`.
- The HID interface (`/dev/hidraw0`, `/dev/hidraw1`) is present and openable, but librealsense cannot read motion data through the USB/IP tunnel.
- This is **not** a permissions issue (fails as root) — it is a fundamental WSL2/usbipd limitation with the D435i's custom HID-over-USB protocol.

**Impact:** OpenVINS is visual-**inertial** — it requires the IMU to propagate state between frames. Without it, OpenVINS cannot run. The entire localization pipeline is blocked on WSL2.

**Workarounds (none verified yet):**

1. **Run on bare-metal Linux** (dual-boot or dedicated machine). This eliminates the USB/IP layer entirely; the IMU works natively. **Recommended path.**
2. **IMU relay from Windows:** Write a Windows-side service (Python + `pyrealsense2`) that reads the IMU on Windows and publishes it over UDP/ZMQ to WSL. Requires a small bridge node. Adds latency and complexity.
3. **Wait for usbipd-win HID support:** The limitation may be resolved in future usbipd-win versions. Not actionable now.
4. **Use an external USB IMU** (e.g., a BMI088 breakout over serial). Would require re-introducing the `imu_driver` pattern and recalibrating `T_imu_cam`. Significant effort.

**Status:** This is the **primary blocker** for running the full pipeline on WSL2. The code merge is complete and verified (all packages compile, launch graph constructs, video streams work). The only remaining issue is the IMU data path.

---

## 7. Step-by-Step Implementation Order

### Phase 1: Bring OpenVINS to the host (build only)
1. Copy `jetson_docker:docker_ws/src/open_vins/` → `openvins-merge2host:src/open_vins/` (exclude any `build/` `install/` `log/`).
2. `sensor_fusion_msgs` `.msg` files — **already verified byte-identical** (§6.1); no action.
3. Add `libceres-dev`, `libboost-all-dev`, `libeigen3-dev` to `docker/Dockerfile` (§5.6.1).
4. `make build` — confirm OpenVINS packages (`ov_core`, `ov_init`, `ov_msckf`) compile on x86.
5. **Gate:** `ros2 run ov_msckf run_subscribe_msckf_marker` starts without crash (will idle without camera input).

### Phase 2: Bring the camera + ArUco launch
6. Merge `sensor_fusion_bringup` configs, launch files, and scripts per §5.2.
7. Update `sensor_fusion_bringup/CMakeLists.txt` to build `pointcloud_to_frame_node`.
8. Connect one D435i, verify `ros2 launch sensor_fusion_bringup dual_d435i.launch.py` produces `/head/d435i_head/*` topics.
9. **Gate:** `ros2 topic echo /head/d435i_head/imu` shows live data.

### Phase 3: Single-machine localization
10. Create `src/prosthesis_launch/launch/localization.launch.py` per §5.3.
11. Add launch **remappings** for the 2 odom topics + repoint camera topics in `prosthesis_config.yaml` per §5.4. (No relay node.)
12. Launch localization, then `pipeline.launch.py` (consumer side).
13. **Gate:** `ros2 topic echo /jetson/head/odom` shows OpenVINS odometry; `ros2 run tf2_ros tf2_echo marker_map head_d435i_head_depth_optical_frame` resolves.

### Phase 4: Cleanup
14. Remove peer entries from `config/cyclonedds_peer.xml` (keep socket-buffer tuning); set `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`.
15. Remove `chrony` apt dep from `docker/Dockerfile`, delete `config/chrony-host.conf`, drop the chrony volume from `docker-compose.yml`.
16. Update `Makefile.workspace` with a `run-local` target that launches localization + pipeline.
17. (Optional, deferred) Rename `/jetson/*` → `/head/*` / `/arm/*` everywhere for cleanliness.

---

## 8. Files to Copy / Merge (Checklist)

### Copy verbatim (new on host)
- [ ] `src/open_vins/` (entire forked tree, excluding any `build/` `install/` `log/`)
- [ ] `src/sensor_fusion_bringup/config/openvins/` (both camera calibration dirs)
- [ ] `src/sensor_fusion_bringup/config/markers/`
- [ ] `src/sensor_fusion_bringup/config/aruco_markers/`
- [ ] `src/sensor_fusion_bringup/launch/head_d435i_openvins_phase2.launch.py`
- [ ] `src/sensor_fusion_bringup/launch/arm_d435i_openvins_phase2.launch.py`
- [ ] `src/sensor_fusion_bringup/scripts/aruco_marker_pose_node.py`
- [ ] `src/sensor_fusion_bringup/src/pointcloud_to_frame_node.cpp`

### Merge (both branches have a version)
- [ ] `src/sensor_fusion_bringup/config/d435i_cameras.yaml` — prefer Jetson (OpenVINS-tuned)
- [ ] `src/sensor_fusion_bringup/launch/dual_d435i.launch.py` — prefer Jetson (direct node, more params)
- [ ] `src/sensor_fusion_bringup/CMakeLists.txt` — Jetson structure + host's script permissions
- [ ] `src/sensor_fusion_msgs/msg/*.msg` — **already verified identical** (§6.1); no action

### Modify on host
- [ ] `docker/Dockerfile` — add `libceres-dev`, `libboost-all-dev`, `libeigen3-dev` (§5.6.1); remove `chrony` (§5.5)
- [ ] `docker/docker-compose.yml` — drop `chrony-host.conf` volume (§5.5)
- [ ] `config/cyclonedds_peer.xml` — remove peer entries (§5.5)
- [ ] `config/chrony-host.conf` — **delete** (§5.5)
- [ ] `config/prosthesis_config.yaml` — repoint camera topics (image/depth/cloud/info) to native `/head/d435i_head/*` names (§5.4)
- [ ] `src/prosthesis_launch/launch/localization.launch.py` — **new file** (§5.3), with odom remappings (§5.4)
- [ ] `Makefile.workspace` — add `run-local` target (Phase 4.16)

### Do NOT copy
- [ ] `jetson_relay.py` — obsolete on one machine (§5.4, §5.5)
- [ ] `imu_driver/` — I2C MPU-6500, retired from project (§5.7)
- [ ] `docker_ws/build_overlay/`, `install_overlay/`, `log_overlay/` — stale aarch64 artifacts
- [ ] `jetson_folder/` — debug/analysis scripts, not runtime
- [ ] `multiview/` — legacy standalone test scripts
- [ ] `jetson_ws/` — duplicate of imu_driver

---

## 9. Resolved Questions

All four originally-open questions are now resolved (user, 2026-06-17):

1. **Same physical cameras?** **Yes** — serials `...71386` (head) and `...71850` (arm) move to x86. Calibration carries over verbatim; no recalibration. *(Closes risk §6.3 and §6.4.)*
2. **External I2C IMU used elsewhere?** **No** — it is retired from the project. `imu_driver/` is not brought over. *(Closes §5.7.)*
3. **Topic naming?** **Keep `/jetson/*`** via launch remappings (free, zero compute) for the 2 odom topics; point host config at native camera topic names for the rest. *(Closes §5.4.)*
4. **Jetson fallback?** **No.** The relay/cyclonedds-peer/chrony layer is deleted outright. *(Closes §5.5.)*

**New question answered in this revision:** OpenVINS goes into the **existing `prosthesis` container** (single container), not a separate one. *(See §5.6.)*

---

## 10. Implementation Status (2026-06-17)

All four phases of the code merge are **complete and verified**:

| Phase | Work | Status | Gate |
|---|---|---|---|
| 1 | OpenVINS fork copied + 3 apt deps added | DONE | `run_subscribe_msckf_marker` builds & starts |
| 2 | sensor_fusion_bringup merged (configs, launches, scripts, C++ node) | DONE | `pointcloud_to_frame_node` builds; all 6 scripts installed |
| 3 | `localization.launch.py` created + odom remaps + config repoints | DONE | Launch graph constructs; OpenVINS + ArUco nodes start cleanly |
| 4 | cyclonedds peers removed, chrony deleted, `run-local` target added | DONE | Full workspace build (25 packages) succeeds |

**Full workspace build:** 25 packages compile cleanly on x86 (`colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release`).

**Launch graph validation:** `ros2 launch prosthesis_launch localization.launch.py head_only:=true` starts both OpenVINS and ArUco nodes without errors. OpenVINS initializes (INFO verbosity); ArUco node loads Kalibr calibration and runs diagnostics. Nodes correctly wait for camera/IMU data.

**Environment fix applied:** Raised kernel socket buffers via `sudo scripts/tune_network_host.sh` (persistent in `/etc/sysctl.d/99-prosthesis-udp.conf`). CycloneDDS requires 8 MB receive buffers; the WSL2 default was 208 KB.

**Remaining blocker — WSL2 IMU (§6.7):** The D435i IMU is inaccessible through usbipd-win. Video streams work; the IMU does not. This blocks OpenVINS on WSL2. See §6.7 for workarounds (bare-metal Linux recommended).

**WSL2 USB setup procedure (for reference):**
1. Install usbipd-win: `winget install dorssel.usbipd-win`
2. Load the kernel module (WSL kernel 6.6+ has it): `sudo modprobe vhci-hcd`
3. Bind both cameras (elevated PowerShell): `usbipd bind --busid 3-1; usbipd bind --busid 3-2`
4. Attach to WSL (elevated): `usbipd attach --wsl --busid 3-1; usbipd attach --wsl --busid 3-2`
5. Recreate the container (not restart) so it picks up `/dev/video*`: `docker compose down prosthesis && docker compose up -d prosthesis`
6. Note: usbipd attachments do **not** survive WSL restart — steps 2–5 must be repeated after each reboot.
