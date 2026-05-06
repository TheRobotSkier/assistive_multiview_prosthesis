# Chalkboard

## Objective
Restore Intel RealSense camera access inside containers, first with a minimal non-ROS image and then with a ROS image that can run the official `realsense2_camera` node. After validation, propagate the fix into the full system test and digital twin services.

## Working plan
1. Inspect prior notes, current compose setup, and host device topology.
2. Prove host-side camera nodes and stable identifiers (`/dev/v4l/by-id`, `/dev/bus/usb`) are available.
3. Build a minimal non-ROS container that can enumerate and read RealSense V4L2 devices.
4. Build or adapt a ROS container that can access the camera and start the official `realsense2_camera` node.
5. Identify the root cause of the current failure (permissions, device remapping after reset, missing udev/USB passthrough, or launch parameters).
6. Apply the confirmed fix to `multiview_full`, then wire the same container/device setup into `full_system_test` and `digital_twin` as required.
7. Rebuild and rerun the relevant services to verify camera data is readable inside containers and ROS topics are published.

## Attempt log
- Initial host inspection: `/dev/video4`-`/dev/video14` belong to the RealSense D435, and stable V4L symlinks exist under `/dev/v4l/by-id` and `/dev/v4l/by-path`.
- Container runtime on this machine is Podman/Podman Compose.
- The RealSense D435 appears as `/dev/video9` through `/dev/video14` (6 video nodes) plus `/dev/media2`, `/dev/media3`.
- Device numbering changes across USB re-enumeration — `--device /dev/videoN` alone is brittle; `/dev:/dev` bind mount or `--privileged` is safer for camera containers.

## Failures / blockers

### Blocked: `docker compose` Podman shim
`docker compose -f docker-compose.linux-podman.yml build multiview_cameras` failed with `missing services [multiview_cameras]`. Direct `podman build` / `podman run` was used for verification instead.

### Solved: `set -u` before sourcing ROS setup
First run of the new ROS probe script failed because `set -u` was active before sourcing `/opt/ros/humble/setup.bash`, which triggered `AMENT_TRACE_SETUP_FILES: unbound variable`. Fixed by sourcing ROS before enabling nounset mode.

### Solved: Permission denied on /dev/videoN inside container
Using `--device /dev/bus/usb:/dev/bus/usb` alone does NOT give access to the V4L2 video device nodes. The librealsense library eventually tries to open `/dev/videoN` and gets EACCES. Fix: pass the actual video devices with `--device /dev/videoN:/dev/videoN` for each camera node, or (simpler) use `-v /dev:/dev` or `--privileged` which the compose file already does.

### Solved: `xioctl(VIDIOC_S_FMT) failed: I/O error`
Occurs when the camera is in a bad state after a previous session that didn't cleanly release the device. A device firmware reset (`initial_reset:=true`) or physical re-plug resolves it. The `realsense_ros_probe.sh` respects `REALSENSE_INITIAL_RESET` (default `false`).

### Solved: `map_device_descriptor Cannot open /dev/videoN: Permission denied`
After the first format-setting attempt fails (I/O error), subsequent retries hit EACCES because the USB/video device access model inside the container doesn't re-validate capabilities after a USB reset. Ensuring all node paths are available (`--privileged` or `-v /dev:/dev`) avoids this.

### Solved: CycloneDDS intra-container peer discovery failure (ROOT CAUSE OF "NO TOPICS")
**This was the main issue.** The `rs_launch.py` node starts, the camera streams, but `ros2 topic list` sees NO topics.

- **RMW = rmw_cyclonedds_cpp**: `rclpy.init()` works, but `get_topic_names_and_types()` blocks/hangs. DDS participants cannot discover each other inside the same container even with `--network host` and `--ipc host`. Root cause: CycloneDDS SHM/network discovery mechanism fails in this Podman environment. Attempted fixes that did NOT help: `--ipc host`, unsetting `ROS_AUTOMATIC_DISCOVERY_RANGE` (Humble ignores it), creating CycloneDDS XML config.
- **RMW = rmw_fastrtps_cpp**: Everything works. Topics are discovered, depth images stream with valid headers. FastDDS does not have the SHM/discovery issue that CycloneDDS has in this environment.

**Fix**: Switch from `rmw_cyclonedds_cpp` to `rmw_fastrtps_cpp` for all services that subscribe to camera topics.

### Compose file changes applied
1. **multiview_cameras**: `RMW_IMPLEMENTATION` changed to `rmw_fastrtps_cpp`, removed `ROS_AUTOMATIC_DISCOVERY_RANGE`.
2. **multiview_full**: Same RMW change.
3. **full_system_test**: Same RMW change.
4. **digital_twin**: Added `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` to environment.
5. **Dockerfile.humble_cameras**: Removed `ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` and `ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`. Removed BuildKit `--mount=type=cache` which causes lock issues with Podman.

## Verified working
- `podman run` with `--privileged --network host --ipc host --device /dev/video9:/dev/video9 ... --device /dev/bus/usb:/dev/bus/usb` on `localhost/multiview-humble-realsense:latest`
- realsense2_camera node starts, finds D435 (serial 829212072207), opens depth at 640x480x15 Z16
- `ros2 topic list` shows 8 topics including `/cam1/d435_1/depth/image_rect_raw`
- `ros2 topic echo /cam1/d435_1/depth/image_rect_raw --once` returns valid stamped image headers
- Frame ID: `d435_1_depth_optical_frame`
- Timestamps are live (not stale)
- No permissions or I/O errors when the full device set is available
