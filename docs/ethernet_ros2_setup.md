# Ethernet ROS2 Setup: Jetson Orin Nano ↔ Host PC Pointcloud Streaming

## Overview

This document describes the complete setup for streaming ROS2 pointclouds from two Intel D435i cameras on a Jetson Orin Nano to a host PC over a direct Ethernet cable.

**Why CycloneDDS?** The default ROS2 middleware (FastRTPS/FastDDS) failed to discover nodes across hosts — it worked within a single host but cross-host DDS discovery did not work reliably. Switching both devices to CycloneDDS with explicit peer configuration solved the problem.

**Why Ethernet?** WiFi DDS discovery does not work reliably for this use case. A direct Ethernet cable is required.

---

## Network Setup

| Device         | IP Address  | Role         |
|----------------|-------------|--------------|
| Host PC        | `10.42.0.1` | RViz viewer  |
| Jetson Orin Nano | `10.42.0.2` | Camera publisher |

- **Connection**: Direct Ethernet cable (no switch/router)
- **Interface**: Check with `ip addr` on host — likely `enp*` or `eth*`
- **ROS_DOMAIN_ID**: `0` on both devices

### Configuring the host Ethernet interface

The host IP (`10.42.0.1`) can be set via NetworkManager or manually:

```bash
# Find the interface name
ip addr

# Set static IP (replace enp5s0 with your interface)
sudo ip addr add 10.42.0.1/24 dev enp5s0
sudo ip link set enp5s0 up
```

Or use `nmcli` / GNOME network settings to set a static IPv4 address of `10.42.0.1/24` on the Ethernet interface connected to the Jetson.

### Configuring the Jetson Ethernet interface

On the Jetson (via SSH or directly):

```bash
sudo ip addr add 10.42.0.2/24 dev eth0   # replace eth0 with actual interface
sudo ip link set eth0 up
```

Verify connectivity:
```bash
ping 10.42.0.2   # from host
ping 10.42.0.1   # from Jetson
```

---

## CycloneDDS: What It Is and Why

CycloneDDS is an alternative DDS implementation for ROS2. It supports explicit peer configuration, which is essential for cross-host communication without multicast — particularly important on point-to-point Ethernet links where multicast may not propagate correctly.

Both devices use **the same structure** of CycloneDDS config, differing only in the `NetworkInterfaceAddress`.

---

## Host Side Setup

### CycloneDDS Configuration

File: `config/cyclonedds_peer.xml` (relative to host repo root)

```xml
<CycloneDDS>
  <Domain Id="0">
    <General>
      <NetworkInterfaceAddress>10.42.0.1</NetworkInterfaceAddress>
    </General>
    <Discovery>
      <Peers>
        <Peer address="10.42.0.1"/>
        <Peer address="10.42.0.2"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
```

- `NetworkInterfaceAddress`: binds CycloneDDS to the Ethernet interface IP. Without this, CycloneDDS may choose WiFi or loopback.
- `Peers`: explicit peer list forces discovery attempts to both addresses. Required when multicast doesn't work.

> **Note**: `NetworkInterfaceAddress` is deprecated in CycloneDDS v0.10.5+ in favor of `<Interfaces><NetworkInterface>`. However, the newer syntax caused container crashes. The deprecated syntax is functional and should be used until the container image is updated.

> **Note**: Use `<Domain Id="0">` as an attribute (not `<Domain><Id>0</Id></Domain>`). The latter form caused parse errors.

### RViz Container (`make rviz`)

The `rviz` target in the host `Makefile` launches a podman container with RViz2 configured to receive topics from the Jetson.

Key environment variables and flags:

```makefile
rviz:
	xhost +
	podman run --rm -d --name rviz-robotlab \
		--network host \
		--ipc host \
		--device /dev/dri \
		--userns=keep-id \
		-e DISPLAY=$(DISPLAY) \
		-e XAUTHORITY=/tmp/.xauth \
		-e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
		-e CYCLONEDDS_URI=/tmp/cyclonedds_peer.xml \
		-e ROS_DOMAIN_ID=0 \
		-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
		-v $(XAUTHORITY):/tmp/.xauth:ro \
		-v $(CURDIR)/rviz/robotlab_cameras.rviz:/rviz_config.rviz:ro \
		-v $(CURDIR)/config/cyclonedds_peer.xml:/tmp/cyclonedds_peer.xml:ro \
		localhost/rviz-robotlab \
		bash -c 'source /opt/ros/jazzy/setup.bash && rviz2 -d /rviz_config.rviz'
```

**Critical flags explained:**

| Flag | Purpose |
|------|---------|
| `--network host` | Container shares host network stack — required for DDS to reach Jetson |
| `--ipc host` | Shared IPC namespace — required for efficient ROS2 intra-process |
| `--userns=keep-id` | **Required for Wayland/XWayland**: without this, the container runs as a different UID and X11 drops the connection in under 1 second |
| `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` | Selects CycloneDDS as the ROS2 middleware |
| `CYCLONEDDS_URI=/tmp/cyclonedds_peer.xml` | Points CycloneDDS to the peer config |
| `ROS_DOMAIN_ID=0` | Must match Jetson |

**Building the RViz image** (if not already built):

```bash
# From the host repo root
docker build -f docker/Dockerfile.rviz -t localhost/rviz-robotlab .
# or with podman:
podman build -f docker/Dockerfile.rviz -t localhost/rviz-robotlab .
```

**Running:**

```bash
make rviz
```

**Stopping:**

```bash
podman kill rviz-robotlab
```

---

## Jetson Side Setup

### CycloneDDS Configuration

File: `docker_ws/docker-deployment/cyclonedds_robotlab.xml` (in `jetson-docker` repo)

```xml
<CycloneDDS>
  <Domain Id="0">
    <General>
      <NetworkInterfaceAddress>10.42.0.2</NetworkInterfaceAddress>
    </General>
    <Discovery>
      <Peers>
        <Peer address="10.42.0.1"/>
        <Peer address="10.42.0.2"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
```

Same structure as the host config, but `NetworkInterfaceAddress` is set to the Jetson's IP (`10.42.0.2`).

### Camera Container (`make cameras`)

The `cameras` target in `~/jetson_ws/jetson-docker/Makefile` launches the camera Docker container:

```makefile
cameras:
	$(SUDO) docker rm -f cameras_test 2>/dev/null || true
	$(SUDO) docker run -d \
		--name cameras_test \
		--network host \
		--ipc host \
		--runtime nvidia \
		--privileged \
		--group-add video \
		--group-add plugdev \
		--group-add dialout \
		--group-add i2c \
		-e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
		-e CYCLONEDDS_URI=/tmp/cyclonedds_peer.xml \
		-e ROS_DOMAIN_ID=0 \
		-v $(CURDIR)/docker_ws:/miahand_ws/src \
		-v $(CURDIR)/docker_ws/docker-deployment/cyclonedds_robotlab.xml:/tmp/cyclonedds_peer.xml:ro \
		-v /dev:/dev \
		-v /run/udev:/run/udev:ro \
		-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
		docker-deployment-miahand_ros2 \
		bash -c 'source /miahand_ws/install/setup.bash 2>/dev/null; source /opt/ros/jazzy/setup.bash; ros2 launch sensor_fusion_bringup dual_d435i.launch.py'
```

**Key flags:**

| Flag | Purpose |
|------|---------|
| `--network host` | Container shares Jetson network — DDS traffic reaches host directly |
| `--runtime nvidia` | Enables NVIDIA GPU runtime (needed for CUDA-accelerated camera processing) |
| `--privileged` + `/dev` mount | Required for USB camera access |
| `CYCLONEDDS_URI` | Points to the mounted peer config |
| `cyclonedds_robotlab.xml` → `/tmp/cyclonedds_peer.xml` | Config mounted read-only |

**Running:**

```bash
cd ~/jetson_ws/jetson-docker
make cameras
```

**Checking logs:**

```bash
docker logs -f cameras_test
```

**Stopping:**

```bash
docker rm -f cameras_test
```

### Launch File: `dual_d435i.launch.py`

Located at `docker_ws/multi_cam_localization/sensor_fusion_bringup/launch/dual_d435i.launch.py`.

This launch file:
- Reads camera config from `sensor_fusion_bringup/config/d435i_cameras.yaml`
- Starts two `realsense2_camera` nodes with namespaces `head` and `arm`
- Enables pointcloud, aligned depth, IMU (gyro+accel)
- Applies a Jetson-specific fix: after startup, sets `pointcloud__neon_.enable` parameter via `ros2 param set` to work around a NEON optimization bug on ARM

---

## Published Topics

After both containers are running, the following topics are available on the ROS2 network (visible from both host and Jetson):

| Topic | Type | Rate |
|-------|------|------|
| `/head/d435i_head/depth/color/points` | `sensor_msgs/PointCloud2` | ~15 Hz |
| `/arm/d435i_arm/depth/color/points` | `sensor_msgs/PointCloud2` | ~15 Hz |
| `/head/d435i_head/color/image_raw` | `sensor_msgs/Image` | ~15 Hz |
| `/arm/d435i_arm/color/image_raw` | `sensor_msgs/Image` | ~15 Hz |
| `/head/d435i_head/imu` | `sensor_msgs/Imu` | ~200 Hz |
| `/arm/d435i_arm/imu` | `sensor_msgs/Imu` | ~200 Hz |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | — |

---

## Verification

### 1. Check network connectivity

```bash
# From host
ping 10.42.0.2

# From Jetson
ping 10.42.0.1
```

### 2. Verify ROS2 topics are visible on host

```bash
# Run inside a ROS2 environment on host with correct env vars:
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
CYCLONEDDS_URI=$(pwd)/config/cyclonedds_peer.xml \
ROS_DOMAIN_ID=0 \
ros2 topic list
```

Expected: `/head/d435i_head/depth/color/points`, `/arm/d435i_arm/depth/color/points`, etc.

Or exec into the running RViz container:

```bash
podman exec -it rviz-robotlab bash
source /opt/ros/jazzy/setup.bash
ros2 topic list
```

### 3. Check pointcloud rate

```bash
ros2 topic hz /head/d435i_head/depth/color/points
```

Expected: ~15 Hz.

### 4. Check node graph

```bash
ros2 node list
```

Should show nodes from both host and Jetson.

---

## Troubleshooting

### Topics not visible on host

1. **Check IP connectivity first**: `ping 10.42.0.2`
2. **Check RMW on both sides**: must be `rmw_cyclonedds_cpp` on both
3. **Check `CYCLONEDDS_URI`**: must point to the correct XML file and be readable inside the container
4. **Check `ROS_DOMAIN_ID`**: must be `0` on both
5. **Verify the Ethernet interface has the right IP**: `ip addr show` — the interface connected to Jetson must have `10.42.0.1`

### X11 drops immediately (RViz shows window then closes)

- Cause: Running podman without `--userns=keep-id` under Wayland/XWayland. The container runs as a remapped UID, which is rejected by the X server.
- Fix: Ensure `--userns=keep-id` is in the `podman run` command.
- Also ensure `xhost +` was run before starting the container.

### Camera container starts but no topics appear

```bash
docker logs cameras_test
```

Common causes:
- USB cameras not detected (`--privileged` or `/dev` mount missing)
- CycloneDDS config not found at `/tmp/cyclonedds_peer.xml` (check volume mount path)
- Wrong `NetworkInterfaceAddress` — CycloneDDS bound to wrong interface

### CycloneDDS config syntax errors / container crashes

- Do NOT use `<Interfaces><NetworkInterface name="...">` — this caused crashes on the container image in use
- Use `<NetworkInterfaceAddress>IP</NetworkInterfaceAddress>` instead (deprecated but functional)
- Use `<Domain Id="0">` as an XML attribute, not `<Domain><Id>0</Id></Domain>`

### Bandwidth / latency issues

Pointcloud data is large (~10-30 MB/s per camera). Ensure:
- Gigabit Ethernet (not 100Mbps)
- Cable is Cat5e or better
- No other traffic on the interface

---

## Key Lessons Learned

| What was tried | Result |
|----------------|--------|
| Default FastRTPS/FastDDS for cross-host ROS2 | **Did not work** — discovery failed between host and Jetson |
| WiFi for DDS transport | **Did not work** — DDS multicast/unicast unreliable over WiFi |
| CycloneDDS with explicit peers over Ethernet | **Works** |
| `<Interfaces><NetworkInterface>` syntax (CycloneDDS v0.10.5+) | **Container crash** — not compatible with container image |
| `<NetworkInterfaceAddress>` (deprecated syntax) | **Works** — use this until image is updated |
| `<Domain><Id>0</Id></Domain>` form | **Parse error** — use `<Domain Id="0">` attribute form |
| podman without `--userns=keep-id` under Wayland | **X11 drops in <1s** |
| podman with `--userns=keep-id` | **Works** |

---

## Quick Start Reference

```bash
# On Jetson (SSH in first)
ssh robotlab
cd ~/jetson_ws/jetson-docker
make cameras

# On Host
cd /path/to/multiview_prosthesis
make rviz

# Verify (on host, in a ROS2 shell or exec into rviz container)
ros2 topic list
ros2 topic hz /head/d435i_head/depth/color/points
```
