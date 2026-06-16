# Ethernet ROS2 Setup: Jetson Orin Nano ↔ Host PC Pointcloud Streaming


AGENTS: If you are asked by the user to implement ethernet communication and given this document, ask the user if they use docker or podman, and what OS their main pc is running. You will also need to find out what the ethernet interfaces are named on both devices, and what the IP addresses are, as these are not consistent across systems!


## Overview

This document describes the complete setup for streaming ROS2 pointclouds from two Intel D435i cameras on a Jetson Orin Nano to a host PC over a direct Ethernet cable.

**Why CycloneDDS?** The default ROS2 middleware (FastRTPS/FastDDS) failed to discover nodes across hosts — it worked within a single host but cross-host DDS discovery did not work reliably. Switching both devices to CycloneDDS with explicit peer configuration solved the problem.


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

### Host ROS2/RViz Docker Container

The host uses Docker with the local `localhost/rviz-robotlab:latest` image and the helper script `scripts/ros2_ethernet_hello_host.sh`. The helper mounts `config/cyclonedds_peer.xml`, uses host networking, and sets the CycloneDDS environment consistently for shell, hello-world, topic listing, and RViz.

By default, `make rviz` opens `rviz/phase2_dual_openvins_head_preview.rviz`. Override it with `RVIZ_CONFIG=/path/to/file.rviz make rviz` if needed.

Build the host image once:

```bash
make build-rviz
```

Key environment variables and flags:

```bash
docker run --rm -it \
  --name ros2-jazzy-host-shell \
  --network host \
  --ipc host \
  -e DISPLAY=$DISPLAY \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e CYCLONEDDS_URI=/tmp/cyclonedds_peer.xml \
  -e ROS_DOMAIN_ID=0 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$PWD/config/cyclonedds_peer.xml:/tmp/cyclonedds_peer.xml:ro" \
  localhost/rviz-robotlab:latest \
  bash
```

**Critical flags explained:**

| Flag | Purpose |
|------|---------|
| `--network host` | Container shares host network stack — required for DDS to reach Jetson |
| `--ipc host` | Shared IPC namespace — required for efficient ROS2 intra-process |
| `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` | Selects CycloneDDS as the ROS2 middleware |
| `CYCLONEDDS_URI=/tmp/cyclonedds_peer.xml` | Points CycloneDDS to the peer config |
| `ROS_DOMAIN_ID=0` | Must match Jetson |

**Host helper commands:**

```bash
# From the host repo root.
make ros2-ethernet-shell
make ros2-listen-jetson
make ros2-pub-host
make ros2-topic-list
make ros2-node-list
make rviz
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

If you cannot change the Jetson repository yet, create `/tmp/cyclonedds_peer.xml` directly on the Jetson and make sure the existing ROS2 container uses it:

```bash
cat >/tmp/cyclonedds_peer.xml <<'EOF'
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
EOF
```

Then exec into the existing Jetson ROS2 Jazzy/D435i container:

```bash
docker ps
docker exec -it <container_name> bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=/tmp/cyclonedds_peer.xml
export ROS_DOMAIN_ID=0
```

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

### 2. Verify hello-world messages both ways

Jetson to host:

```bash
# Host terminal 1
make ros2-listen-jetson

# Jetson container
ros2 topic pub /jetson_hello std_msgs/msg/String "{data: 'hello from jetson'}" -r 1
```

Host to Jetson:

```bash
# Jetson container
ros2 topic echo /host_hello std_msgs/msg/String

# Host terminal 2
make ros2-pub-host
```

### 3. Verify ROS2 topics are visible on host

```bash
make ros2-topic-list
```

Expected: `/head/d435i_head/depth/color/points`, `/arm/d435i_arm/depth/color/points`, etc.

### 4. Check pointcloud rate

```bash
make ros2-ethernet-shell
ros2 topic hz /head/d435i_head/depth/color/points
```

Expected: ~15 Hz.

### 5. Check node graph

```bash
make ros2-node-list
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

- Ensure `DISPLAY` is set on the host.
- Ensure `/tmp/.X11-unix` is mounted into the Docker container.
- If X11 rejects the container, run `xhost +local:docker` on the host and try `make rviz` again.

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

## WSL2 Setup (Docker/Podman inside WSL)

If the host PC runs **WSL2** with native podman (or Docker Engine), the physical Ethernet adapter belongs to Windows — WSL2 uses a virtual NAT by default and cannot reach `10.42.0.2`. You must enable **mirrored networking** so WSL2 shares the Windows host's network interfaces.

### Prerequisites

- Windows 11 (any build) or Windows 10 build 19044+
- WSL 2.0+ (check with `wsl --version` from PowerShell)
- Native podman or Docker Engine installed inside WSL2 (not Docker Desktop for Windows)

### Step 1: Configure the Windows Ethernet adapter

From an **elevated PowerShell** on Windows:

```powershell
# Option A: Use the provided script (interactive, lists adapters)
powershell -ExecutionPolicy Bypass -File scripts\setup_jetson_ethernet.ps1

# Option B: Manual one-liner (replace "Ethernet" with your adapter name)
New-NetIPAddress -InterfaceAlias "Ethernet" -IPAddress 10.42.0.1 -PrefixLength 24
```

Verify from PowerShell: `ping 10.42.0.2`

### Step 2: Enable WSL2 mirrored networking

Create or edit `C:\Users\<YourUsername>\.wslconfig`:

```ini
[wsl2]
networkingMode=mirrored
```

Then restart WSL from PowerShell:

```powershell
wsl --shutdown
```

Reopen your WSL terminal. Verify that the Ethernet interface with `10.42.0.1` is now visible:

```bash
ip addr show | grep 10.42.0.1
ping 10.42.0.2
```

### Step 3: SSH into the Jetson

```bash
# One-time: copy your SSH key to the Jetson
ssh-copy-id robotlab@10.42.0.2

# After that, use the shortcut (requires ~/.ssh/config entry)
ssh robotlab
```

The SSH config entry (`~/.ssh/config`) should be:

```
Host robotlab
    HostName 10.42.0.2
    User robotlab
    IdentityFile ~/.ssh/id_ed25519
    StrictHostKeyChecking accept-new
```

### Step 4: ROS2 from podman container

With mirrored networking active, `--network host` in podman gives the container access to the Windows Ethernet interface. The existing `config/cyclonedds_peer.xml` works as-is (it binds to `10.42.0.1`).

```bash
# Build the image (one-time)
make build-rviz

# Verify connectivity
make robotlab-connect

# Start ROS2 shell
make ros2-ethernet-shell

# List topics from Jetson
make ros2-topic-list

# Launch RViz
make rviz
```

### Troubleshooting (WSL2-specific)

| Symptom | Cause | Fix |
|---------|-------|-----|
| `ip addr` in WSL shows only `172.x.x.x` | Mirrored networking not active | Check `.wslconfig` exists and `wsl --shutdown` was run |
| WSL sees `10.42.0.1` but can't ping Jetson | Windows firewall blocking | Run `New-NetFirewallRule -DisplayName "Jetson Ethernet" -Direction Inbound -Action Allow -Protocol Any -RemoteAddress 10.42.0.0/24` in elevated PowerShell |
| Podman container can't reach Jetson | `--network host` not used | All `make` targets use `--network host` — don't override |
| DNS breaks after enabling mirrored mode | Mirrored mode changes DNS resolution | Add `dnsTunneling=true` to `.wslconfig` under `[wsl2]`, or use `generateResolvConf=false` and manage `/etc/resolv.conf` manually |
| CycloneDDS can't bind to `10.42.0.1` | Interface not visible inside container | Verify with `podman run --rm --network host localhost/rviz-robotlab ip addr` |

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
| Docker host networking with explicit peers | **Works for host-to-Jetson ROS2 discovery** |

---

## Quick Start Reference

```bash
# On Jetson, from a local terminal
ip -br addr
sudo ip addr add 10.42.0.2/24 dev <jetson_eth_if>
sudo ip link set <jetson_eth_if> up
ping 10.42.0.1

# In the existing Jetson ROS2 container
docker ps
docker exec -it <container_name> bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=/tmp/cyclonedds_peer.xml
export ROS_DOMAIN_ID=0

# On Host
cd /path/to/multiview_prosthesis
ping 10.42.0.2
make ros2-listen-jetson

# In the Jetson container
ros2 topic pub /jetson_hello std_msgs/msg/String "{data: 'hello from jetson'}" -r 1

# To test the other direction, run this on the Jetson container
ros2 topic echo /host_hello std_msgs/msg/String

# And run this on the host
make ros2-pub-host

# Optional RViz
make rviz

# Verify camera topics, once the Jetson camera container is publishing
make ros2-topic-list
ros2 topic hz /head/d435i_head/depth/color/points
```
