# Jetson Orin Nano — D435i Camera Setup

This guide covers the host-side prerequisites needed to run dual D435i cameras
with IMU in Docker on the **Jetson Orin Nano** (Ubuntu 24.04 / JetPack 6.x / L4T 36.4.7).

The Docker setup automatically handles in-container RealSense packages
(`ros-jazzy-librealsense2`, `ros-jazzy-realsense2-camera`). The steps below
only need to be run **once on the host**.

---

## Expected topics after successful setup

```
/head/d435i_head/color/image_raw
/head/d435i_head/depth/image_rect_raw
/head/d435i_head/depth/color/points
/head/d435i_head/accel/sample
/head/d435i_head/gyro/sample
/head/d435i_head/imu
/arm/d435i_arm/depth/color/points
/arm/d435i_arm/imu
```

---

## 1. Prerequisites

### 1.1 Update D435i firmware

Update both cameras to firmware **5.17.0.10** or later using `realsense-viewer`
on a Linux x86 machine before deploying to the Jetson.

### 1.2 Remove conflicting host RealSense packages

If you previously installed RealSense packages on the host (e.g. from a ROS
Humble setup), purge them before proceeding:

```bash
dpkg -l | grep realsense
sudo dpkg -l | grep "realsense" | awk '{print $2}' | xargs -r sudo dpkg --purge
```

Open a new terminal after purging.

---

## 2. Patch `librealsense` on the Jetson host

These steps run on the **Jetson host** (not in Docker).

### 2.1 Clone librealsense

```bash
cd ~
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense
```

### 2.2 Apply Jetson patch fixes

Two workarounds are needed for JetPack 6.x:

1. Skip a broken NVIDIA license display step.
2. Replace outdated `nv-tegra.nvidia.com` host references.

```bash
cp scripts/patch-realsense-ubuntu-L4T.sh scripts/patch-realsense-ubuntu-L4T.sh.bak

# Skip broken license-display step
sed -i 's/^DisplayNvidiaLicense /# DisplayNvidiaLicense /' \
    scripts/patch-realsense-ubuntu-L4T.sh

# Replace old NVIDIA source host in repo lists
for f in scripts/Tegra/5.0.repos scripts/Tegra/6.0.repos scripts/Tegra/7.0.repos; do
    sed -i 's#nv-tegra.nvidia.com/#gitlab.com/nvidia/nv-tegra/#g' "$f"
done
```

### 2.3 Run the Jetson patch script

```bash
sudo ./scripts/patch-realsense-ubuntu-L4T.sh
```

This patches kernel modules so the D435i IMU (`/dev/iio*`) is accessible
from userspace and inside Docker via `/dev:/dev`.

### 2.4 Reboot

```bash
sudo reboot
```

---

## 3. Verify host-side device access

After rebooting, plug in both D435i cameras and confirm they appear:

```bash
# Should list USB devices including Intel RealSense
lsusb | grep RealSense

# Should show camera and IMU devices
ls /dev/video* /dev/iio*

# Enumerate cameras and serials
rs-enumerate-devices | grep -E "Serial|Name"
```

Record the serial numbers — update `docker/.env` with:

```
CAM_HEAD_SERIAL=336222071386
CAM_ARM_SERIAL=310622071850
```

> **Note**: The `docker/.env` values do NOT use the underscore prefix. The
> prefix is added in `sensor_fusion_bringup/config/d435i_cameras.yaml`
> (required by the `realsense2_camera` YAML parser to force string type).

---

## 4. Launch the hardware profile

```bash
# Build the image (first time or after Dockerfile changes)
make build

# Start hardware container with D435i support
make up-hw

# Tail camera logs
make logs-cameras
```

Inside the container, launch the D435i cameras:

```bash
ros2 launch sensor_fusion_bringup dual_d435i.launch.py
```

Or use the full pipeline:

```bash
ros2 launch prosthesis_launch digital_twin.launch.py camera:=true
```

---

## 5. Pointcloud NEON fix

On the Jetson Orin Nano, the RealSense pointcloud stream requires an extra
parameter to be set **after** the camera node initializes (~6 seconds):

```bash
ros2 param set /head/d435i_head pointcloud__neon_.enable true
ros2 param set /arm/d435i_arm  pointcloud__neon_.enable true
```

This is applied automatically by the `TimerAction` in `dual_d435i.launch.py`
and `single_d435i.launch.py`. Set `enable_pointcloud_neon_fix:=false` to skip
it on x86 systems where it is not needed.

---

## 6. Troubleshooting

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| `/dev/iio*` missing | Host kernel not patched | Re-run section 2 |
| `Device or resource busy` | Two nodes for same camera | Kill duplicate processes |
| IMU topics missing | Firmware too old | Update to 5.17.0.10+ |
| Pointcloud empty after launch | NEON fix not applied | Wait 6s or run param set manually |
| `serial_no type error` in logs | Serial without underscore prefix in YAML | Ensure `_336222071386` format in config |
