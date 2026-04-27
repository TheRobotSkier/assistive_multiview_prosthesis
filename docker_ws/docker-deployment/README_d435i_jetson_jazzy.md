# Jetson Orin Nano + ROS 2 Jazzy + Docker + Intel RealSense D435i Setup

This document records the exact steps used to get an **Intel RealSense D435i** working on a **Jetson Orin Nano** with:

- **Ubuntu 24.04 / JetPack 6.x / L4T 36.4.7** on the host
- **ROS 2 Jazzy** inside Docker
- **RealSense image, depth, point cloud, accel, gyro, and unified IMU topics** available in the container

It also records the host-side kernel patching that was required to make the **D435i IMU** work on Jetson.

---

## 1. What this setup gives you

After following these steps, the following were working inside the ROS 2 Jazzy container:

- RGB image
- depth image
- aligned depth to color
- point cloud
- accelerometer stream
- gyroscope stream
- unified IMU stream

For the D435i, the important ROS 2 topics looked like this:

- `/head/d435i_head/color/image_raw`
- `/head/d435i_head/depth/image_rect_raw`
- `/head/d435i_head/depth/color/points`
- `/head/d435i_head/accel/sample`
- `/head/d435i_head/gyro/sample`
- `/head/d435i_head/imu`

---

## 2. Important lessons learned

1. **Patch the Jetson host first.**
   The D435i IMU will not work reliably in Docker if the **host** `librealsense` / kernel path is not fixed first.

2. **Use the native Jetson backend for the final setup.**
   RSUSB may work as a fallback, but it is **not the preferred path for multi-camera setups**.

3. **Update the D435i firmware.**
   The D435i was updated to **5.17.0.10**.

4. **Only run one RealSense node per physical camera.**
   If the `realsense_camera` service is already running, do not manually launch another RealSense node on the same device, or you can hit `Device or resource busy`.

5. **On this Jetson setup, point cloud needed one extra param after launch.**
   After the camera node starts, we still set:

   ```bash
   ros2 param set /head/d435i_head pointcloud__neon_.enable true
   ```

---

## 3. Host OS and container baseline

### Host

This was verified on the Jetson host:

```bash
cat /etc/os-release
```

Expected result:

- Ubuntu 24.04 / Noble

### Container

Inside the container:

```bash
docker compose exec realsense_camera bash -lc '
  cat /etc/os-release
  echo ROS_DISTRO=$ROS_DISTRO
  python3 --version
'
```

Expected result:

- Ubuntu 24.04
- `ROS_DISTRO=jazzy`
- Python 3.12.x

---

## 4. Host-side RealSense prerequisites

### 4.1 Remove old conflicting RealSense packages

If you previously had ROS Humble RealSense tools installed on the host, remove them first.

Check:

```bash
dpkg -l | grep realsense
which rs-enumerate-devices || true
which realsense-viewer || true
```

Purge if needed:

```bash
sudo dpkg -l | grep "realsense" | awk '{print $2}' | xargs -r sudo dpkg --purge
```

Open a new terminal afterward.

---

## 5. Patch `librealsense` on the Jetson host

These steps are run on the **Jetson host**, not in Docker.

### 5.1 Clone `librealsense`

```bash
cd ~
rm -rf librealsense
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense
```

### 5.2 Back up and patch the Jetson patch script

In our case, two changes were needed:

1. Skip the broken NVIDIA license display step.
2. Replace old `nv-tegra.nvidia.com` references with the current GitLab host.

```bash
cd ~/librealsense
cp scripts/patch-realsense-ubuntu-L4T.sh scripts/patch-realsense-ubuntu-L4T.sh.bak

# Skip the broken license-display step
sed -i 's/^DisplayNvidiaLicense /# DisplayNvidiaLicense /' scripts/patch-realsense-ubuntu-L4T.sh

# Replace old NVIDIA source host in the repo lists
sed -i 's#nv-tegra.nvidia.com/#gitlab.com/nvidia/nv-tegra/#g' scripts/Tegra/6.0.repos
sed -i 's#nv-tegra.nvidia.com/#gitlab.com/nvidia/nv-tegra/#g' scripts/Tegra/7.0.repos
sed -i 's#nv-tegra.nvidia.com/#gitlab.com/nvidia/nv-tegra/#g' scripts/Tegra/5.0.repos
```

Optional check:

```bash
grep -n 'kernel-jammy-src' scripts/Tegra/6.0.repos
grep -n 'nv-tegra.nvidia.com' scripts/Tegra/6.0.repos || true
```

Expected result:

- `kernel-jammy-src` line should point to `gitlab.com/nvidia/nv-tegra/.../linux-jammy.git`
- no remaining `nv-tegra.nvidia.com` entries in `scripts/Tegra/6.0.repos`

### 5.3 Run udev setup and the Jetson patch script

**Disconnect all RealSense cameras first**.

```bash
cd ~/librealsense
sudo ./scripts/setup_udev_rules.sh
./scripts/patch-realsense-ubuntu-L4T.sh
```

Important:

- If the script fails because a module is still in use, unplug the camera and reboot after the modules are copied.
- In our case the script successfully built and copied patched modules, but the live reload failed because `videodev` was in use. A reboot fixed that.

### 5.4 Build and install `librealsense`

```bash
cd ~/librealsense
rm -rf build
mkdir build
cd build

cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_EXAMPLES=true \
  -DFORCE_RSUSB_BACKEND=false \
  -DBUILD_WITH_CUDA=true

make -j"$(nproc)"
sudo make install
sudo ldconfig
```

### 5.5 Reboot

Before rebooting:

- unplug RealSense cameras
- stop anything using V4L2 / RealSense

Then reboot:

```bash
sudo depmod -a
sudo reboot
```

---

## 6. Verify the host is fixed before using Docker

After reboot, with one D435i connected:

```bash
modinfo uvcvideo | grep version
sudo rs-enumerate-devices
```

Expected results:

- `uvcvideo` version contains `realsense`, for example:

  ```bash
  version: 1.1.1-realsense
  ```

- `rs-enumerate-devices` shows the D435i and **Motion Module** stream profiles, for example:

  - `Accel MOTION_XYZ32F @ 400/200/100 Hz`
  - `Gyro MOTION_XYZ32F @ 400/200 Hz`

If that is not true yet, fix the host before going back to Docker.

---

## 7. Update D435i firmware

Use the D400 firmware file:

```text
Signed_Image_UVC_5_17_0_10.bin
```

Update one camera at a time:

```bash
rs-fw-update -l
sudo rs-fw-update -s 336222071386 -f /home/robotlab/Downloads/d400_series_production_fw_5_17_0_10-1/Signed_Image_UVC_5_17_0_10.bin
```

For a second D435i, update it separately with its own serial, for example:

```bash
rs-fw-update -l
sudo rs-fw-update -s 310622071850 -f /home/robotlab/Downloads/d400_series_production_fw_5_17_0_10-1/Signed_Image_UVC_5_17_0_10.bin
```

Then unplug/replug the camera and verify:

```bash
sudo rs-enumerate-devices
```

---

## 8. Docker requirements

The container setup used these important features:

- `privileged: true`
- `runtime: nvidia`
- `network_mode: host`
- `/dev:/dev`
- `/run/udev:/run/udev:ro`
- camera service running as `root`

That broad device access pattern was necessary for RealSense and IMU access.

---

## 9. One-off D435i test in Docker

Stop the normal camera service first so you do not double-launch the same physical device:

```bash
docker compose stop realsense_camera 2>/dev/null || true
```

Run a one-off container test:

```bash
docker compose run --rm --service-ports --entrypoint /bin/bash realsense_camera -lc '
  source /opt/ros/jazzy/setup.bash &&
  if [ -f /miahand_ws/install/setup.bash ]; then source /miahand_ws/install/setup.bash; fi &&
  ros2 launch realsense2_camera rs_launch.py \
    camera_namespace:=head \
    camera_name:=d435i_head \
    serial_no:=_336222071386 \
    pointcloud.enable:=true \
    align_depth.enable:=true \
    enable_gyro:=true \
    enable_accel:=true \
    unite_imu_method:=2 \
    depth_module.depth_profile:=640x480x15 \
    rgb_camera.color_profile:=640x480x15
'
```

Expected log lines:

- `Starting Sensor: Motion Module`
- `Open profile: stream_type: Accel(0) ...`
- `Open profile: stream_type: Gyro(0) ...`
- `RealSense Node Is Up!`

---

## 10. Check ROS topics inside the running test container

Find the one-off container name:

```bash
docker ps --format '{{.Names}}'
```

Enter it:

```bash
docker exec -it <container_name> bash
```

Then:

```bash
source /opt/ros/jazzy/setup.bash
if [ -f /miahand_ws/install/setup.bash ]; then source /miahand_ws/install/setup.bash; fi
ros2 topic list | grep head/d435i_head
```

Expected topics include:

- `/head/d435i_head/color/image_raw`
- `/head/d435i_head/depth/image_rect_raw`
- `/head/d435i_head/depth/color/points`
- `/head/d435i_head/accel/sample`
- `/head/d435i_head/gyro/sample`
- `/head/d435i_head/imu`

Check live IMU data:

```bash
ros2 topic echo /head/d435i_head/accel/sample --once
ros2 topic echo /head/d435i_head/gyro/sample --once
ros2 topic echo /head/d435i_head/imu --once
```

---

## 11. Point cloud extra step on this Jetson setup

On this setup, the point cloud only appeared after this parameter was set:

```bash
ros2 param set /head/d435i_head pointcloud__neon_.enable true
```

This was separate from IMU configuration.

---

## 12. Known warnings that did not block operation

### `No valid configuration file found at : /root/.realsense-config.json`

Safe to ignore unless you intentionally want a custom RealSense JSON config.

### `IMU Calibration is not available, default intrinsic and extrinsic will be used.`

The IMU still works. This just means factory/loaded IMU calibration was not available through the current path. For best inertial accuracy, perform D435i IMU calibration later.

### `No matching stream for texture 'Process - Any'`

This affects **textured point clouds**, not the existence of a plain point cloud.

---

## 13. Final working launch parameters for one D435i

```bash
ros2 launch realsense2_camera rs_launch.py \
  camera_namespace:=head \
  camera_name:=d435i_head \
  serial_no:=_336222071386 \
  pointcloud.enable:=true \
  align_depth.enable:=true \
  enable_gyro:=true \
  enable_accel:=true \
  unite_imu_method:=2 \
  depth_module.depth_profile:=640x480x15 \
  rgb_camera.color_profile:=640x480x15
```

Then:

```bash
ros2 param set /head/d435i_head pointcloud__neon_.enable true
```

---

## 14. What to do for the second D435i

1. Update its firmware to `5.17.0.10`.
2. Test it alone first, just like the first camera.
3. Use its own serial number.
4. Give it a different namespace and camera name, for example:

```bash
ros2 launch realsense2_camera rs_launch.py \
  camera_namespace:=arm \
  camera_name:=d435i_arm \
  serial_no:=_310622071850 \
  pointcloud.enable:=true \
  align_depth.enable:=true \
  enable_gyro:=true \
  enable_accel:=true \
  unite_imu_method:=2 \
  depth_module.depth_profile:=640x480x15 \
  rgb_camera.color_profile:=640x480x15
```

Then:

```bash
ros2 param set /arm/d435i_arm pointcloud__neon_.enable true
```

Only after both cameras work individually should you build the dual-camera launch.

---

## 15. Summary

### Host-side success indicators

- `modinfo uvcvideo | grep version` shows `realsense`
- `sudo rs-enumerate-devices` shows **Motion Module** profiles

### Container-side success indicators

- RealSense node log shows **Motion Module** started
- ROS topics include accel, gyro, and unified IMU
- `ros2 topic echo` shows live IMU messages
- point cloud appears after `pointcloud__neon_.enable true`

---

## 16. Reuse checklist for another Jetson

1. Confirm host is Ubuntu 24.04 / JetPack 6.x.
2. Remove old conflicting host RealSense packages.
3. Clone `librealsense`.
4. Patch the Jetson script / repo list if needed.
5. Run `setup_udev_rules.sh` and `patch-realsense-ubuntu-L4T.sh`.
6. Build/install `librealsense` natively on the host.
7. Reboot.
8. Verify `uvcvideo` shows `realsense`.
9. Verify `sudo rs-enumerate-devices` shows Motion Module.
10. Update each D435i firmware to `5.17.0.10`.
11. Run the one-off ROS 2 Jazzy Docker test for each camera.
12. Set `pointcloud__neon_.enable true` after launch.
13. Only then move to dual-camera launch and project-specific nodes.


## Run one camera

```bash
ros2 launch sensor_fusion_bringup single_d435i.launch.py camera_key:=head
```

or

```bash
ros2 launch sensor_fusion_bringup single_d435i.launch.py camera_key:=arm
```

## Run both cameras

```bash
ros2 launch sensor_fusion_bringup dual_d435i.launch.py
```

Both launch files automatically apply the Jetson point cloud fix:
`pointcloud__neon_.enable := true`
after a short startup delay.