# Sprint 2 Report: Mixed D435 + D435i OpenVINS Setup

Date: 2026-05-16

## Scope

This sprint made the Jetson camera/OpenVINS bringup usable with the current
temporary hardware:

- one D435 without onboard IMU
- one D435i with onboard IMU
- one external GY-91/MPU-compatible IMU on the Jetson I2C header

The final two-D435i path is still present and selectable.

## Changes

- Added `mixed_d435_d435i_cameras.yaml`.
  - Head camera keeps ROS name `d435i_head` for topic compatibility, even
    though the physical unit is a D435.
  - Head D435 disables RealSense gyro/accel streams.
  - Head external IMU publishes `/head/d435i_head/imu`.
  - Arm D435i keeps built-in IMU enabled.
  - Mixed mode uses conservative USB bandwidth settings: depth
    `424x240x15`, color `640x480x30`.
- Added `i2c_mpu9250_imu_node.py`.
  - Publishes `sensor_msgs/Imu` directly from `/dev/i2c-*`.
  - Uses no third-party I2C Python package.
  - Defaults to bus `7`, address `0x68`, frame `head_imu`.
  - Publishes high covariance values so OpenVINS does not over-trust the
    temporary external IMU.
- Updated `dual_d435i.launch.py`.
  - Accepts `camera_config:=...`.
  - Supports per-camera built-in IMU enable/disable.
  - Starts the external I2C IMU publisher when configured.
- Updated OpenVINS launch selection.
  - `dual_openvins_phase2.launch.py` now has `rig_mode`.
  - `rig_mode:=mixed_d435_d435i` selects D435 + external IMU for the head.
  - `rig_mode:=dual_d435i` selects the final D435i head launch.
- Added `head_d435i_openvins_phase2.launch.py` for the final two-D435i rig.
- Updated the temporary head D435 OpenVINS calibration.
  - IMU topic changed to `/head/d435i_head/imu`.
  - IMU noise was inflated for lower trust in the external IMU.
- Updated Jetson Makefile targets.
  - `make cameras-mixed`
  - `make openvins-mixed`
  - `make cameras-final`
  - `make openvins-final`
- Updated host Makefile targets.
  - `make jetson-cameras` defaults to mixed mode for current hardware.
  - `make jetson-openvins` defaults to mixed mode for current hardware.
  - `make jetson-cameras-final` and `make jetson-openvins-final` remain
    available for the second D435i.
- Fixed test image rebuild behavior so `make test` rebuilds the compose
  `test` service, not only `prosthesis`.

## Interfaces

Mixed camera mode expects:

- Head D435:
  - color: `/head/d435i_head/color/image_raw`
  - depth cloud: `/head/d435i_head/depth/color/points`
  - external IMU: `/head/d435i_head/imu`
  - IMU frame: `head_imu`
- Arm D435i:
  - color: `/arm/d435i_arm/color/image_raw`
  - depth cloud: `/arm/d435i_arm/depth/color/points`
  - built-in IMU: `/arm/d435i_arm/imu`
  - IMU frame: `arm_imu`

The host-side pipeline remains unchanged because the camera names preserve the
existing topic and TF contracts.

## Tests

Passing in Podman:

```bash
make test
```

`make test` currently passes:

- build
- launch syntax
- preshaping `.so`
- node startup
- twist propagation
- digital twin contracts
- mixed camera contracts

## Runtime Status

Jetson runtime testing was not completed in this sprint because the direct
Ethernet helper could not reach `192.168.100.2` during discovery. The code path
is ready to sync and run when the Jetson is reachable again.

## Next Sprint

1. Restore Jetson connectivity.
2. Run `make jetson-list-cameras`.
3. If serials differ from `mixed_d435_d435i_cameras.yaml`, edit only that YAML.
4. Run `make jetson-cameras-mixed`.
5. Verify:
   - `/head/d435i_head/imu`
   - `/head/d435i_head/depth/color/points`
   - `/arm/d435i_arm/imu`
   - `/arm/d435i_arm/depth/color/points`
6. Run `make jetson-openvins-mixed`.
