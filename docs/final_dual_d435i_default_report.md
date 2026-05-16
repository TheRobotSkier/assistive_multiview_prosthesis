# Final Dual-D435i Default Report

## Scope

The repository now defaults to the final hardware layout:

- Head camera: Intel RealSense D435i, namespace `/head`, name `d435i_head`.
- Arm camera: Intel RealSense D435i, namespace `/arm`, name `d435i_arm`.
- Both cameras use built-in IMU streams united into `/imu`.

The previous mixed D435 + external I2C IMU rig remains available through explicit
`*-mixed` targets only.

## Interfaces

Jetson camera container publishes:

- `/head/d435i_head/color/image_raw`
- `/head/d435i_head/depth/color/points`
- `/head/d435i_head/imu`
- `/arm/d435i_arm/color/image_raw`
- `/arm/d435i_arm/depth/color/points`
- `/arm/d435i_arm/imu`

Jetson OpenVINS container consumes the color and IMU topics and publishes the
OpenVINS odometry/TF outputs under:

- `/ov_msckf_head`
- `/ov_msckf_arm`

Marker pose correction still accepts fixed marker IDs `0,1`, so the visible
ArUco marker ID `1` is part of the correction path.

## Launch Commands

Default final rig:

```bash
make jetson-cameras
make jetson-openvins
```

Explicit final rig:

```bash
make jetson-cameras-final
make jetson-openvins-final
```

Fallback mixed rig:

```bash
make jetson-cameras-mixed
make jetson-openvins-mixed
```

## Tests

Passed locally:

```bash
bash scripts/test_final_camera_contracts.sh
bash scripts/test_mixed_camera_contracts.sh
```

The direct host run of `scripts/test_digital_twin_contracts.sh` requires `ros2`
on `PATH`; run it through the repository test container with:

```bash
make test-digital-twin-contracts
```
