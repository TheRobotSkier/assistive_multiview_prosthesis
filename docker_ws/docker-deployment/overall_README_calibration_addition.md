# Calibration

Calibration outputs and reproduction notes are kept under `docker_ws/calibration/`.

The current head-mounted Intel RealSense D435i calibration is stored here:

```text
docker_ws/calibration/head/d435i_336222071386/
```

This folder contains the tracked calibration artifacts needed for OpenVINS integration/testing:

```text
docker_ws/calibration/head/d435i_336222071386/
├── README.md
├── aprilgrid_6x6_80mm_0p3.yaml
├── camera_intrinsics/
├── camera_intrinsics_from_dynamic/
├── imu_noise/
└── camera_imu_extrinsics_inflated10x/
```

Large ROS bags are intentionally not tracked in Git. Keep raw and converted bags under the gitignored folder:

```text
docker_ws/bags/
```

## Current head D435i OpenVINS calibration

Use the final 10x-inflated camera-IMU calibration for OpenVINS RGB monocular + IMU testing:

```text
docker_ws/calibration/head/d435i_336222071386/camera_imu_extrinsics_inflated10x/head_rgb_imu_dynamic_20260429_131443_ros1_inflated10x-camchain-imucam.yaml
docker_ws/calibration/head/d435i_336222071386/camera_imu_extrinsics_inflated10x/head_d435i_imu_200hz_trim_first30min_inflated10x.yaml
```

The relevant runtime ROS topics are:

```text
/head/d435i_head/color/image_raw
/head/d435i_head/imu
```

The calibration workflow uses:

```text
Aprilgrid 6x6
tagSize: 0.08 m
tagSpacing: 0.3
```

The full reproduction guide, calibration results, redo criteria, and file-use notes are in:

```text
docker_ws/calibration/head/d435i_336222071386/README.md
```

## OpenVINS integration note

When creating the OpenVINS head-camera configuration, use the final Kalibr `*camchain-imucam.yaml` directly as the source of truth for camera intrinsics, camera-IMU extrinsics, and camera-IMU time offset. Do not manually swap `T_ci` / `T_ic` by intuition; check the OpenVINS config field convention and convert only if that specific field requires the inverse transform.
