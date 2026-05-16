# Sprint 3 Report: Jetson Runtime Smoke Test Blocked

Date: 2026-05-16

## Goal

Verify the mixed D435 + D435i camera/IMU topics from the Jetson before running
OpenVINS and the full host pipeline.

## Result

Runtime testing is blocked because the Jetson is not reachable from the host.

Checked:

- `robotlab` from SSH config, currently `192.168.100.2`
- `192.168.100.2`
- `10.42.0.2`
- `robotlab.local`

All failed ping and SSH during this sprint. The host Ethernet adapter
`enp0s13f0u2u2` was present but down/no route to the Jetson.

## Added While Blocked

Added `scripts/check_mixed_jetson_topics.sh` and Makefile target:

```bash
make check-mixed-topics
```

This verifies that the mixed rig is publishing:

- `/head/d435i_head/color/image_raw`
- `/head/d435i_head/depth/color/points`
- `/head/d435i_head/imu`
- `/arm/d435i_arm/color/image_raw`
- `/arm/d435i_arm/depth/color/points`
- `/arm/d435i_arm/imu`

It also waits for one message on both IMU topics and both pointcloud topics.

## Next Action When Jetson Is Reachable

```bash
make jetson-sync
make jetson-list-cameras
make jetson-cameras-mixed
make check-mixed-topics
make jetson-openvins-mixed
```

If camera serials differ, edit only:

```text
src/sensor_fusion_bringup/config/mixed_d435_d435i_cameras.yaml
```
