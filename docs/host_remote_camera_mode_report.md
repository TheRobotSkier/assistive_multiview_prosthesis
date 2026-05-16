# Host Remote Camera Mode Report

## Scope

The host digital twin launch now treats camera data as a remote Jetson input by
default. This matches the target architecture: cameras and OpenVINS run on the
Jetson, while fusion, segmentation, grasp prediction, and RViz hand rendering
run on the host PC.

## Interfaces

With `camera:=true`, the host subscribes to Jetson camera pointcloud topics:

- `/head/d435i_head/depth/color/points`
- `/arm/d435i_arm/depth/color/points`

The host then publishes:

- `/fused_pointcloud`
- `/segmentation/input_cloud`
- digital hand `/joint_states`

Local host RealSense drivers are opt-in only:

```bash
ros2 launch prosthesis_launch digital_twin.launch.py camera:=true launch_cameras:=true
```

Mock cloud mode remains available:

```bash
ros2 launch prosthesis_launch digital_twin.launch.py camera:=false
```

## Test

Focused contract test:

```bash
make test-digital-twin-contracts
```
