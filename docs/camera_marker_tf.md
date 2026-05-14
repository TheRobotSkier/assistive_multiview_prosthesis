# Marker-Based Camera TF

This branch adds a marker-map based camera positioning node for the Jetson
dual-D435/D435i setup. It publishes live `world -> d435i_head_link` and
`world -> d435i_arm_link` transforms from visible 100 x 100 mm ArUco markers,
so the existing RealSense TF tree can place both depth point clouds in RViz's
`world` frame.

## Runtime

From the host:

```bash
make jetson-camera-tf
```

This syncs the current branch to the Jetson, starts the camera container, starts
the `camera_marker_tf` container, and opens RViz with `world` as the fixed
frame. Stop it with:

```bash
make jetson-camera-tf-stop
```

The Jetson-side target is:

```bash
cd /home/robotlab/multiview_prosthesis/jetson
make camera-positioning
```

## Marker Map

The default config is:

```text
src/sensor_fusion_bringup/config/markers/camera_tf_markers.yaml
```

It uses OpenCV ArUco `DICT_6X6_1000` and a single `marker_0` with `size_m:
0.100`. Add more markers under `markers:` with measured `T_world_marker`
transforms. The node fuses all valid markers visible in each camera frame, with
weights based on image area, distance, and reprojection error.

Marker frame convention is centered on the printed marker with x right and y
down in the printed plane, matching the node's `solvePnP` object point order.

## Outputs

For each camera, the node publishes:

```text
/head/camera_marker_tf/pose
/head/camera_marker_tf/valid
/head/camera_marker_tf/active_marker_id
/head/camera_marker_tf/quality

/arm/camera_marker_tf/pose
/arm/camera_marker_tf/valid
/arm/camera_marker_tf/active_marker_id
/arm/camera_marker_tf/quality
```

It also publishes TF:

```text
world -> marker_<id>
world -> <color_optical_frame>_from_markers
world -> d435i_head_link
world -> d435i_arm_link
```

The `*_link` transforms are the ones RViz needs for the RealSense point clouds.

## Notes

NVIDIA's CUDA AprilTag stack is relevant if the physical markers are switched to
AprilTags, but this implementation uses OpenCV ArUco because that is already in
the existing Jetson ROS image and supports the requested 100 mm ArUco markers
without rebuilding an Isaac ROS container.
