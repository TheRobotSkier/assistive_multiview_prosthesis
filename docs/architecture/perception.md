# Perception Architecture

## Scope

Perception covers camera transport, TF normalization, fused point clouds, segmentation, and contact-target prediction.

Relevant packages:

- `camera`
- `sensor_fusion_bringup`
- `pointcloud_fusion`
- `segmentation`
- `twist_propagation`

## Camera and TF Bridge Layer

Key files:

- `src/camera/camera/odom_to_pose_relay.py`
- `src/camera/camera/openvins_odom_tf_relay.py`
- `src/camera/camera/openvins_realsense_tf_bridge_node.py`
- `src/sensor_fusion_bringup/scripts/publish_camera_mounts.py`
- `src/sensor_fusion_bringup/scripts/tf_pipeline_diagnostics.py`

Responsibilities:

- Convert OpenVINS odometry into host-visible TF
- Convert odometry into `/hand_pose`, `/hand_twist`, and `/hand_odom`
- Attach camera mount and grasp-contact related frames into the TF tree
- Diagnose TF connectivity issues during startup and runtime

## Point Cloud Fusion

Main file: `src/pointcloud_fusion/pointcloud_fusion/pointcloud_fusion_node.py`

Inputs:

- head camera cloud
- arm camera cloud
- TF between camera frames and target frame

Outputs:

- `/fused_pointcloud`
- hand-removal marker topic for visualization

Processing stages:

1. Synchronize both clouds.
2. Transform into a common frame, normally `marker_map`.
3. Merge cloud points.
4. Apply distance filtering.
5. Remove hand/arm points using pruning-box logic and mount configuration.
6. Downsample with a voxel filter.
7. Publish the unified cloud.

Important config source:

- `config/prosthesis_config.yaml` under `pointcloud_fusion`

## Segmentation Bridge

Main runtime file: `src/segmentation/segmentation_bridge/segmentation_ros2_node.py`

Role:

- Accept `/fused_pointcloud`
- Use click/contact seeds
- Call an external inference service over HTTP
- Publish `/segmentation/object_cloud`

Important runtime fact:

- The segmentation package contains both model/inference assets and a ROS bridge layer.
- For architecture work, focus first on the `segmentation_bridge` runtime path.

## Twist Propagation and Contact Prediction

Main file: `src/twist_propagation/twist_propagation/twist_propagation_node.py`

Role:

- Consume live hand pose and scene cloud
- Estimate hand twist
- Propagate motion into the cloud
- Detect likely collision/contact
- Publish a segmentation click and predicted contact pose

Important outputs used elsewhere:

- `/twist_propagation/hit_detected`
- `/twist_propagation/collision_distance`
- `/grasp_preshaping/contact_pose`
- `/grasp_preshaping/contact_twist`
- `/segmentation/click_positive`

## Perception Change Checklist

If you change perception behavior, review and likely update:

- `docs/architecture/perception.md`
- `docs/architecture/runtime-entrypoints.md`
- `docs/architecture/packages.md`
- `docs/reference/directory-structure.md`
- `config/prosthesis_config.yaml` references if parameters or topics changed
