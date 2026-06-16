# Package Reference

This is the fast package map for everything under `src/`.

## Core Runtime Packages

### `prosthesis_launch`

- Purpose: top-level launch composition for all runtime modes
- Look here first for: what actually starts together
- Key files: `src/prosthesis_launch/launch/*.launch.py`

### `pipeline_manager`

- Purpose: state-machine orchestration
- Key files: `src/pipeline_manager/pipeline_manager/pipeline_manager_node.py`, `src/pipeline_manager/pipeline_manager/simple_pipeline_manager_node.py`

### `pointcloud_fusion`

- Purpose: dual-camera cloud merge and filtering
- Key file: `src/pointcloud_fusion/pointcloud_fusion/pointcloud_fusion_node.py`

### `segmentation`

- Purpose: segmentation inference assets plus ROS bridge runtime
- Key runtime file: `src/segmentation/segmentation_bridge/segmentation_ros2_node.py`

### `twist_propagation`

- Purpose: hand-motion prediction and segmentation target selection
- Key file: `src/twist_propagation/twist_propagation/twist_propagation_node.py`

### `grasp_preshaping`

- Purpose: compute grasp outputs and manage approach control
- Key files: `src/grasp_preshaping/nodes/preshaping_service_bridge_node.cpp`, `src/grasp_preshaping/nodes/grasp_proximity_controller_node.py`

### `force_controller`

- Purpose: force-driven closure and hold control
- Key file: `src/force_controller/force_controller/force_controller_node.py`

### `command_bridge`

- Purpose: translate topic commands to raw hand services
- Key file: `src/command_bridge/command_bridge/command_bridge_node.py`

## Sensing and Tracking Packages

### `camera`

- Purpose: camera-side ROS helpers and TF relays
- Key files: `src/camera/camera/*.py`, `src/camera/launch/*.py`

### `sensor_fusion_bringup`

- Purpose: camera launch/config and TF utilities
- Key files: `src/sensor_fusion_bringup/launch/*.py`, `src/sensor_fusion_bringup/config/*.yaml`, `src/sensor_fusion_bringup/scripts/*.py`

## Human Input and Feedback Packages

### `emg_bridge`

- Purpose: EMG collection, training, inference, and ROS publication
- Key files: `src/emg_bridge/emg_bridge/*.py`, `src/emg_bridge/scripts/*`
- Dedicated EMG container: `docker/Dockerfile.emg`, `docker/docker-compose.yml` service `emg`
- Important entry points:
  - `ros2 run emg_bridge collect_data`
  - `ros2 run emg_bridge train`
  - `ros2 run emg_bridge run_classifier`
  - `ros2 run emg_bridge latency_benchmark`
- Important latency modules:
  - `src/emg_bridge/emg_bridge/latency_analysis.py`
  - `src/emg_bridge/emg_bridge/latency_protocol.py`
  - `src/emg_bridge/emg_bridge/scripts/latency_benchmark.py`
- Latency behavior note: onset detection can look back across the last `N` target-predicting windows and only accepts onset evidence from channels that stay consistently active across that window history

### `haptic_band`

- Purpose: haptic control logic and Bluetooth bridge
- Runtime package name used in launch files: `haptic_bridge`
- The Mia haptic force test bypasses `haptic_controller_node` and publishes the full 8-motor command vector directly to `bridge_node`

## Hardware Interface Packages

### `mia_hand_driver`

- Purpose: raw serial interface to Mia Hand hardware

### `wrist_driver`

- Purpose: Dynamixel wrist driver

### `mia_hand_ros2_control`

- Purpose: ros2_control system interface, controllers, and launch/config support
- Exposes fingertip normal force as joint `effort` and publishes raw `mia_hand_msgs/ForceData` for ros2_control-based force tests

### `mia_hand_msgs`

- Purpose: shared custom messages, services, and actions for hand integration

### `mia_hand_description`

- Purpose: URDF, xacro, visualization assets, and hand model support

## Vendored Or External Stacks

### `open_vins`

- Purpose: upstream OpenVINS stack used by the workspace

### `realsense-ros`

- Purpose: upstream RealSense ROS integration

Treat both as dependency code unless your task explicitly requires editing them.
