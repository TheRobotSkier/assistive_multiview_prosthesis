# Runtime Entrypoints

## Primary Launch Files

Located in `src/prosthesis_launch/launch/`.

### `pipeline.launch.py`

Primary hardware pipeline.

Starts:

- `pipeline_manager_node`
- `mia_hand_driver_node`
- `command_bridge_node`
- `wrist_driver_node`
- `run_classifier`
- `emg_grasp_controller`
- `bridge_node` and `haptic_controller_node`
- `openvins_realsense_tf_bridge_node`
- `pointcloud_fusion_node`
- `odom_to_pose_relay`
- `openvins_odom_tf_relay`
- `segmentation_ros2_node`
- `twist_propagation_node`
- `preshaping_service_bridge_node`
- `grasp_proximity_controller_node.py`
- `force_controller_node`
- optional `rviz2`

Key launch arguments include:

- `config_file`
- `camera`
- `camera_tf_bridge`
- `mia_hand`
- `wrist`
- `emg`
- `haptic`
- `mia_serial_port`
- `wrist_serial_port`
- `target_frame`
- `odom_topic`
- `cam1_topic`
- `cam2_topic`
- `camera_mount`
- `mounts_config`
- `inference_url`
- `roi_radius`
- `tf_diagnostics`
- `require_dual_openvins`

### `simple_emg_grasp.launch.py`

Execution-only pipeline with no perception stack.

Starts:

- `simple_pipeline_manager_node`
- `mia_hand_driver_node`
- `command_bridge_node`
- `wrist_driver_node`
- `run_classifier`
- `force_controller_node`
- optional `rviz2`

Use this when changing EMG-triggered closing behavior without needing cameras, segmentation, or grasp planning.

### `mia_haptic_force_test.launch.py`

Bench-test pipeline for the Mia Hand, wrist, EMG classifier, and Vibro8 haptics with no perception stack.

Starts:

- Mia Hand ros2_control stack
- `run_classifier`
- optional `wrist_driver_node`
- optional `haptic_bridge_node`
- `scripts/mia_haptic_force_test.py`

Use this when testing wrist rotation, force closure, force-hold target adjustment, POWER toggling between force and wrist control, OPEN-stop behavior, haptic mappings, and CSV capture without cameras or the production pipeline manager.

### Other Launch Files

- `mock.launch.py`: mock pipeline for non-hardware use
- `digital_twin.launch.py`: digital twin mode
- `grasp_test.launch.py`: legacy grasp testing path
- `emg_grasp_test.launch.py`: EMG-focused test path
- `mia_haptic_force_test.launch.py`: EMG/haptic force test with CSV logging
- `tf_bridge.launch.py`: TF bridge only
- `twist_propagation_test.launch.py`: twist/fusion-related testing

## Central Configuration

Primary file: `config/prosthesis_config.yaml`

This file has two roles:

- ROS parameter source for nodes via namespaced sections like `pipeline_manager.ros__parameters`
- Flat reference store for scripts and external readers

If a node parameter is added or renamed, update both the node implementation and the correct config section here.

## Host and Container Entrypoints

### Host-level

- `Makefile`
- `docker/`
- `scripts/`

Main commands:

- `make dev`
- `make shell`
- `make run`
- `make run-emg-grasp`
- `make test-grasp`
- `make test`

### In-container workspace

- `Makefile.workspace`

Main targets:

- `make build`
- `make test`
- `make pipeline`
- `make run`
- `make run-emg-grasp`
- `make test-grasp`
- `make camera-test`
- `make tonight-*`
