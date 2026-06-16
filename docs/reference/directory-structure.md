# Directory Structure Reference

Use this as the primary navigation map for the repository.

## Top Level

- `config/`: central runtime configuration, network/timesync config, scenario config
- `docker/`: container definitions and compose files
- `docs/`: canonical architecture and navigation documentation
- `logs/`: historical log captures, not source-of-truth docs
- `models/`: model assets used by runtime or experiments
- `data/`: persisted EMG recordings and latency benchmark outputs
- `rviz/`: RViz configuration
- `scripts/`: host and workspace utility scripts, smoke tests, setup scripts
- `src/`: ROS packages and vendored dependencies
- `tests/`: higher-level verification assets outside package-local tests
- `Makefile`: host workflows
- `Makefile.workspace`: in-container workflows

## `src/` Packages By Concern

### Runtime orchestration and execution

- `src/prosthesis_launch/`
- `src/pipeline_manager/`
- `src/grasp_preshaping/`
- `src/force_controller/`
- `src/command_bridge/`

### Perception and targeting

- `src/camera/`
- `src/sensor_fusion_bringup/`
- `src/pointcloud_fusion/`
- `src/segmentation/`
- `src/twist_propagation/`

### Input and feedback

- `src/emg_bridge/`
- `src/haptic_band/`

### Hardware interface and models

- `src/mia_hand_driver/`
- `src/wrist_driver/`
- `src/mia_hand_ros2_control/`
- `src/mia_hand_msgs/`
- `src/mia_hand_description/`

### Vendored dependencies

- `src/open_vins/`
- `src/realsense-ros/`

## Where To Look For Common Tasks

### Change full runtime composition

- Start: `src/prosthesis_launch/launch/pipeline.launch.py`
- Then: `config/prosthesis_config.yaml`

### Change state-machine behavior

- Start: `src/pipeline_manager/pipeline_manager/pipeline_manager_node.py`

### Change EMG-only execution mode

- Start: `src/prosthesis_launch/launch/simple_emg_grasp.launch.py`
- Then: `src/pipeline_manager/pipeline_manager/simple_pipeline_manager_node.py`

### Change EMG collection, training, inference, or latency benchmarking

- Start: `src/emg_bridge/`
- Live benchmark runner: `src/emg_bridge/emg_bridge/scripts/latency_benchmark.py`
- Latency analysis logic: `src/emg_bridge/emg_bridge/latency_analysis.py`
- Host workflow wrapper: `scripts/emg_latency_workflow.sh`
- Host command: `make emg-latency-workflow`
- Dedicated EMG container: `docker/Dockerfile.emg`, compose service `emg`

### Change point cloud fusion

- Start: `src/pointcloud_fusion/pointcloud_fusion/pointcloud_fusion_node.py`
- Then: `src/sensor_fusion_bringup/config/camera_mounts.yaml`

### Change segmentation integration

- Start: `src/segmentation/segmentation_bridge/segmentation_ros2_node.py`
- Then: launch files and inference URL config

### Change target selection before segmentation

- Start: `src/twist_propagation/twist_propagation/twist_propagation_node.py`

### Change grasp planning outputs

- Start: `src/grasp_preshaping/nodes/preshaping_service_bridge_node.cpp`
- Then: Rust code under `src/grasp_preshaping/src/`

### Change approach behavior near the object

- Start: `src/grasp_preshaping/nodes/grasp_proximity_controller_node.py`

### Change raw hand command flow

- Start: `src/command_bridge/command_bridge/command_bridge_node.py`
- Then: `src/mia_hand_driver/`

### Change force-based closure and hold

- Start: `src/force_controller/force_controller/force_controller_node.py`
- Then: `src/mia_hand_ros2_control/` if controller ownership or controller-manager integration changes

### Change wrist control

- Start: `src/wrist_driver/wrist_driver/wrist_driver_node.py`

### Change haptics

- Start: `src/haptic_band/haptic_bridge/haptic_controller_node.py`
- Then: `src/haptic_band/haptic_bridge/bridge_node.py`

### Change build/test workflow

- Start: `Makefile`, `Makefile.workspace`, `scripts/run_tests.sh`

## Documentation Cross-Reference

- Overview: `docs/architecture/overview.md`
- Runtime composition: `docs/architecture/runtime-entrypoints.md`
- Perception: `docs/architecture/perception.md`
- Control and actuation: `docs/architecture/control-and-actuation.md`
- Package map: `docs/architecture/packages.md`
- Validation workflows: `docs/reference/validation-and-workflows.md`
