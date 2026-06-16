# System Overview

The workspace is a ROS 2 Jazzy system for a multiview prosthesis pipeline. Its main runtime path starts in `src/prosthesis_launch/launch/pipeline.launch.py`, which composes perception, planning, control, and actuation into one orchestrated pipeline.

## Runtime Flow

1. Camera and tracking data arrive from RealSense and OpenVINS topics.
2. Host-side camera bridge nodes normalize TF and hand-pose signals.
3. `pointcloud_fusion_node` transforms and merges the two camera clouds into `/fused_pointcloud`.
4. `twist_propagation_node` predicts hand motion into the fused cloud and publishes a likely contact target.
5. `segmentation_ros2_node` uses the click/contact seed plus `/fused_pointcloud` to obtain `/segmentation/object_cloud`.
6. `pipeline_manager_node` drives the system state machine and triggers grasp planning.
7. `preshaping_service_bridge_node` converts the segmented object plus tracked hand state into grasp outputs.
8. `grasp_proximity_controller_node.py` applies partial closure and wrist approach control until the hand is near contact.
9. `force_controller_node` takes over during grasp closure and hold.
10. Wrist, Mia Hand, and haptic nodes execute hardware outputs and feedback.

## Architectural Centers

- Orchestration center: `src/pipeline_manager/pipeline_manager/pipeline_manager_node.py`
- Main launch composition: `src/prosthesis_launch/launch/pipeline.launch.py`
- Central runtime config: `config/prosthesis_config.yaml`
- Main perception cloud: `/fused_pointcloud`
- Main segmented object output: `/segmentation/object_cloud`
- Main pipeline state topics: `/pipeline/state`, `/pipeline/state_name`

## Main State Machine

`pipeline_manager_node` defines the operational states:

- `IDLE`
- `TWISTING`
- `SEGMENTING`
- `PLANNING`
- `APPROACHING`
- `GRASPING`
- `HOLDING`
- `VOLITIONAL`
- `RELEASING`

The nominal path is:

`IDLE -> TWISTING -> SEGMENTING -> PLANNING -> APPROACHING -> GRASPING -> HOLDING -> VOLITIONAL -> RELEASING -> IDLE`

This is the most important behavioral contract in the repository. Any agent changing pipeline behavior should review both:

- `src/pipeline_manager/pipeline_manager/pipeline_manager_node.py`
- `src/prosthesis_launch/launch/pipeline.launch.py`

## Execution Modes

- Full hardware pipeline: `src/prosthesis_launch/launch/pipeline.launch.py`
- EMG-only simplified execution pipeline: `src/prosthesis_launch/launch/simple_emg_grasp.launch.py`
- EMG/haptic Mia force test with no cameras: `src/prosthesis_launch/launch/mia_haptic_force_test.launch.py`
- Mock pipeline: `src/prosthesis_launch/launch/mock.launch.py`
- Digital twin: `src/prosthesis_launch/launch/digital_twin.launch.py`
- Grasp test and twist test modes: files in `src/prosthesis_launch/launch/`

## Source-of-Truth Files

When documentation and code disagree, trust these first:

- `config/prosthesis_config.yaml`
- `src/prosthesis_launch/launch/*.launch.py`
- `src/*/package.xml`
- Node implementations in `src/*/`
