# grasp_preshaping

This crate currently provides a preshaping solver core and a CLI entrypoint.

## What is implemented

- LUT-driven preshape solving extracted into a reusable planner module.
- Optional AABB filtering of the point cloud.
- Configurable execution frequency and iteration count.
- Dual point cloud input mode: file (`.xyz`) or ROS2 PointCloud2 topic.
- Global fingertip offset with two parameters (distal/proximal and palmar/dorsal), applied once in finger-local frame.
- Optional direct publication of MuJoCo position command topics using the same command path as manual `ros2 topic pub --once` calls.

## Run examples

Dry-run with full point cloud (default when AABB is not set):

```bash
cargo run --
```

Run at 2 Hz for 10 iterations:

```bash
cargo run -- --frequency-hz 2 --iterations 10
```

Dry-run with AABB enabled:

```bash
cargo run -- --aabb -0.2 -0.2 -0.2 0.2 0.2 0.2
```

Apply fingertip offsets in finger-local frame:

```bash
cargo run -- --offset-distal-proximal 0.01 --offset-palmar-dorsal -0.005
```

Use ROS PointCloud2 input mode (expects a publisher on the topic):

```bash
cargo run -- --mode ros --pointcloud-topic /segmented_object_cloud --frequency-hz 5 --iterations 0
```

Compute and publish controller commands:

```bash
cargo run -- --publish-commands
```

Compute and publish with AABB:

```bash
cargo run -- --aabb -0.2 -0.2 -0.2 0.2 0.2 0.2 --publish-commands
```

Compute, offset, and publish at frequency:

```bash
cargo run -- --frequency-hz 5 --iterations 0 --offset-distal-proximal 0.005 --publish-commands
```

## CLI summary

- `--mode file|ros`: point cloud source mode.
- `--cloud PATH`: `.xyz` path for file mode.
- `--pointcloud-topic TOPIC`: ROS PointCloud2 topic for ros mode.
- `--frequency-hz VALUE`: execution frequency.
- `--iterations N`: iteration count (`0` means run forever).
- `--aabb xmin ymin zmin xmax ymax zmax`: optional AABB filter.
- `--offset-distal-proximal VALUE`: local +X fingertip offset in meters.
- `--offset-palmar-dorsal VALUE`: local +Z fingertip offset in meters.
- `--publish-commands`: send commands to MuJoCo controller topics.

## Current command topics

- /thumb_pos_ff_controller/commands
- /index_pos_ff_controller/commands
- /mrl_pos_ff_controller/commands

## Note

ROS mode currently pulls one PointCloud2 message per cycle via `ros2 topic echo --once` and converts it internally. This keeps standard ROS2 message compatibility while the dedicated in-process ROS node wrapper is built.
