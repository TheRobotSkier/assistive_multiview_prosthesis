# grasp_preshaping

This crate currently provides a preshaping solver core and a CLI entrypoint.

## What is implemented

- LUT-driven preshape solving extracted into a reusable planner module.
- Optional AABB filtering of the point cloud.
- Optional direct publication of MuJoCo position command topics using the same command path as manual `ros2 topic pub --once` calls.

## Run examples

Dry-run with full point cloud (default when AABB is not set):

```bash
cargo run --
```

Dry-run with AABB enabled:

```bash
cargo run -- --aabb -0.2 -0.2 -0.2 0.2 0.2 0.2
```

Compute and publish controller commands:

```bash
cargo run -- --publish-commands
```

Compute and publish with AABB:

```bash
cargo run -- --aabb -0.2 -0.2 -0.2 0.2 0.2 0.2 --publish-commands
```

## Current command topics

- /thumb_pos_ff_controller/commands
- /index_pos_ff_controller/commands
- /mrl_pos_ff_controller/commands

## Note

The ROS2 node/service wrapper is the next step. The current implementation starts by hardening the solver core and adds a command-publication bridge compatible with your MuJoCo control topics.
