# grasp_preshaping

This crate provides the preshaping solver core and a ROS2 Trigger service node.

## ROS2 service node

- Binary: `ros_node` (built with feature `ros`)
- Service: `/grasp_preshaping/compute_grasp`
- Type: `std_srvs/srv/Trigger`

On each service call, the node:

1. Uses the latest cached ROS messages from:
- `/hand_pose` (`geometry_msgs/msg/PoseStamped`)
- `/hand_twist` (`geometry_msgs/msg/TwistWithCovarianceStamped`)
- `/segmented_object_cloud` (`sensor_msgs/msg/PointCloud2`)
2. Runs the preshaping pipeline.
3. Publishes controller commands to:
- `/thumb_pos_ff_controller/commands`
- `/index_pos_ff_controller/commands`
- `/mrl_pos_ff_controller/commands`

## Build and run (container)

Build the ROS node with ROS feature enabled:

```bash
cargo build --release --features ros --bin ros_node
```

Run the node:

```bash
cargo run --release --features ros --bin ros_node
```

Trigger one preshaping request:

```bash
ros2 service call /grasp_preshaping/compute_grasp std_srvs/srv/Trigger {}
```

## Notes

- This package no longer uses the old planner CLI flow used by MuJoCo wrappers.
- If required inputs are missing, the Trigger response returns `success=false` with a descriptive message.
