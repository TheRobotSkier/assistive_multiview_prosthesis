# Implementation Plan: Step 3 — Topic & TF Bridges

**Beads Issue:** `bridge-nodes`  
**Type:** task  
**Priority:** 1 (can be done in parallel with `rviz-config`; blocks `launch-digital-twin`)  
**Estimated Effort:** Small

---

## Objective

Provide the lightweight glue that connects the real camera stream (`/fused_pointcloud`) to the segmentation node (`/segmentation/input_cloud`) and aligns the camera coordinate frame with the simulation `world` frame via static TF.

---

## Acceptance Criteria

- [ ] A mechanism exists to forward `/fused_pointcloud` → `/segmentation/input_cloud` without data loss or significant latency.
- [ ] The grasp preshaping bridge successfully resolves camera positions via TF lookup to `world` (TF is provided by the camera node, not this step).
- [ ] `publish_initial_commands` parameter is added to `preshaping_service_bridge_node.cpp` so the digital twin can suppress immediate controller commands.

---

## Implementation Details

### 3.1 Pointcloud Relay

**Preferred solution:** `ros2 run topic_tools relay /fused_pointcloud /segmentation/input_cloud`

- `topic_tools relay` is a standard ROS2 utility that subscribes to one topic and republishes on another, preserving message type and QoS.
- It is available in `ros-jazzy-topic-tools`.

**Check availability:**
```bash
apt list --installed 2>/dev/null | grep topic-tools || echo "NOT INSTALLED"
```

If **not installed**, we have two options:

**Option A — Install it:**
Add `ros-${ROS_DISTRO}-topic-tools` to the Dockerfile's apt install list.
- Pro: standard, well-tested.
- Con: requires rebuilding the Docker image (or installing at runtime).

**Option B — Custom relay node:**
Create a minimal Python node (≤30 lines) in `dev/mujoco/nodes/pointcloud_relay_node.py`:
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

class RelayNode(Node):
    def __init__(self):
        super().__init__('pointcloud_relay')
        self.pub = self.create_publisher(PointCloud2, '/segmentation/input_cloud', 10)
        self.create_subscription(PointCloud2, '/fused_pointcloud', self.cb, 10)
    def cb(self, msg):
        self.pub.publish(msg)

def main():
    rclpy.init()
    rclpy.spin(RelayNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
- Pro: no extra apt package, works immediately.
- Con: one more file to maintain.

**Recommendation:** Check if `topic_tools` is already in the image. If yes, use it. If no, add the custom relay node to avoid forcing a Docker rebuild.

### 3.2 Preshaping Bridge `publish_initial_commands` Parameter

Add a boolean parameter to `preshaping_service_bridge_node.cpp` (default `true` for backward compatibility):

```cpp
const bool publish_initial_commands_ = declare_parameter<bool>("publish_initial_commands", true);
```

In `try_handle_direct_request`, guard the immediate `publish_joint_commands()` and wrist pose publication:

```cpp
if (publish_initial_commands_) {
    publish_joint_commands(preshape_thumb, preshape_index, preshape_mrl);
    std_msgs::msg::Float64 wrist_msg;
    wrist_msg.data = ffi_response.wrist_rotation_deg;
    wrist_pose_pub_->publish(wrist_msg);
}
```

When `publish_initial_commands:=false`, the bridge still publishes planner topics (`/grasp_preshaping/*`) so the proximity controller can consume them and send the initial preshape itself.

### 3.3 Proximity Controller Initial Preshape

Modify `grasp_proximity_controller_node.py` so that `_try_commit_plan()` immediately sends the initial partial preshape and wrist rotation:

```python
def _try_commit_plan(self) -> None:
    if (self._buf_closures is not None
            and self._buf_wrist_deg is not None
            and self._buf_hand_frame is not None):
        self._planned_closures = self._buf_closures
        self._planned_wrist_deg = self._buf_wrist_deg
        self._planned_hand_frame = self._buf_hand_frame
        self._buf_closures = None
        self._buf_wrist_deg = None
        self._buf_hand_frame = None
        self._is_near = False
        self.get_logger().info(
            f'New plan committed — closures={self._planned_closures}, '
            f'wrist={self._planned_wrist_deg:.1f}°'
        )
        # Immediately send initial preshape (far mode)
        self._publish_wrist_command(self._planned_wrist_deg)
        self._publish_joint_commands(
            self._partial_factor * self._planned_closures[0],
            self._partial_factor * self._planned_closures[1],
            self._partial_factor * self._planned_closures[2],
        )
```

This ensures the hand begins moving as soon as the plan is available, even before the first control loop tick.

### 3.4 Camera TF (external)

**No action required in this step.** The camera node uses a CharUco board and AprilTag marker to localize itself relative to `world`. The marker location is the `world` frame, and the camera driver publishes the dynamic TF. The digital twin launch does **not** need a static TF publisher for the camera.

---

## Sub-tasks (beads tracking)

1. Check if `topic_tools` is available in the current Docker image.
2. If unavailable, create `dev/mujoco/nodes/pointcloud_relay_node.py`.
3. Add `publish_initial_commands` parameter to `preshaping_service_bridge_node.cpp`.
4. Modify `grasp_proximity_controller_node.py` to send initial preshape on plan commit.
5. Verify relay and planner topics in a running container.
6. Commit.

---

## Notes / Risks

- **Risk:** `cam1_d435_1_color_optical_frame` name may differ if the user changes launch files or serial numbers.
  - **Mitigation:** Make the camera frame name a launch argument with the RealSense default as the fallback.
- **Risk:** TF frame names in the multiview launch vs. the RealSense driver may differ between Humble and Jazzy.
  - **Mitigation:** Verify the actual frame names published by `multiview_full` (they are `cam1_d435_1_color_optical_frame` and `cam2_d435_2_color_optical_frame` as shown in `pointcloud_fusion_node.py`).
- **Risk:** Changing `preshaping_service_bridge_node.cpp` may break existing `full_system_test` launch.
  - **Mitigation:** Default `publish_initial_commands:=true` preserves existing behavior. Only the digital twin launch sets it to `false`.
