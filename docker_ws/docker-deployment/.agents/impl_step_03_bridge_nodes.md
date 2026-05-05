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
- [ ] A static TF from `world` to `cam1_d435_1_color_optical_frame` is published, with transform parameters exposed as launch arguments.
- [ ] If a second camera is used, a second static TF from `world` to `cam2_d435_2_color_optical_frame` is also published (or documented as manual setup).
- [ ] The grasp preshaping bridge successfully resolves camera positions via TF lookup to `world`.

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

### 3.2 Camera Static TF

The grasp preshaping bridge (`preshaping_service_bridge_node.cpp`) looks up camera frames in TF relative to `world`:
```cpp
tf_buffer_->lookupTransform("world", frame, tf2::TimePointZero);
```

The RealSense ROS driver publishes internal camera TFs (e.g. `cam1_d435_1_link` → `cam1_d435_1_color_optical_frame`), but it does **not** know about `world`.

We must publish:
```
world → cam1_d435_1_color_optical_frame
```

**In the launch file:**
```python
Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments=[
        camera_world_x, camera_world_y, camera_world_z,
        camera_world_qx, camera_world_qy, camera_world_qz, camera_world_qw,
        'world', 'cam1_d435_1_color_optical_frame',
    ],
    name='cam1_world_tf',
)
```

**If two cameras are used**, add a second static TF:
```python
Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments=[
        camera2_world_x, ..., 'world', 'cam2_d435_2_color_optical_frame',
    ],
    name='cam2_world_tf',
    condition=IfCondition(use_camera2),
)
```

For the first iteration, supporting one camera is sufficient. The second camera can be added later or configured manually by the user.

**Determining the transform:**
Users must measure or estimate the physical camera mounting pose relative to their chosen `world` origin. This is inherently site-specific. The launch arguments make it configurable without code changes.

### 3.3 Verification

Run:
```bash
ros2 topic echo /segmentation/input_cloud --once
```
While `multiview_full` is publishing `/fused_pointcloud`. You should see pointcloud data.

Run:
```bash
ros2 run tf2_ros tf2_echo world cam1_d435_1_color_optical_frame
```
You should see the configured translation and rotation.

---

## Sub-tasks (beads tracking)

1. Check if `topic_tools` is available in the current Docker image.
2. If unavailable, create `dev/mujoco/nodes/pointcloud_relay_node.py`.
3. Add static TF publisher node to the launch file with launch arguments.
4. Verify relay and TF in a running container.
5. Commit.

---

## Notes / Risks

- **Risk:** `cam1_d435_1_color_optical_frame` name may differ if the user changes launch files or serial numbers.
  - **Mitigation:** Make the camera frame name a launch argument with the RealSense default as the fallback.
- **Risk:** TF frame names in the multiview launch vs. the RealSense driver may differ between Humble and Jazzy.
  - **Mitigation:** Verify the actual frame names published by `multiview_full` (they are `cam1_d435_1_color_optical_frame` and `cam2_d435_2_color_optical_frame` as shown in `pointcloud_fusion_node.py`).
