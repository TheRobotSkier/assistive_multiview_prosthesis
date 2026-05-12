# Pointcloud Streaming: Jetson → Host over Ethernet

## Summary

Cross-Ethernet ROS 2 communication already works: Jetson publishes pointclouds,
host sees them. Default FastRTPS (no config). The remaining work: reconcile
RMW on host containers, handle bandwidth, bridge topic names, and add Makefile
targets.

**Status:** Core DDS path works. Need to formalize and optimize.

---

## Step 1: Verify DDS Health (Diagnostic Commands)

Run on **host** while Jetson bringup is running.

### 1a. Check topic visibility
```bash
# From host terminal (no container):
ros2 topic list 2>/dev/null | grep -E 'head|arm'

# Expected:
#   /head/d435_head/depth/color/points
#   /arm/d435_arm/depth/color/points
#   /head/d435_head/color/image_raw
#   /arm/d435_arm/color/image_raw
#   /odometry/filtered
#   /tf
#   /tf_static
```

### 1b. Check pointcloud data rate
```bash
ros2 topic hz /head/d435_head/depth/color/points
ros2 topic hz /arm/d435_arm/depth/color/points
# Expected: 5–15 Hz (depends on depth profile)
```

### 1c. Check bandwidth
```bash
ros2 topic bw /head/d435_head/depth/color/points
ros2 topic bw /arm/d435_arm/depth/color/points
# 424x240@5fps → ~8 MB/s per camera
# 424x240@15fps → ~24 MB/s per camera
```

### 1d. Check RMW in use
```bash
ros2 topic info /head/d435_head/depth/color/points 2>/dev/null
# Look for "Node name" — both sides should be visible
```

### 1e. Check TF tree
```bash
ros2 run tf2_tools view_frames 2>/dev/null
# Verify head_d435_head_depth_optical_frame appears
```

### Gotchas
- If no topics visible: check Ethernet cable, verify Jetson bringup is running
- If topics visible but no data: check pointcloud frequency, `ros2 topic hz`
- If only some nodes visible: RMW discovery issue (see Step 2)

---

## Step 2: RMW Alignment

### Decision: Default FastRTPS on both sides for Ethernet

| Machine    | Current RMW          | Action                          |
|------------|----------------------|---------------------------------|
| Jetson     | Default (FastRTPS)   | **No change** — works for Ethernet |
| Host RViz  | Default (FastRTPS)   | **No change** — `make rviz` already correct |
| Host `prosthesis` container | CycloneDDS  | **Change to FastRTPS** for cross-device profiles |

### Why

- Ethernet multicast works with default FastRTPS, no config needed
- CycloneDDS with peer discovery was a WiFi fallback — WiFi adapter doesn't support DDS multicast anyway
- The `ros:jazzy-desktop` image includes FastRTPS by default (via `rmw_fastrtps_cpp`)

### Changes to docker-compose.yml

For profiles that consume Jetson pointclouds (`grasp_test`, `digital_twin`), they already use `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` — keep this.

For the `rviz` target in Makefile, ensure NO `RMW_IMPLEMENTATION` is set (uses default FastRTPS). **Already correct.**

### Verification
```bash
# On host, inside container:
echo $RMW_IMPLEMENTATION
# Should be empty or "rmw_fastrtps_cpp"

ros2 topic list | grep -c head
# Should be > 0
```

---

## Step 3: Bandwidth Management

### Current Jetson Setup (from ROBOTLAB_ARCHITECTURE.md)
- Depth: **424x240@15fps**
- Color: 640x480@15fps
- Two cameras

### Bandwidth Calculation

| Resolution  | FPS | Bytes/msg (approx) | MB/s per cam | MB/s both cams |
|-------------|-----|--------------------|---------------|-----------------|
| 424x240     | 15  | 1,630,000          | 24.5         | **49.0**        |
| 424x240     | 6   | 1,630,000          | 9.8          | 19.6            |
| 424x240     | 5   | 1,630,000          | 8.2          | 16.3            |
| 640x480     | 6   | 3,690,000          | 22.1         | 44.3            |
| 640x480     | 5   | 3,690,000          | 18.5         | 36.9            |

Gigabit Ethernet theoretical: ~125 MB/s. Practical: ~80–100 MB/s.
At 424x240@5fps, two cameras use ~16 MB/s — safe margin.

### Recommendation: 424x240@5fps depth pointcloud

This is set in the Jetson's `d435_cameras.yaml` config (in jetson-docker repo).

**Config snippet** (in `d435_cameras.yaml`):
```yaml
/head/d435_head:
  ros__parameters:
    serial_no: "829212072207"
    depth_module.depth_profile: "424x240x5"
    rgb_camera.color_profile: "640x480x15"
    pointcloud.enable: true
    pointcloud.stream_filter: 2        # downsampling factor (2 = every 2nd pixel)
    pointcloud.ordered_pc: false        # unstructured = smaller messages
    align_depth.enable: true
    enable_color: true
    enable_infra1: false
    enable_infra2: false
    initial_reset: false

/arm/d435_arm:
  ros__parameters:
    serial_no: "827112072033"
    depth_module.depth_profile: "424x240x5"
    rgb_camera.color_profile: "640x480x15"
    pointcloud.enable: true
    pointcloud.stream_filter: 2
    pointcloud.ordered_pc: false
    align_depth.enable: true
    enable_color: true
    enable_infra1: false
    enable_infra2: false
    initial_reset: false
```

### Additional bandwidth options (if needed later)

| Option | How | Savings |
|--------|-----|---------|
| `stream_filter: 2` | Every 2nd pixel in X and Y -> 1/4 points | 75% reduction |
| `stream_filter: 4` | Every 4th pixel -> 1/16 points | 93% reduction |
| `ordered_pc: false` | Unstructured = no invalid points padding | ~10-30% |
| Single camera only | Only head camera active | 50% reduction |
| ROS `compressed` transport | Happens outside realsense node | Not implemented yet |

**Note:** ROS 2 `compressed` transport for PointCloud2 is NOT built-in. Would need a separate `pointcloud_transport` node (like `cv_bridge` for images). Not needed at current bandwidth levels.

---

## Step 4: QoS Settings

### realsense2_camera_node publishes with:
```python
# Default ROS 2 QoS:
Reliability: RELIABLE
Durability: VOLATILE
History: KEEP_LAST (depth=5)
```

### RViz pointcloud display expects:
```yaml
# From rviz/robotlab_cameras.rviz:
Topic:
  Depth: 5
  Durability Policy: Volatile
  Reliability Policy: Reliable
```

### Compatibility: **OK.** RViz QoS matches the publisher's defaults exactly.

### Host-side subscriber QoS

Any host-side Python subscriber (relay node, fuser node) should keep default (RELIABLE, KEEP_LAST/5). Do not switch to BEST_EFFORT without testing — RELIABLE is important on Ethernet to avoid DDS retransmission storms.

---

## Step 5: Jetson Launch Configuration

### Prerequisites (SSH into Jetson)
```bash
ssh robotlab@192.168.100.2   # or ssh robotlab (WiFi alias)
# password: robotlab
```

### Current bringup command
```bash
cd ~/jetson_ws/jetson-docker
make up
```

This launches Docker container with:
- Both D435 cameras (depth 424x240, pointclouds enabled)
- Both GY-91 IMUs (15 Hz)
- EKF odometry fusion

### Verify pointcloud publishing
```bash
# Inside the Jetson Docker container:
make shell
source /miahand_ws/install/setup.bash
ros2 topic list | grep points
# Expected: /head/d435_head/depth/color/points, /arm/d435_arm/depth/color/points

ros2 topic hz /head/d435_head/depth/color/points
# Expected: ~5 Hz (or configured FPS)
```

### NEON pointcloud fix (already applied)
The NEON fix is in the Jetson's `robotlab_bringup.launch.py`. It sets `pointcloud.enable: true` in the camera config YAML (`d435_cameras.yaml`). No additional delay needed because the config is loaded at node startup.

**If pointclouds don't appear:** The fix may need to be reapplied after config changes:
```bash
# Check current setting
ros2 param get /head/d435_head pointcloud.enable
# If false, set it:
ros2 param set /head/d435_head pointcloud.enable true
ros2 param set /arm/d435_arm pointcloud.enable true
```

---

## Step 6: Host-Side RViz Viewing

### Option A: `make rviz` (recommended)
```bash
cd ~/Drive/AAU/P8/grasping/multiview_prosthesis
make rviz
```
Launches podman container with:
- Default FastRTPS (matches Jetson Ethernet DDS)
- Uses `rviz/robotlab_cameras.rviz` config
- Displays both Head and Arm PointCloud2 topics
- Fixed frame: `head_d435_head_depth_optical_frame`

### Option B: Manual podman command
```bash
podman run --rm -it --name rviz-robotlab \
    --network host \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v $(pwd)/rviz/robotlab_cameras.rviz:/rviz_config.rviz:ro \
    docker.io/osrf/ros:jazzy-desktop \
    bash -c 'source /opt/ros/jazzy/setup.bash && rviz2 -d /rviz_config.rviz'
```

### Gotchas
- `make rviz` references `localhost/rviz-robotlab` image — must exist. Re-tag with:
  ```bash
  podman tag docker.io/osrf/ros:jazzy-desktop localhost/rviz-robotlab
  ```
- X11 must work: `echo $DISPLAY` should show `:0` or similar
- Fixed frame `head_d435_head_depth_optical_frame` must exist in TF tree — wait ~5s after Jetson bringup before launching RViz

---

## Step 7: Topic Name Bridge / Relay Node

### Problem
| Ecosystem | Head Pointcloud Topic | Arm Pointcloud Topic |
|-----------|----------------------|----------------------|
| Jetson publishes | `/head/d435_head/depth/color/points` | `/arm/d435_arm/depth/color/points` |
| Host pipeline expects | `/cam1/d435_1/depth/color/points` | `/cam2/d435_2/depth/color/points` |

The host pipeline (grasp_test, digital_twin) has hardcoded topic names.

### Solution: Robotlab pointcloud relay node

A Python node on host that bridges Jetson topics to host-expected topic names.
Optionally throttles bandwidth.

**File to create:** `src/camera/camera/robotlab_pointcloud_relay.py`

```python
"""Bridges Jetson robotlab pointcloud topics to host-local topic names.

Subscribes to:
  /head/d435_head/depth/color/points  ->  /cam1/d435_1/depth/color/points
  /arm/d435_arm/depth/color/points    ->  /cam2/d435_2/depth/color/points
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


class RobotlabPointcloudRelay(Node):
    def __init__(self):
        super().__init__("robotlab_pointcloud_relay")

        self.declare_parameter(
            "head_topic", "/head/d435_head/depth/color/points"
        )
        self.declare_parameter(
            "arm_topic", "/arm/d435_arm/depth/color/points"
        )
        self.declare_parameter(
            "cam1_topic", "/cam1/d435_1/depth/color/points"
        )
        self.declare_parameter(
            "cam2_topic", "/cam2/d435_2/depth/color/points"
        )
        self.declare_parameter("throttle_hz", 0.0)  # 0 = no throttle

        head_topic = self.get_parameter("head_topic").value
        arm_topic = self.get_parameter("arm_topic").value
        cam1_topic = self.get_parameter("cam1_topic").value
        cam2_topic = self.get_parameter("cam2_topic").value
        self._throttle_hz = self.get_parameter("throttle_hz").value

        self._pub_cam1 = self.create_publisher(PointCloud2, cam1_topic, 10)
        self._pub_cam2 = self.create_publisher(PointCloud2, cam2_topic, 10)

        self.create_subscription(PointCloud2, head_topic, self._cb_head, 10)
        self.create_subscription(PointCloud2, arm_topic, self._cb_arm, 10)

        self._last_pub_cam1 = self.get_clock().now()
        self._last_pub_cam2 = self.get_clock().now()
        self._throttle_ns = 0
        if self._throttle_hz > 0:
            self._throttle_ns = int(1e9 / self._throttle_hz)

        self._stats = {"head": 0, "arm": 0}
        self.create_timer(10.0, self._log_stats)
        self.get_logger().info(
            f"Relay: {head_topic} -> {cam1_topic}, {arm_topic} -> {cam2_topic}"
            + (f", throttle={self._throttle_hz}Hz" if self._throttle_hz > 0 else "")
        )

    def _throttle_ok(self, last_time):
        if self._throttle_ns == 0:
            return True
        now = self.get_clock().now()
        return (now - last_time).nanoseconds >= self._throttle_ns

    def _cb_head(self, msg: PointCloud2):
        self._stats["head"] += 1
        if self._throttle_ok(self._last_pub_cam1):
            self._pub_cam1.publish(msg)
            self._last_pub_cam1 = self.get_clock().now()

    def _cb_arm(self, msg: PointCloud2):
        self._stats["arm"] += 1
        if self._throttle_ok(self._last_pub_cam2):
            self._pub_cam2.publish(msg)
            self._last_pub_cam2 = self.get_clock().now()

    def _log_stats(self):
        self.get_logger().info(
            f"Relay stats — head: {self._stats['head']}, arm: {self._stats['arm']}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = RobotlabPointcloudRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
```

### Register in setup.py

**File to edit:** `src/camera/setup.py`

Add to console_scripts:
```python
"robotlab_pointcloud_relay = camera.robotlab_pointcloud_relay:main",
```

### Usage
```bash
# Basic relay (all clouds, no throttle)
ros2 run camera robotlab_pointcloud_relay

# Throttle to 5 Hz
ros2 run camera robotlab_pointcloud_relay --ros-args -p throttle_hz:=5.0
```

### Design note
Not using the existing `pointcloud_fuser_node` because:
1. Fuser publishes to a single `/fused_pointcloud` topic (loses which camera originated each cloud)
2. The pipeline needs per-camera clouds for proper TF transforms
3. The relay preserves individual topic identities

---

## Step 8: Makefile Targets

### Host Makefile additions (edit `Makefile`)

```makefile
# --- Robotlab Pointcloud Relay -----------------------------------
relay-start:
	@echo "Starting robotlab pointcloud relay..."
	@/bin/bash -c '\
		source /prosthesis_ws/install/setup.bash 2>/dev/null || true; \
		source /opt/ros/jazzy/setup.bash 2>/dev/null || true; \
		ros2 run camera robotlab_pointcloud_relay --ros-args -p throttle_hz:=5.0 &'

relay-kill:
	pkill -f robotlab_pointcloud_relay 2>/dev/null || true

# Update rviz to ensure localhost/rviz-robotlab image exists
rviz:
	@echo "Launching RViz on host (connects to robotlab via Ethernet ROS network)"
	@test -f rviz/robotlab_cameras.rviz || { echo "Missing rviz/robotlab_cameras.rviz"; exit 1; }
	@podman image exists localhost/rviz-robotlab 2>/dev/null || \
		podman tag docker.io/osrf/ros:jazzy-desktop localhost/rviz-robotlab
	podman run --rm -d --name rviz-robotlab \
		--network host \
		-e DISPLAY=$(DISPLAY) \
		-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
		-v $(CURDIR)/rviz/robotlab_cameras.rviz:/rviz_config.rviz:ro \
		localhost/rviz-robotlab \
		bash -c 'source /opt/ros/jazzy/setup.bash && rviz2 -d /rviz_config.rviz'
	@sleep 3
	@echo "RViz container started (rviz-robotlab). Kill with: podman kill rviz-robotlab"

rviz-kill:
	podman kill rviz-robotlab 2>/dev/null || true
	podman rm rviz-robotlab 2>/dev/null || true

# Full robotlab pipeline: relay + RViz
robotlab-view: relay-start rviz
	@echo "Robotlab pointcloud viewing active. Stop with: make robotlab-stop"

robotlab-stop: relay-kill rviz-kill
	@echo "Robotlab viewing stopped."
```

### Jetson Makefile (documentation only — runs on Jetson, already exists)

```makefile
# On Jetson (~/jetson_ws/jetson-docker/Makefile) — already exists
up:
	docker compose -f docker_ws/docker-deployment/docker-compose.yml \
		run --rm miahand_ros2 \
		bash -c 'source /miahand_ws/install/setup.bash && \
		ros2 launch sensor_fusion_bringup robotlab_bringup.launch.py'

kill:
	docker compose -f docker_ws/docker-deployment/docker-compose.yml down

shell:
	docker compose -f docker_ws/docker-deployment/docker-compose.yml \
		run --rm miahand_ros2 bash
```

---

## Step 9: Full Integration Test Checklist

Run these in order:

### 1. Jetson side (SSH robotlab)
```bash
cd ~/jetson_ws/jetson-docker
make up
# Wait ~10s for cameras to initialize and pointclouds to start
```

### 2. Verify cross-network (host terminal)
```bash
ros2 topic list | grep -E 'head|arm'
ros2 topic hz /head/d435_head/depth/color/points
ros2 topic bw /head/d435_head/depth/color/points
# Confirm data flowing at expected rate
```

### 3. Host RViz (host terminal)
```bash
cd ~/Drive/AAU/P8/grasping/multiview_prosthesis
make rviz
# Confirm both pointclouds render correctly in RViz
```

### 4. Host relay (optional — needed for pipeline integration)
```bash
# In one terminal:
make relay-start
# Verify relayed topics:
ros2 topic list | grep cam
# Expected: /cam1/d435_1/depth/color/points, /cam2/d435_2/depth/color/points
```

### 5. Full pipeline test (optional)
```bash
# With relay running, start the grasp test with mock cameras disabled
podman compose --profile grasp_test up grasp_test
```

---

## Appendix A: RMW Summary Table

| Container                | RMW for Ethernet (default) | RMW for WiFi (deprecated) |
|--------------------------|---------------------------|--------------------------|
| Jetson miahand_ros2      | Default FastRTPS          | --                       |
| Host prosthesis          | Default FastRTPS (change) | rmw_cyclonedds_cpp       |
| Host grasp_test          | rmw_fastrtps_cpp (OK)     | --                       |
| Host digital_twin        | rmw_fastrtps_cpp (OK)     | --                       |
| Host rviz-robotlab       | Default FastRTPS (OK)     | --                       |

## Appendix B: Topic Reference

### Jetson topics (published)
| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/head/d435_head/depth/color/points` | PointCloud2 | 5-15 Hz | Head camera depth+color aligned |
| `/arm/d435_arm/depth/color/points` | PointCloud2 | 5-15 Hz | Arm camera depth+color aligned |
| `/head/d435_head/color/image_raw` | Image | 15 Hz | Head RGB image |
| `/arm/d435_arm/color/image_raw` | Image | 15 Hz | Arm RGB image |
| `/cam0/data_raw` | Imu | 15 Hz | Head IMU |
| `/cam1/data_raw` | Imu | 15 Hz | Arm IMU |
| `/odometry/filtered` | Odometry | 15 Hz | EKF fused odometry |

### Host relayed topics (published by relay node)
| Topic | Source |
|-------|--------|
| `/cam1/d435_1/depth/color/points` | from `/head/d435_head/depth/color/points` |
| `/cam2/d435_2/depth/color/points` | from `/arm/d435_arm/depth/color/points` |

## Appendix C: Troubleshooting

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| No topics from Jetson on host | Ethernet not connected or IP mismatch | `ping 192.168.100.2`; check cable |
| Topics visible but no data | Pointclouds not enabled on Jetson | `ros2 param set /head/d435_head pointcloud.enable true` |
| RViz shows "No transform from X to Y" | Fixed frame mismatch or TF not publishing | Wait 5s; verify frame in `ros2 run tf2_tools view_frames` |
| High bandwidth / laggy RViz | Too many points per second | Lower FPS in `d435_cameras.yaml`, use `stream_filter: 2` |
| Relay node not finding topics | Jetson bringup not running or different RMW | Verify `ros2 topic list` on host; ensure both sides use default FastRTPS |
| `make rviz` fails with image not found | `localhost/rviz-robotlab` doesn't exist | `podman tag docker.io/osrf/ros:jazzy-desktop localhost/rviz-robotlab` |
| DDS discovery intermittent | ROS_DOMAIN_ID mismatch | Both sides should have `ROS_DOMAIN_ID=0` (default) |

## Appendix D: Known Limitations

1. **No pointcloud compression transport** — Raw PointCloud2 only. If bandwidth becomes an issue, a `compressed_pointcloud_transport` node would be needed (not built-in to ROS 2).
2. **Fixed TF frame name dependency** — RViz config hardcodes `head_d435_head_depth_optical_frame`. If Jetson camera namespacing changes, update the RViz config.
3. **Ethernet-only** — WiFi DDS discovery does NOT work with the Linksys AE3000 adapter. Always use Ethernet for cross-device ROS.
4. **Single machine RMW** — All containers on the same machine must use the same RMW. Mixing CycloneDDS and FastRTPS on the same host prevents discovery between those containers.
