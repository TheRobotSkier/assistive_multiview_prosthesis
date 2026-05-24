# Hardware Integration Test Plan: Proximity Controller, Twist Propagation, Force Controller

## Objective

Create a simple, practical procedure to verify that the proximity controller, twist propagation node, and force controller all work correctly with the actual Mia Hand hardware — without needing the full pipeline (camera, segmentation, EMG band). The test uses manual ROS 2 CLI commands to inject fake data and observe the hand's response.

---

## Background: How the Three Components Work

### 1. Twist Propagation (`twist_propagation_node.py`)

**Purpose:** Predicts where the hand is heading and triggers segmentation when it's about to hit an object.

**How it works:**
- Subscribes to `/hand_pose` (PoseStamped) — the current hand position
- Subscribes to `/camera/depth/color/points` (PointCloud2) — the scene
- Estimates the hand's velocity (twist) from sequential pose messages using finite differences + least-squares fitting over a sliding window
- Propagates the hand pose forward in time (up to 2 seconds) and checks if the predicted path intersects any point cloud points (within 5 cm threshold)
- If a hit is detected → publishes a "click" on `/segmentation/click_positive` to trigger segmentation
- When segmentation completes → calls `/grasp_preshaping/compute_grasp` service
- Has an internal state machine: IDLE → WAITING_FOR_SEGMENTATION → WAITING_FOR_PRESHAPING → IDLE
- Starts **inactive** by default (`active: false` in config); must be activated via `/twist_propagation/activate` service

**Key params** (`config/prosthesis_config.yaml:112-128`): cycle_delay=0.5s, horizon=2.0s, hit_threshold=0.05m, cloud_max_age=2.0s

### 2. Grasp Proximity Controller (`grasp_proximity_controller_node.py`)

**Purpose:** Gradually closes the hand and adjusts the wrist as the hand approaches the target — like a "pre-grasp" controller.

**How it works:**
- Subscribes to three plan topics from the preshaping pipeline:
  - `/grasp_preshaping/target_finger_closures` — planned [thumb, index, mrl] closure values (0.0–1.0)
  - `/grasp_preshaping/wrist_pose` — planned wrist rotation in degrees
  - `/grasp_preshaping/target_hand_pose` — planned final hand position (PoseStamped)
- Subscribes to `/hand_pose` for the current hand position
- Subscribes to `/pipeline/state` to know when to stop (doesn't publish during GRASPING/HOLDING)
- Uses **hysteresis** on distance to the target:
  - **FAR** (dist > 10 cm): sends partial closure (30% of planned) + wrist rotation
  - **NEAR** (dist < 8 cm): sends full planned closure
- When a new plan arrives, immediately sends the partial (far) closure and wrist command
- Publishes to the three `*_pos_ff_controller/commands` topics and `/wrist/set_position`

**Key params** (`config/prosthesis_config.yaml:79-86`): enter_threshold=0.08m, exit_threshold=0.10m, partial_closure_factor=0.3

### 3. Force Controller (`force_controller_node.py`)

**Purpose:** Maintains stable grasp force once the hand has closed on the object.

**How it works:**
- Subscribes to `data_streams/fingers/forces/data` (ForceData) — 6 raw force values from Mia Hand sensors
- Subscribes to `/pipeline/state` — activates only during GRASPING and HOLDING
- Subscribes to `/joint_states` — tracks current finger positions
- On entering GRASPING: activates force streaming via `data_streams/fingers/forces/switch` service
- Runs a **PI controller** at 10 Hz:
  - Target force = midpoint of [50, 200] raw units
  - Computes error → adjusts finger positions incrementally (max 0.05 rad/tick)
  - Moving-average filter (window=5) on force readings
  - Anti-windup on integral term
- **Slip detection:** monitors tangential force rate-of-change
- **Emergency release:** backs off if any force exceeds 500 raw units
- Publishes `/force_controller/status` (ForceControllerStatus) — used by pipeline manager for GRASPING→HOLDING transition

**Key params** (`config/prosthesis_config.yaml:88-102`): target=[50,200], kp=0.01, ki=0.001, max_step=0.05rad

### How They Chain Together

```
Hand moves toward object
        │
        ▼
Twist Propagation detects path will hit object
        │ → triggers segmentation → triggers preshaping
        ▼
Preshaping produces: closures + wrist angle + target hand pose
        │
        ▼
Proximity Controller:
  FAR → partial close (30%) + wrist rotation
  NEAR → full close
        │
        ▼ (pipeline transitions APPROACHING → GRASPING)
Force Controller:
  Activates force streaming
  PI loop adjusts finger positions to maintain target force
  Reports stability → pipeline transitions to HOLDING
```

The **pipeline manager** orchestrates the state transitions: IDLE → SEGMENTING → PLANNING → APPROACHING → GRASPING → HOLDING → RELEASING.

---

## Hardware Test Plan

### Prerequisites

- Mia Hand connected via USB (check `ls /dev/ttyUSB*`)
- Wrist Dynamixel connected via USB
- Docker container running with `--profile hardware` (or native ROS 2 install sourced)
- All packages built: `colcon build` in the workspace
- No other nodes running (clean ROS session)

### Test 1: Proximity Controller — Verify Hand Closes Gradually

**Goal:** Confirm the proximity controller correctly sends partial closure when "far" and full closure when "near" the target.

**Setup:**
```bash
# Terminal 1: Launch just the proximity controller
ros2 run grasp_preshaping grasp_proximity_controller_node.py
```

**Test steps:**
```bash
# Terminal 2: Inject a fake plan — target at (0.5, 0, 0.5), closures [0.8, 0.7, 0.6], wrist 30°
ros2 topic pub --once /grasp_preshaping/target_finger_closures std_msgs/msg/Float64MultiArray "{data: [0.8, 0.7, 0.6]}"
ros2 topic pub --once /grasp_preshaping/wrist_pose std_msgs/msg/Float64 "{data: 30.0}"
ros2 topic pub --once /grasp_preshaping/target_hand_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: 'world'}, pose: {position: {x: 0.5, y: 0.0, z: 0.5}, orientation: {w: 1.0}}}"

# Terminal 3: Report current hand position FAR from target (should get 30% closure)
ros2 topic pub --rate 10 /hand_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: 'world'}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"

# Watch the commands — should see partial closures (~0.24, 0.21, 0.18)
ros2 topic echo /thumb_pos_ff_controller/commands

# Now move hand "near" the target — should see full closures (0.8, 0.7, 0.6)
ros2 topic pub --rate 10 /hand_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: 'world'}, pose: {position: {x: 0.49, y: 0.01, z: 0.51}, orientation: {w: 1.0}}}"

# Watch commands again — should jump to full closure
ros2 topic echo /thumb_pos_ff_controller/commands
```

**What to observe:**
- When hand is far: thumb≈0.24, index≈0.21, mrl≈0.18 (30% of plan)
- When hand is near: thumb=0.8, index=0.7, mrl=0.6 (full plan)
- Wrist should have received a 30° command
- The Mia Hand should physically move its fingers

### Test 2: Force Controller — Verify Force Regulation

**Goal:** Confirm the force controller activates during GRASPING, reads force data, and adjusts finger positions.

**Setup:**
```bash
# Terminal 1: Launch the Mia Hand driver (ros2_control)
ros2 launch mia_hand_ros2_control mia_hand_control.launch.py

# Terminal 2: Launch the force controller
ros2 run force_controller force_controller_node

# Terminal 3: Monitor force controller status
ros2 topic echo /force_controller/status
```

**Test steps:**
```bash
# Terminal 4: Force the pipeline into GRASPING state
ros2 topic pub --once /pipeline/state std_msgs/msg/Int32 "{data: 4}" --qos-durability transient_local --qos-depth 1

# Watch for:
# - "Activating force controller..." log message
# - Force streaming activation service call
# - Status messages showing active=true

# Now place an object in the hand and manually close it slightly
# (or use the proximity controller to close it first)
# Then watch force readings and position adjustments

# Monitor force data from the hand
ros2 topic echo data_streams/fingers/forces/data

# Monitor position commands from force controller
ros2 topic echo /thumb_pos_ff_controller/commands
```

**What to observe:**
- Force controller logs "Activating force controller..." when GRASPING state is set
- If force streaming activates, you should see `ForceData` messages with non-zero values when fingers touch an object
- The controller should publish small position adjustments on the command topics
- Status should show `active: true`, force values, and eventually `force_stable: true`
- If you squeeze the object harder, forces should rise and the controller should back off

### Test 3: Twist Propagation — Verify Hit Detection

**Goal:** Confirm twist propagation estimates hand velocity, propagates the pose, and detects a cloud intersection.

**Setup:**
```bash
# Terminal 1: Launch twist propagation
ros2 run twist_propagation twist_propagation_node

# Terminal 2: Monitor outputs
ros2 topic echo /hand_twist
ros2 topic echo /segmentation/click_positive
ros2 topic echo /twist_propagation/status
```

**Test steps:**
```bash
# Terminal 3: Activate the node
ros2 service call /twist_propagation/activate std_srvs/srv/Trigger

# Publish a fake point cloud with a cluster at (0.5, 0, 0.5)
# (Use a small Python script or the integration test helper)
python3 -c "
import rclpy, struct, time, numpy as np
from sensor_msgs.msg import PointCloud2, PointField
rclpy.init()
node = rclpy.create_node('cloud_pub')
pub = node.create_publisher(PointCloud2, '/camera/depth/color/points', 10)
rng = np.random.default_rng(42)
pts = rng.normal(loc=[0.5, 0.0, 0.5], scale=0.02, size=(200, 3))
msg = PointCloud2()
msg.header.frame_id = 'world'
msg.header.stamp = node.get_clock().now().to_msg()
msg.height = 1; msg.width = len(pts)
msg.fields = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
]
msg.is_bigendian = False; msg.point_step = 12
msg.row_step = 12 * len(pts)
buf = bytearray()
for p in pts: buf.extend(struct.pack('fff', float(p[0]), float(p[1]), float(p[2])))
msg.data = bytes(buf); msg.is_dense = True
pub.publish(msg)
time.sleep(0.5)
rclpy.shutdown()
"

# Publish moving hand poses toward the cluster
# (hand starts at origin, moves in +x direction)
for i in $(seq 1 10); do
  x=$(echo "$i * 0.03" | bc)
  ros2 topic pub --once /hand_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: 'world', stamp: {sec: 0, nanosec: $((i * 50000000))}}, pose: {position: {x: $x, y: 0.0, z: 0.5}, orientation: {w: 1.0}}}"
  sleep 0.05
done

# Watch /segmentation/click_positive — should get a hit point near (0.5, 0, 0.5)
# Watch /twist_propagation/status — should show state transitions
```

**What to observe:**
- `/hand_twist` shows non-zero linear velocity in +x direction
- `/twist_propagation/status` JSON shows active=true and eventually a hit_point
- `/segmentation/click_positive` gets a PointStamped near the cloud cluster

---

## Simplest Possible Combined Test

If you want to test all three together with the real hand in one go:

```bash
# Terminal 1: Launch the full pipeline (hardware mode)
ros2 launch prosthesis_launch pipeline.launch.py

# Terminal 2: Simulate the full sequence manually

# Step A: Trigger a "grasp" by publishing a grasp gesture
ros2 topic pub --once /emg/gesture_label std_msgs/msg/Int32 "{data: 1}"  # POWER

# Step B: Inject a fake preshaping result
# (this simulates what segmentation+preshaping would produce)
ros2 topic pub --once /grasp_preshaping/grasp_type std_msgs/msg/Int32 "{data: 1}"
ros2 topic pub --once /grasp_preshaping/target_finger_closures std_msgs/msg/Float64MultiArray "{data: [0.8, 0.7, 0.6]}"
ros2 topic pub --once /grasp_preshaping/wrist_pose std_msgs/msg/Float64 "{data: 30.0}"
ros2 topic pub --once /grasp_preshaping/target_hand_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: 'world'}, pose: {position: {x: 0.5, y: 0.0, z: 0.5}, orientation: {w: 1.0}}}"

# Step C: Move the hand "near" the target to trigger full closure
ros2 topic pub --rate 10 /hand_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: 'world'}, pose: {position: {x: 0.49, y: 0.01, z: 0.51}, orientation: {w: 1.0}}}"

# Step D: Force pipeline into GRASPING to activate force controller
ros2 topic pub --once /pipeline/state std_msgs/msg/Int32 "{data: 4}" --qos-durability transient_local

# Step E: Place an object in the hand and watch force regulation
ros2 topic echo /force_controller/status

# Step F: Release
ros2 topic pub --once /emg/gesture_label std_msgs/msg/Int32 "{data: 3}"  # OPEN
```

**Watch for:**
1. Hand partially closes (proximity controller, far mode)
2. As hand moves near target, fingers close fully
3. Force controller activates, force data starts flowing
4. Position adjustments maintain stable force
5. OPEN gesture triggers release

---

## Quick Diagnostic Commands

```bash
# Check all running nodes
ros2 node list

# Check topic connections
ros2 topic info /thumb_pos_ff_controller/commands
ros2 topic info /force_controller/status

# Monitor pipeline state
ros2 topic echo /pipeline/state

# Check force data is flowing
ros2 topic hz data_streams/fingers/forces/data

# Check joint positions
ros2 topic echo /joint_states

# Verify twist propagation status
ros2 topic echo /twist_propagation/status
```

---

## Key Risks and Mitigations

1. **No `/hand_pose` publisher in hardware mode:** The system expects an external hand tracking source to publish `/hand_pose`. If there's no tracker, you must publish it manually (as shown above) or the proximity controller and twist propagation won't work.
   - Mitigation: Use the manual `ros2 topic pub` commands shown in the tests.

2. **Force streaming requires Mia Hand driver to be running:** The force controller calls `data_streams/fingers/forces/switch` which is a service provided by the Mia Hand driver. If the driver isn't running, the service call will fail and the controller will log a warning but continue in passive mode.
   - Mitigation: Always launch the Mia Hand driver first and verify with `ros2 service list | grep forces`.

3. **Pipeline state must be set with TRANSIENT_LOCAL durability:** The pipeline manager publishes `/pipeline/state` with latched QoS. If you publish it manually, you must use `--qos-durability transient_local` or the force controller may miss the message.
   - Mitigation: Use the `--qos-durability transient_local` flag shown in the commands.

4. **Finger position commands are in radians:** The `*_pos_ff_controller/commands` topics accept radians. The closure values (0.0–1.0) from the proximity controller are NOT radians — they're normalized closure amounts. The ros2_control system interface maps these. Be aware of this when interpreting topic values.
   - Mitigation: Just watch the hand physically move. The exact numeric values are less important than seeing the correct behavior.

5. **Emergency stop:** If the hand starts closing too hard, you can immediately release by publishing:
   ```bash
   ros2 topic pub --once /pipeline/state std_msgs/msg/Int32 "{data: 6}" --qos-durability transient_local
   ```
   Or cut power to the hand.
