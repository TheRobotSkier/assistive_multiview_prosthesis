# Cloud Snapshot Node — Unit Test Plan

## Overview

The **Cloud Snapshot Node** (`camera` package, `cloud_snapshot_node` executable) subscribes to the segmented object cloud (`/segmentation/object_cloud`) and republishes it as a snapshot topic (`/segmentation/object_cloud_snapshot`). Its behavior depends on the pipeline state:

| Pipeline State | Cloud Snapshot Behavior |
|----------------|------------------------|
| IDLE | **Pass-through** — live cloud appears on both topics unchanged |
| SEGMENTING | **Pass-through** — live cloud forwarded unchanged |
| PLANNING | **Freeze** — current cloud is frozen; same message published repeatedly |
| APPROACHING | **Freeze persists** — frozen cloud stays while approaching target |
| GRASPING | **Freeze persists** — frozen cloud stays while grasping |
| HOLDING | **Freeze persists** — frozen cloud stays while holding |
| RELEASING | **Clear** — frozen cloud is cleared; empty snapshot published |
| IDLE (after release) | **Pass-through resumes** — live cloud forwarded again |

The node subscribes to the pipeline state topic (`/pipeline/state_name`) to determine the current state.

## Test Environment

### Pre-requisites

- ROS 2 Jazzy workspace built (`colcon build --packages-select camera`)
- Pipeline manager node available (or manually publish `/pipeline/state_name`)
- Bag file or live source for `/segmentation/object_cloud`

### Tools

```bash
# Echo snapshot topic
ros2 topic echo /segmentation/object_cloud_snapshot

# Echo live cloud topic
ros2 topic echo /segmentation/object_cloud

# Publish pipeline state manually
ros2 topic pub /pipeline/state_name std_msgs/String "{data: 'PLANNING'}"

# Record and replay bags
ros2 bag record -o test_bag /segmentation/object_cloud /pipeline/state_name
ros2 bag play test_bag
```

## Test Cases

### Test 1: Pass-through Mode (IDLE State)

**Description:** When the pipeline is in IDLE state, the snapshot topic should mirror the live cloud topic with no modifications.

**Setup:**
- Start `ros2 run camera cloud_snapshot_node`
- Pipeline manager publishes `/pipeline/state_name = "IDLE"`
- A cloud publisher publishes a known point cloud on `/segmentation/object_cloud`

**Input:**
- `/pipeline/state_name`: `"IDLE"`
- `/segmentation/object_cloud`: `Cloud_A` (sphere of 1000 points at origin)

**Expected Output:**
- `/segmentation/object_cloud_snapshot` publishes `Cloud_A` with identical points, identical frame_id, and same fields.
- Each new `Cloud_A` update (e.g., `Cloud_A'`) is immediately forwarded (no freeze).

**How to Run:**
```bash
# Terminal 1: start snapshot node
ros2 run camera cloud_snapshot_node

# Terminal 2: echo snapshot topic
ros2 topic echo /segmentation/object_cloud_snapshot

# Terminal 3: publish state
ros2 topic pub /pipeline/state_name std_msgs/String "{data: 'IDLE'}" -r 1

# Terminal 4: publish cloud (quality of service must match)
ros2 topic pub /segmentation/object_cloud sensor_msgs/PointCloud2 "{...}" -r 1
```

**Verification:** Compare cloud data between live and snapshot topics — they should be identical.

---

### Test 2: Freeze on PLANNING

**Description:** When the pipeline transitions from any non-PLANNING state to PLANNING, the snapshot freezes at the last cloud received.

**Setup:**
- Start cloud snapshot node
- Start in IDLE state with cloud `Cloud_A` being published
- Transition to PLANNING

**Input:**
- State changes: `IDLE` → `PLANNING`
- Cloud `Cloud_A` continues publishing after transition

**Expected Output:**
- At the moment of PLANNING transition, the snapshot publishes the last received `Cloud_A`.
- Subsequent updates to the live cloud are **not** reflected in the snapshot.
- Snapshot header stamps may update (depending on implementation) but point data remains identical to the frozen `Cloud_A`.

**How to Run:**
```bash
# Terminal 1: start snapshot node
ros2 run camera cloud_snapshot_node

# Terminal 2: monitor snapshot
ros2 topic echo /segmentation/object_cloud_snapshot --no-arr

# Terminal 3: send state transition
ros2 topic pub --once /pipeline/state_name std_msgs/String "{data: 'PLANNING'}"

# Terminal 4: publish changing cloud
ros2 topic pub /segmentation/object_cloud sensor_msgs/PointCloud2 "{...}" -r 2
```

**Verification:** Save the cloud data at PLANNING transition and 5 seconds later — points should be identical.

---

### Test 3: Freeze Persists (APPROACHING / GRASPING / HOLDING)

**Description:** Once frozen at PLANNING, the snapshot remains frozen through APPROACHING, GRASPING, and HOLDING states.

**Setup:**
- Start cloud snapshot node
- Transition through states: `IDLE` → `SEGMENTING` → `PLANNING` → `APPROACHING` → `GRASPING` → `HOLDING`

**Input:**
- State transitions as above
- Cloud `Cloud_A` published continuously

**Expected Output:**
- Snapshot freezes at `Cloud_A` when PLANNING is received.
- `Cloud_A` continues to appear on snapshot during APPROACHING, GRASPING, and HOLDING.
- No cloud update propagates through the snapshot during these states.

**How to Run:**
```bash
# Send a sequence of states with 2-second intervals
ros2 topic pub --once /pipeline/state_name std_msgs/String "{data: 'SEGMENTING'}"
sleep 2
ros2 topic pub --once /pipeline/state_name std_msgs/String "{data: 'PLANNING'}"
sleep 5
ros2 topic pub --once /pipeline/state_name std_msgs/String "{data: 'APPROACHING'}"
sleep 5
ros2 topic pub --once /pipeline/state_name std_msgs/String "{data: 'GRASPING'}"
sleep 5
ros2 topic pub --once /pipeline/state_name std_msgs/String "{data: 'HOLDING'}"
```

**Verification:** The snapshot topic data hash should not change after PLANNING is received.

---

### Test 4: Clear on RELEASING

**Description:** When the pipeline transitions to RELEASING, the frozen cloud should be cleared and an empty snapshot published.

**Setup:**
- Start cloud snapshot node
- Pipeline in HOLDING state with frozen cloud `Cloud_A`
- Transition to RELEASING

**Input:**
- State changes: `HOLDING` → `RELEASING`
- Cloud `Cloud_A` was frozen

**Expected Output:**
- On RELEASING, snapshot publishes an empty point cloud (width=0, no points).
- The snapshot topic may publish a single empty message or a cloud with zero points depending on implementation.

**How to Run:**
```bash
# After HOLDING, send RELEASING
ros2 topic pub --once /pipeline/state_name std_msgs/String "{data: 'RELEASING'}"
ros2 topic echo /segmentation/object_cloud_snapshot --once
```

**Verification:** The snapshot message has `width=0` or equivalent empty cloud representation.

---

### Test 5: Empty Cloud Input

**Description:** The snapshot node should handle an empty incoming cloud gracefully without crashing or entering an invalid state.

**Setup:**
- Start cloud snapshot node in IDLE state
- Publish an empty cloud on `/segmentation/object_cloud`

**Input:**
- `/pipeline/state_name`: `"IDLE"`
- `/segmentation/object_cloud`: empty cloud (width=0)

**Expected Output:**
- Snapshot publishes the empty cloud (pass-through).
- No crashes, no segmentation faults, no ROS warnings about invalid data.
- Node remains responsive to state transitions.

**How to Run:**
```bash
# Use the demo_cloud_publisher with an empty PLY, or publish a manually crafted empty cloud
ros2 topic pub --once /segmentation/object_cloud sensor_msgs/PointCloud2 "{height: 1, width: 0, fields: [], is_bigendian: false, point_step: 0, row_step: 0, data: [], is_dense: true}"
ros2 topic echo /segmentation/object_cloud_snapshot --once
```

**Verification:** Node continues running without errors. Empty cloud is forwarded.

---

### Test 6: Full State Cycle — Verify Snapshot at Each Transition

**Description:** Run through a complete IDLE → SEGMENTING → PLANNING → APPROACHING → GRASPING → HOLDING → RELEASING → IDLE cycle and verify snapshot state at every step.

**Setup:**
- Start cloud snapshot node
- Use a bag file or scripted publisher to send both state changes and cloud updates
- Record all snapshot output for offline analysis

**Input:**
- State sequence (with timing):
  1. `IDLE` (0s) — live cloud published every 1s
  2. `SEGMENTING` (3s) — live cloud continues
  3. `PLANNING` (6s) — expect freeze
  4. `APPROACHING` (10s) — expect freeze persists
  5. `GRASPING` (15s) — expect freeze persists
  6. `HOLDING` (20s) — expect freeze persists
  7. `RELEASING` (25s) — expect clear
  8. `IDLE` (28s) — expect pass-through resumes

- Cloud: 5 updates during the cycle, arbitrarily changing

**Expected Output:**

| Time (s) | State | Snapshot Behavior | Snapshot Content |
|----------|-------|-------------------|-----------------|
| 0–3 | IDLE | Pass-through | Latest live cloud |
| 3–6 | SEGMENTING | Pass-through | Latest live cloud |
| 6–10 | PLANNING | Freeze | Cloud as of t=6 |
| 10–15 | APPROACHING | Freeze persists | Same as t=6 |
| 15–20 | GRASPING | Freeze persists | Same as t=6 |
| 20–25 | HOLDING | Freeze persists | Same as t=6 |
| 25–28 | RELEASING | Clear | Empty cloud |
| 28+ | IDLE | Pass-through resumes | Latest live cloud |

**How to Run:**
```bash
# Record a bag or use a script that publishes the sequence
ros2 bag record -o snapshot_test /segmentation/object_cloud_snapshot /segmentation/object_cloud /pipeline/state_name
# Then run the node and replay the bag
ros2 run camera cloud_snapshot_node &
ros2 bag play snapshot_test
```

**Verification:** Post-process the bag — check that snapshot matches expected content at each state transition.

## Implementation Notes

- The node should use **QoS matching** (RELIABLE + TRANSIENT_LOCAL) to avoid losing state transitions.
- Cloud comparison for "frozen" checks should compare either:
  - Raw data bytes (md5sum of `data` field)
  - Point count + first/last point coordinates
- Empty cloud can be detected by checking `width == 0` or `row_step == 0`.

## Appendix: Helper Script for Test Automation

```python
#!/usr/bin/env python3
"""Helper to run state sequence for snapshot tests."""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class StateSequencePublisher(Node):
    def __init__(self):
        super().__init__('state_sequence_publisher')
        self.pub = self.create_publisher(String, '/pipeline/state_name', 10)
        
    def run_sequence(self):
        states = ['IDLE', 'SEGMENTING', 'PLANNING', 'APPROACHING',
                  'GRASPING', 'HOLDING', 'RELEASING', 'IDLE']
        intervals = [3, 3, 4, 5, 5, 5, 3, 3]  # seconds per state
        for state, interval in zip(states, intervals):
            msg = String(data=state)
            self.pub.publish(msg)
            self.get_logger().info(f'State: {state} (for {interval}s)')
            time.sleep(interval)
        self.get_logger().info('Sequence complete')

def main():
    rclpy.init()
    node = StateSequencePublisher()
    node.run_sequence()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
