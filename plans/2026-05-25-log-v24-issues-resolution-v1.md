# Log v24 Issues — Diagnosis & Resolution Plan

## Objective

Fix the two crashes found in `host-log-v24.txt`.

## Issues Found

### Issue 1 — Twist Propagation Crash: Missing `_hit_detected` and `_hit_detected_pub`
**Source:** `host-log-v24.txt:169-793`

After activation and detecting the first hit, the node spams:
```
AttributeError: 'TwistPropagationNode' object has no attribute '_hit_detected'
```

**Root Cause:** Same pattern as the `_collision_distance_pub` fix from v23. Two things are missing:
- `_hit_detected` — a boolean instance variable, used at lines 798, 1715, 1719 but never initialized in `__init__`
- `_hit_detected_pub` — a publisher for `/twist_propagation/hit_detected` (Bool), used at lines 797, 1718 but never created with `create_publisher()`

The pipeline manager subscribes to `/twist_propagation/hit_detected` at `pipeline_manager_node.py:127,236`.

### Issue 2 — Wrist Driver Crash: Wrong throttled logging API
**Source:** `host-log-v24.txt:102-128`

```
AttributeError: 'RcutilsLogger' object has no attribute 'throttle'
```

**Root Cause:** In the v23 fix, I introduced `self.get_logger().throttle(5.0, msg)` at `wrist_driver_node.py:145`. This is not a valid ROS 2 Python logging API. The correct API is `self.get_logger().warn(msg, throttle_duration_sec=5.0)`. The codebase uses this pattern consistently elsewhere (e.g., `pipeline_manager_node.py`, `pointcloud_fusion_node.py`).

## Implementation Plan

- [ ] **Fix 1:** Add `_hit_detected = False` initialization and `_hit_detected_pub` publisher in `TwistPropagationNode.__init__`
  - File: `src/twist_propagation/twist_propagation/twist_propagation_node.py`
  - Add `self._hit_detected = False` in the state initialization section (near other instance variables)
  - Add `self._hit_detected_pub = self.create_publisher(Bool, "/twist_propagation/hit_detected", 10)` in the publishers section (near `_hit_time_pub` at line 713)
  - Ensure `Bool` is imported from `std_msgs.msg` (already imported at line 75)

- [ ] **Fix 2:** Replace `self.get_logger().throttle(5.0, msg)` with correct ROS 2 API
  - File: `src/wrist_driver/wrist_driver/wrist_driver_node.py:145-149`
  - Change to `self.get_logger().warn(msg, throttle_duration_sec=5.0)`

## Verification Criteria

1. `twist_propagation_node` does not crash with `AttributeError` when a hit is detected
2. `wrist_driver_node` does not crash on communication failures — logs throttled warnings instead
3. Pipeline manager receives hit detection events on `/twist_propagation/hit_detected`
