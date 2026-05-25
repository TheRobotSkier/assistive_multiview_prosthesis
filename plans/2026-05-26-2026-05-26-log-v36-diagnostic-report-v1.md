# Log v36 Diagnostic Report

**Date**: 2026-05-26
**Log file**: `logs/host-log-v36.txt`
**Duration**: ~6 minutes (22:59:01 to 23:05:34)
**Container**: `prosthesis` (Podman, host network, privileged, USB passthrough)

---

## Hardware Setup

| Component | Device | Path |
|-----------|--------|------|
| Mia Hand | /dev/ttyUSB0 (FTBY495J) | /dev/ttyMiaHand |
| Wrist Dynamixel | /dev/ttyUSB1 (FTAO4Z0Y) | /dev/ttyDynamixel |
| Arm Camera | RealSense D435i (arm-mounted) | OpenVINS tracking |
| Head Camera | RealSense D435i (head-mounted) | OpenVINS NOT initialized |
| EMG Board | MindRove WiFi (192.168.4.1:4210) | 500 Hz, 8 channels |
| Segmentation | CPU container (segmentation-cpu) | Port 5678 |

**Key config**: `publish_initial_commands: false`, grasp type override to cylindrical with 10% closure, `grasp_contact_offset: [0.1543, -0.1485, 0.1352]`.

---

## What Was Run

Two grasp attempts were made:

### Attempt 1 (T+50s to T+88s)

1. EMG POWER held 1.1s -> IDLE -> TWISTING
2. Twist propagation activated, found hit at (0.322, 0.101, 0.047) after ~9s
3. Pipeline: TWISTING -> SEGMENTING -> PLANNING
4. Preshaping result: Cylindrical, closure=0.05, combined score=0.2708 (poor)
5. Override applied: cylindrical with 10% closure
6. Proximity controller committed plan: closures=[0.1, 0.1, 0.1], wrist=277.2 deg
7. Pipeline: PLANNING -> APPROACHING, twist propagation deactivated
8. User moved hand toward object over ~23s
9. Distance dropped from 0.385m to ~0.25m minimum
10. EMG OPEN triggered RELEASING -> IDLE at T+88s
11. **Hand never closed. Never entered near zone during APPROACHING.**

### Attempt 2 (T+107s to end)

1. EMG POWER -> TWISTING again
2. Twist propagation found hit at (0.415, 0.068, 0.039) after ~9s
3. Preshaping **failed**: "No points in ROI, cannot score grasps"
4. Pipeline: PLANNING -> IDLE (preshaping failure)
5. Twist propagation **not deactivated** — continued running in background
6. Multiple subsequent hits found by twist propagation, all triggered segmentation that either timed out or failed
7. Proximity controller **continued running with stale plan from attempt 1**
8. Distance dropped as low as 0.021m — near zone entered multiple times
9. **But pipeline was in IDLE/TWISTING, not APPROACHING, so near zone signal was ignored**
10. Eventually OpenVINS tracking diverged — distance exploded to 930m
11. Ctrl+C at T+390s

---

## Issues Observed

### Issue 1: Proximity Controller Does Not Clear Plan on State Exit

**Severity**: Critical — root cause of "hand never closed"

**What happened**: After attempt 1's RELEASING -> IDLE transition, the proximity controller retained the committed plan (closures, wrist, hand frame). It continued computing distances, publishing finger/wrist commands, and entering/exiting the near zone — all while the pipeline was in IDLE or TWISTING.

**Evidence**:
- `host-log-v36.txt:462`: Plan committed during attempt 1
- `host-log-v36.txt:507`: Pipeline goes to RELEASING (EMG OPEN)
- `host-log-v36.txt:516`: Pipeline goes to IDLE
- `host-log-v36.txt:512-532`: Proximity controller keeps publishing FAR mode commands
- `host-log-v36.txt:705`: "Entered near zone" — but pipeline is in IDLE

**Why it matters**: The near zone signal (`Bool(True)` on `/proximity/near_zone_entered`) is correctly published, but the pipeline manager's handler at `pipeline_manager_node.py:550-553` checks `self._state == State.APPROACHING` and ignores it when in IDLE/TWISTING. This is correct behavior by the pipeline manager — the proximity controller should not be running at all outside APPROACHING.

**Relevant code**:
- `grasp_proximity_controller_node.py:192-196`: Control loop gates on `pipeline_state in (5, 6, 7)` (GRASPING/HOLDING/VOLITIONAL) but does NOT gate on APPROACHING. It runs in ALL states except those three, including IDLE.
- `grasp_proximity_controller_node.py:162-163`: `_on_pipeline_state` only stores the state value — never clears the plan.

### Issue 2: Wrist Rotation of 277.2 Degrees

**Severity**: High — wrist goes to extreme/unusual position

**What happened**: The preshaping planner computed a wrist rotation of 277.2 degrees. The proximity controller sent this to the Dynamixel servo. The user observed the wrist going "very high up" — likely the servo rotating nearly 280 degrees from its initial position.

**Evidence**:
- `host-log-v36.txt:462`: "wrist=277.2 deg"
- User feedback: "we tried to find the position the system was going for and ended up with the hand very high up"

**Why it matters**: The Dynamixel servo may have physical limits. A 277-degree rotation is extreme and likely not the intended approach angle. The planner's wrist computation is unreliable with sparse single-camera data.

**Relevant code**:
- `preshaping_service_bridge_node.cpp`: Wrist rotation comes from `ffi_response.wrist_rotation_deg` — the SMC planner's output. No clamping is applied.
- `grasp_proximity_controller_node.py:280-283`: `_publish_wrist_command` sends the raw planned value.

### Issue 3: No Distance Sanity Check — Commands Sent at 930m

**Severity**: High — garbage commands when tracking lost

**What happened**: At approximately T+295s, OpenVINS arm tracking diverged. The reported hand position started drifting rapidly. The distance went from ~0.1m to 930m over ~40 seconds. During this entire time, the proximity controller kept publishing wrist and finger commands.

**Evidence**:
- `host-log-v36.txt:1060`: dist=0.950m (first sign of divergence)
- `host-log-v36.txt:1065`: dist=5.791m
- `host-log-v36.txt:1069`: dist=9.263m
- `host-log-v36.txt:1111`: dist=930.350m (last before Ctrl+C)
- The odom relay continued reporting arm: OK throughout (tracking was "active" but diverged)

**Why it matters**: The proximity controller has no maximum distance guard. When tracking diverges, it sends increasingly nonsensical wrist/finger commands. This could cause physical harm if the servo tries to reach an extreme position.

**Relevant code**:
- `grasp_proximity_controller_node.py:233-251`: Control loop computes distance and publishes commands with no upper bound check.

### Issue 4: Twist Propagation Not Deactivated on Preshaping Failure

**Severity**: Medium — wasted compute, spurious segmentations

**What happened**: After attempt 2's preshaping failed ("No points in ROI"), the pipeline went PLANNING -> IDLE. But twist propagation was never deactivated. It continued finding hits and triggering segmentation runs that all failed or timed out.

**Evidence**:
- `host-log-v36.txt:573`: "Preshaping failed: No points in ROI"
- `host-log-v36.txt:574`: PLANNING -> IDLE
- `host-log-v36.txt:591`: Twist finds another hit (still active)
- `host-log-v36.txt:639,682,795,922,947,1040`: More hits found, all triggering failed segmentations
- `host-log-v36.txt:650,701,811,935,961,1056`: Segmentation timeouts (5.0s each)

**Why it matters**: Each segmentation timeout wastes 5 seconds of GPU/CPU. Over the ~5 minutes of the run, twist propagation triggered ~8 failed segmentation cycles. The segmentation server also crashed twice (lines 689-690, 956-957: "Connection reset by peer").

**Relevant code**:
- `pipeline_manager_node.py`: `_on_preshaping_response` deactivates twist only on success (APPROACHING transition). The failure path (PLANNING -> IDLE) does not call `_deactivate_twist_propagation()`.

### Issue 5: Rapid FAR/NEAR Switching at Boundary

**Severity**: Low — inconsistent behavior, no physical harm

**What happened**: When the distance was near the 0.08m/0.10m threshold, the proximity controller rapidly alternated between FAR and NEAR mode. The 2cm hysteresis gap (0.08 enter, 0.10 exit) is insufficient for noisy tracking data.

**Evidence**:
- `host-log-v36.txt:1013-1033`: Entered/exited near zone 4 times in ~10 seconds
- `host-log-v36.txt:1281-1033`: Another rapid cycle

**Why it matters**: Each transition publishes a `Bool` on `/proximity/near_zone_entered`. If the pipeline were in APPROACHING, rapid True/False toggling could cause issues with the GRASPING transition. The pipeline manager only acts on `True`, so the main risk is multiple GRASPING transition attempts.

**Relevant config**: `proximity_enter_threshold_m: 0.08`, `proximity_exit_threshold_m: 0.10` (only 2cm gap).

### Issue 6: Mia Hand Communication Failures Throughout

**Severity**: Medium — finger commands silently dropped

**What happened**: Throughout the entire run, the command bridge reported trajectory rejections from the Mia Hand:
- "Invalid ACK received back from Mia Hand" — most common
- "Timeout occurred before receiving command acknowledge from Mia Hand"

These appear roughly every 5-15 seconds during active finger commands.

**Evidence**: Lines 479-480, 482, 493-494, 520-521, 534, 581-582, 633-634, 665-666, 748-749, 765-766, 770, 772-774, 787, 808, 829-831, 859-863, 875-877, 906, 912, 965-966, 977, 984-985, 993, 1018-1020, 1089-1090, 1104, 1110.

**Why it matters**: Even if the proximity controller correctly sends finger closure commands, the Mia Hand may not execute them due to communication failures. The hand was also observed disconnecting/reconnecting (line 818-822).

### Issue 7: Head Camera OpenVINS Never Initialized

**Severity**: Low (known, not the focus of this session)

**What happened**: Throughout the entire 6-minute run, the head OpenVINS chain produced zero messages. Only the arm camera was active. This degraded point cloud quality and planner scores.

**Evidence**: Every 10-second relay status shows "head: WAITING (0 msgs)".

---

## State of Previous Fixes (v30-v35)

| Fix | Status | Evidence |
|-----|--------|----------|
| Pose vs PoseStamped type mismatch (v30) | Working | No type mismatch warnings at startup |
| Wrist publish outside `publish_initial_commands` guard (v31) | Working | Plan commits with wrist value (line 462) |
| STOP_ALL wrist commands removed (v32) | Working | No wrist oscillation observed |
| Near zone publisher added (v32) | Working | "Entered near zone" logged, Bool published |
| QoS TRANSIENT_LOCAL on near zone (v34) | Working | No QoS incompatibility warning |
| Grasp contact offset (v35) | Working | Distances are physically meaningful (down to 0.021m) |
| Cylindrical grasp override with 10% (v32) | Working | closures=[0.1, 0.1, 0.1] committed |
| Twist deactivation on APPROACHING entry (v31) | Working | Twist deactivated at line 465 |
| External twist max_age increased to 2.0s (v30) | Working | No stale twist spam |
| Wrist driver retry logic (v30) | Working | Retries observed (line 769: "after 3 attempts") |

---

## Key Files for Reference

| File | Role |
|------|------|
| `src/grasp_preshaping/nodes/grasp_proximity_controller_node.py` | Proximity-based approach controller |
| `src/pipeline_manager/pipeline_manager/pipeline_manager_node.py` | State machine orchestrator |
| `src/grasp_preshaping/nodes/preshaping_service_bridge_node.cpp` | Preshaping planner bridge (C++) |
| `src/twist_propagation/twist_propagation/twist_propagation_node.py` | Twist propagation and hit detection |
| `src/wrist_driver/wrist_driver/wrist_driver_node.py` | Dynamixel wrist servo driver |
| `src/force_controller/force_controller/force_controller_node.py` | Force-based grasp controller |
| `config/prosthesis_config.yaml` | All node parameters |
