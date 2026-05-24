# Hardware Testing Analysis: Force Controller & Pipeline Manager with Wrist + Hand Only

## Objective

Analyze the system to determine what happens when you run `make up-hw` with only the Mia Hand (wrist + hand) connected, identify what will work, what won't, and what you need to do to test the force controller and pipeline manager on real hardware.

---

## 1. What `make up-hw` Actually Does

### Docker Compose Behavior

`make up-hw` runs (`docker/docker-compose.yml:36`):
```
docker compose --profile hardware up -d --build --force-recreate
```

This activates the `prosthesis-hw` service (`docker-compose.yml:40-47`), which:
- Uses the **same image** as `prosthesis` (no separate build)
- Maps USB devices: `/dev/ttyUSB1` → Mia Hand, `/dev/ttyUSB0` → Wrist Dynamixel
- **Does NOT auto-launch any ROS nodes** — the container CMD is `/bin/bash` (`Dockerfile:71`)
- You must `make shell` and then manually run a launch file

### Critical: No Wrist Driver in pipeline.launch.py

The `pipeline.launch.py` (`src/prosthesis_launch/launch/pipeline.launch.py:150-160`) launches these nodes:
1. `pipeline_manager`
2. `mia_hand_driver`
3. `command_bridge`
4. `emg_bridge`
5. `segmentation_bridge`
6. `twist_propagation`
7. `preshaping_service`
8. `proximity_controller`
9. `force_controller`
10. `rviz`

**The wrist_driver is NOT included in pipeline.launch.py.** It has its own separate launch file (`src/wrist_driver/launch/wrist_driver.launch.py`) and must be launched separately.

---

## 2. Dependency Analysis: What Each Target Node Requires

### Force Controller (`force_controller_node.py`)

**Subscribes to:**
| Topic | Type | Provided By | Available with Hand+Wrist? |
|-------|------|-------------|---------------------------|
| `data_streams/fingers/forces/data` | `ForceData` | mia_hand_driver | YES (if hand connected) |
| `/pipeline/state` | `Int32` | pipeline_manager | YES (internal) |
| `/joint_states` | `JointState` | command_bridge | YES (via bridge) |

**Service client:**
| Service | Provided By | Available? |
|---------|-------------|-----------|
| `data_streams/fingers/forces/switch` | mia_hand_driver | YES |

**Publishes to:**
| Topic | Consumed By | Works? |
|-------|-------------|--------|
| `/*/pos_ff_controller/commands` | command_bridge → mia_hand_driver | YES |
| `/force_controller/status` | pipeline_manager | YES |

**Verdict: Fully functional with hand+wrist.** The force controller only needs the Mia Hand driver + command bridge + pipeline state.

### Pipeline Manager (`pipeline_manager_node.py`)

**Subscribes to:**
| Topic | Type | Provided By | Available? |
|-------|------|-------------|-----------|
| `/emg/gesture_label` | `Int32` | emg_bridge | NO (no MindRove) |
| `/emg/confidence` | `Float32` | emg_bridge | NO |
| `/grasp_preshaping/grasp_type` | `Int32` | preshaping_service | YES (runs but needs cloud) |
| `/force_controller/status` | `ForceControllerStatus` | force_controller | YES |

**Service client:**
| Service | Provided By | Available? |
|---------|-------------|-----------|
| `/grasp_preshaping/compute_grasp` | preshaping_service | YES (runs but needs cloud) |

**Verdict: Starts and idles fine.** Won't transition out of IDLE without EMG input, but you can manually publish to `/emg/gesture_label` to trigger state transitions.

---

## 3. Issues and Gaps When Running with Hand+Wrist Only

### Issue 1: Serial Port Mismatch
- `pipeline.launch.py:68` hardcodes Mia Hand driver to `/dev/ttyUSB0`
- `docker-compose.yml:44-45` maps Mia Hand to `/dev/ttyUSB1` and Wrist to `/dev/ttyUSB0`
- **Conflict:** The Mia Hand driver will try to talk to the wrist Dynamixel, not the hand

### Issue 2: Wrist Driver Not Launched
- `pipeline.launch.py` does not include the wrist driver node
- You need to launch it separately: `ros2 launch wrist_driver wrist_driver.launch.py`
- Or run it directly: `ros2 run wrist_driver wrist_driver_node`

### Issue 3: No EMG Input
- Pipeline manager waits in IDLE for EMG gesture triggers
- Without MindRove, you must manually publish gestures:
  ```bash
  ros2 topic pub /emg/gesture_label std_msgs/Int32 '{data: 1}' --once  # POWER grasp
  ros2 topic pub /emg/gesture_label std_msgs/Int32 '{data: 3}' --once  # OPEN/release
  ```

### Issue 4: No Camera / No Segmentation
- Without a RealSense camera, segmentation and grasp preshaping won't produce real results
- The segmentation_bridge will fail to connect to inference server
- Preshaping service won't have a cloud to process
- Pipeline will fail at SEGMENTING → PLANNING transitions

### Issue 5: Command Bridge Dependency Chain
- The command bridge needs the Mia Hand driver running first to activate joint position streaming
- The force controller needs command bridge for `/joint_states`
- All three must be up for force feedback to work

---

## 4. Recommended Testing Approach

### Step-by-Step: Testing Force Controller on Real Hardware

#### Phase A: Bring Up the Hardware Stack
```bash
# 1. Start container with USB devices
make up-hw

# 2. Shell in
make shell

# 3. Verify devices
ls -la /dev/ttyUSB*

# 4. Launch the Mia Hand driver (correct port!)
ros2 run mia_hand_driver mia_hand_driver_node --ros-args -p serial_port:=/dev/ttyUSB1
```

#### Phase B: Launch the Bridge + Force Controller
In a second shell (`make shell`):
```bash
# 5. Launch command bridge
ros2 run command_bridge command_bridge_node --ros-params-file /prosthesis_ws/config/prosthesis_config.yaml

# 6. Launch force controller
ros2 run force_controller force_controller_node --ros-params-file /prosthesis_ws/config/prosthesis_config.yaml
```

#### Phase C: Launch Pipeline Manager
In a third shell:
```bash
# 7. Launch pipeline manager
ros2 run pipeline_manager pipeline_manager_node --ros-params-file /prosthesis_ws/config/prosthesis_config.yaml
```

#### Phase D: Launch Wrist Driver (optional)
In a fourth shell:
```bash
# 8. Launch wrist driver
ros2 run wrist_driver wrist_driver_node --ros-args -p port:=/dev/ttyUSB0
```

#### Phase E: Trigger the Pipeline Manually
In a fifth shell:
```bash
# 9. Trigger a grasp (POWER gesture = 1)
ros2 topic pub /emg/gesture_label std_msgs/Int32 '{data: 1}' --once

# 10. Watch pipeline state
ros2 topic echo /pipeline/state

# 11. Watch force controller status
ros2 topic echo /force_controller/status

# 12. Release (OPEN gesture = 3)
ros2 topic pub /emg/gesture_label std_msgs/Int32 '{data: 3}' --once
```

---

## 5. What Will NOT Work End-to-End

The full pipeline flow **cannot complete** with only hand+wrist because:

1. **IDLE → SEGMENTING:** Works (triggered by EMG gesture publish)
2. **SEGMENTING → PLANNING:** Fails — segmentation_bridge has no inference server, no camera cloud
3. **PLANNING → APPROACHING:** Fails — preshaping service has no segmented cloud
4. **APPROACHING → GRASPING:** Can't reach this state naturally

### Alternative: Direct Force Controller Testing

To test the force controller **without the full pipeline**, you can:

1. Manually set pipeline state to GRASPING:
   ```bash
   ros2 topic pub /pipeline/state std_msgs/Int32 '{data: 4}' --once
   ```
2. This activates the force controller (it watches for APPROACHING→GRASPING transition)
3. The force controller will try to activate force streaming and begin PI regulation
4. Monitor `/force_controller/status` for feedback

### Alternative: Bypass Pipeline, Test Hand Directly

For pure hand testing:
```bash
# Test single finger movement via command bridge
ros2 topic pub /thumb_pos_ff_controller/commands std_msgs/Float64MultiArray '{data: [0.5]}' --once

# Read force data
ros2 topic echo data_streams/fingers/forces/data

# Activate force streaming
ros2 service call data_streams/fingers/forces/switch std_srvs/srv/SetBool '{data: true}'
```

---

## 6. Summary: Will `make up-hw` Work As Intended?

| Aspect | Status | Notes |
|--------|--------|-------|
| Container starts | OK | USB devices mapped |
| Mia Hand driver connects | PROBLEM | Wrong port in pipeline.launch.py (`/dev/ttyUSB0` vs mapped `/dev/ttyUSB1`) |
| Wrist driver starts | MISSING | Not included in pipeline.launch.py |
| Force controller starts | OK | Once driver + bridge are up |
| Pipeline manager starts | OK | Idles correctly |
| Full pipeline flow | BROKEN | No EMG, no camera, no segmentation |
| Force regulation loop | TESTABLE | With manual state injection |

**Bottom line:** `make up-hw` gets you a container with USB access, but `pipeline.launch.py` has a serial port mismatch and is missing the wrist driver. For force controller testing, you're better off launching nodes individually with correct parameters rather than using the full launch file.
