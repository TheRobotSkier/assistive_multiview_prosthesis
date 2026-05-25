# Log v23 Issues — Diagnosis & Resolution Plan

## Objective

Identify and resolve all issues found in `host-log-v23.txt` (and the missing `jetson-log-v23.txt`) to achieve a stable, fully-functional pipeline run.

## Issues Found (Priority Order)

### Issue 1 — EMG Classifier Crash (FATAL) — `run_classifier` exit code 2
**Source:** `host-log-v23.txt:62-69`

The `run_classifier` node crashes immediately with:
```
run_classifier: error: unrecognized arguments: -r __node:=emg_bridge
```

**Root Cause:** The ROS 2 launch system appends `--ros-args -r __node:=emg_bridge` to the process command line (because the launch file sets `name="emg_bridge"` at `src/prosthesis_launch/launch/pipeline.launch.py:148`). The entry point `run_classifier_entry.py` strips `--ros-args` sections but does **not** strip the `-r __node:=emg_bridge` remapping argument. When `run_classifier.py` calls `argparse.parse_args()` at line 144, the leftover `-r` flag is treated as an unknown argument and argparse exits with code 2.

**Note:** User confirmed EMG is not trained yet, so the model file (`classifier.pkl`) won't exist either. However, the argparse crash happens *before* the model load, so fixing the argument parsing is the first step. After that, the missing model file will cause a graceful exit with a helpful message (lines 154-162), which is expected until training is done.

### Issue 2 — Twist Propagation Crash (FATAL) — Missing `_collision_distance_pub`
**Source:** `host-log-v23.txt:228-317`

After activation, `twist_propagation_node` spams `AttributeError: 'TwistPropagationNode' object has no attribute '_collision_distance_pub'` at ~50 Hz.

**Root Cause:** The publisher `_collision_distance_pub` is **used** at lines 1712 and 1776 of `src/twist_propagation/twist_propagation/twist_propagation_node.py` but is **never created** in the `__init__` method. The `_hit_time_pub` is created at line 713-714, but `_collision_distance_pub` was apparently added to the propagation logic without a corresponding `create_publisher()` call.

### Issue 3 — Wrist Driver Crash (FATAL) — Dynamixel `IndexError`
**Source:** `host-log-v23.txt:148-177`

The `wrist_driver_node` crashes with:
```
IndexError: list index out of range
```
inside `dynamixel_sdk.protocol2_packet_handler.read4ByteTxRx` (called from `_publish_state` at line 124 of `wrist_driver_node.py`).

**Root Cause:** The Dynamixel SDK's `read4ByteTxRx` returns a malformed response (fewer than 4 bytes). This typically happens when the servo loses communication (e.g., cable disconnect, EMI, or servo power loss). The Mia Hand logs show repeated disconnect/reconnect cycles (lines 108-222), suggesting a shared electrical issue (USB bus instability, power supply brownout). The wrist driver does not handle the SDK-level communication failure gracefully.

### Issue 4 — Mia Hand Repeated Disconnect/Reconnect (HIGH)
**Source:** `host-log-v23.txt:108-222`

The Mia Hand disconnects and reconnects every ~2-4 seconds throughout the entire run (at least 15 cycles).

**Root Cause:** Likely a physical connectivity issue — loose USB connection, insufficient power supply, or USB bus contention. Could also be a software-level serial timeout. This is a hardware/wiring issue, not a code bug.

### Issue 5 — OpenVINS Slow Initialization / TF Delay (MEDIUM)
**Source:** `host-log-v23.txt:56-146`

The `pointcloud_fusion_node` waits 94 seconds for the head OpenVINS chain and 115 seconds for the arm chain before TF connects. During this time, no fused pointclouds are published.

**Root Cause:** OpenVINS requires time to initialize and converge. The `marker_map` frame doesn't exist until OpenVINS publishes its first odometry estimate. This is a known startup latency issue — not a bug, but the long delay means the system is non-functional for ~2 minutes after launch.

### Issue 6 — Pointcloud Fusion Degraded Quality — Bbox Removal Failures (MEDIUM)
**Source:** `host-log-v23.txt:183, 204`

After TF connects, the bbox removal success rate is 0% initially, then 43%. The pruning boxes for `palm_frame` and `d435i_arm_bottom_screw_frame_8_cm_cam_mount` frequently fail to transform.

**Root Cause:** The TF tree intermittently disconnects for the arm camera frames (extrapolation errors, stale transforms). The Mia Hand disconnects may cause `palm_frame` to disappear from the TF tree. The arm camera mount frame depends on the arm OpenVINS chain which was unstable.

---

## Implementation Plan

### Phase 1: Critical Code Fixes

- [ ] **Fix 1a: Add `_collision_distance_pub` publisher in `TwistPropagationNode.__init__`**
  - File: `src/twist_propagation/twist_propagation/twist_propagation_node.py`
  - Add `self._collision_distance_pub = self.create_publisher(Float64, "/twist_propagation/collision_distance", 10)` near line 714 (next to `_hit_time_pub`).
  - Ensure the import for `Float64` from `std_msgs.msg` is present (it likely already is, since `_hit_time_pub` uses it).

- [ ] **Fix 1b: Handle missing `_collision_distance_pub` gracefully in `_run_idle_cycle`**
  - As a safety measure, add a `hasattr` guard or initialize with `None` and check before publishing. However, the proper fix (1a) should suffice. This is optional defensive coding.

- [ ] **Fix 2: Strip ROS remapping arguments in `run_classifier_entry.py`**
  - File: `src/emg_bridge/emg_bridge/run_classifier_entry.py`
  - The `_extract_ros_param` function already handles `--ros-args` sections but only extracts `-p` params. It should also strip `-r` (remap) arguments that ROS 2 injects (e.g., `-r __node:=emg_bridge`).
  - In the `ros_section` parsing loop (lines 34-44), add handling for `-r` entries: skip them (don't add to `cleaned`), similar to how `-p` is handled but without extracting a value.

- [ ] **Fix 3: Add error handling to `wrist_driver_node._publish_state`**
  - File: `src/wrist_driver/wrist_driver/wrist_driver_node.py`
  - Wrap the `read4ByteTxRx` calls in `_publish_state` (lines 124-127) in a try/except to catch `IndexError` from the SDK.
  - On failure, log a warning and skip publishing (don't crash the node).
  - Optionally add a reconnection mechanism or counter to log when communication is consistently failing.

### Phase 2: Hardware/Connectivity Investigation

- [ ] **Fix 4: Investigate Mia Hand disconnect/reconnect cycling**
  - Check the physical USB cable connection to the Mia Hand.
  - Verify power supply stability (shared power with wrist Dynamixel?).
  - Check USB bus bandwidth — multiple serial devices (Mia Hand on `/dev/ttyMiaHand`, Dynamixel on `/dev/ttyDynamixel`) may cause contention.
  - Consider adding a powered USB hub if both devices are on the same controller.
  - This is a hardware investigation, not a code change.

- [ ] **Fix 5: Investigate wrist Dynamixel communication stability**
  - The wrist crash (Issue 3) is likely related to the same electrical instability causing Mia Hand disconnects.
  - Check if the Dynamixel servo shares power or USB bus with the Mia Hand.
  - Verify the serial cable and connector integrity.

### Phase 3: Non-EMG Pipeline Validation

- [ ] **Fix 6: Address OpenVINS slow initialization**
  - This is expected behavior but consider: (a) documenting the expected warm-up time, (b) adding a health-check indicator, or (c) investigating if OpenVINS parameters can be tuned for faster convergence.
  - Not a code bug — defer unless it impacts usability.

- [ ] **Fix 7: Address bbox removal TF failures**
  - The `palm_frame` failures are likely caused by the Mia Hand disconnects (when the hand disconnects, its TF frames may disappear).
  - The `d435i_arm_bottom_screw_frame_8_cm_cam_mount` failures are caused by the arm OpenVINS chain being slow to stabilize.
  - Both should improve once the hardware connectivity issues (Fix 4) are resolved.

### Phase 4: EMG Training (Deferred)

- [ ] **Train the EMG classifier model**
  - Once the `run_classifier` argument parsing is fixed (Fix 2), the node will still exit because `classifier.pkl` doesn't exist.
  - Train the model using: `ros2 run emg_bridge train --data-dir /app/data --model-dir /app/models`
  - Collect training data first if not done: `ros2 run emg_bridge collect_data --output-dir /app/data`

---

## Verification Criteria

1. `run_classifier` node starts without argparse errors (exits gracefully with "Model file not found" message until EMG is trained — this is expected)
2. `twist_propagation_node` does not crash with `AttributeError` when activated
3. `wrist_driver_node` does not crash on Dynamixel communication errors — it logs a warning and continues
4. Mia Hand maintains stable connection (no repeated disconnect/reconnect)
5. Pointcloud fusion publishes dual-camera fused clouds with bbox removal success rate > 80%

## Potential Risks and Mitigations

1. **Dynamixel SDK IndexError may indicate deeper hardware fault**
   Mitigation: The try/except fix prevents the crash, but if the servo is truly dead, the wrist won't function. Verify hardware separately.

2. **ROS 2 argument stripping may miss other injected args**
   Mitigation: The `_extract_ros_param` function should be updated to strip ALL ROS 2 arguments (`-r`, `-p`, `--params-file`, etc.), not just `-p`. Consider a more robust approach: strip everything between `--ros-args` and `--`.

3. **Mia Hand disconnects may be caused by the wrist driver crash**
   Mitigation: Both devices share the USB bus. The wrist driver crash (and potential unclean serial port closure) may destabilize the bus. Fix the wrist driver first, then observe.

4. **Adding `_collision_distance_pub` may require downstream subscriber updates**
   Mitigation: The `pipeline_manager_node` already subscribes to collision distance via `_on_collision_distance` at line 514. Check that the topic name matches what the pipeline manager expects (currently `/twist_propagation/collision_distance` — verify this is correct).

## Alternative Approaches

1. **For Fix 2 (EMG args):** Instead of stripping ROS args in the entry point, could use `argparse.parse_known_args()` in `run_classifier.py` to silently ignore unknown arguments. This is simpler but masks potential typos in user-provided arguments.

2. **For Fix 3 (Wrist driver):** Instead of try/except on IndexError, could check `comm_result != COMM_SUCCESS` before accessing the return value. However, the SDK itself throws the error internally, so try/except is the only option without modifying the SDK.

3. **For Fix 4 (Mia Hand):** If hardware investigation doesn't resolve the disconnects, consider adding an auto-reconnect mechanism to the Mia Hand driver (it already reconnects, but the driver could be made more resilient to brief disconnections).
