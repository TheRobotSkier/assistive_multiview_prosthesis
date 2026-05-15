# CppDriver Blocking Serial Read Loop Analysis

## 1. Current Read Flow

The ros2_control `read()` method executes three serial reads sequentially on every controller update tick.

### Call chain (per update cycle, 5 Hz / 200 ms)

```
MiaHandSystemInterface::read()                        [system_interface.cpp:319-371]
  |
  +-- CppDriver::get_joint_positions()                [cpp_driver.cpp:345-405]
  |     |
  |     +-- send_command("@ADJo..*\r")                [cpp_driver.cpp:1284-1339]
  |     |     |-- serial_port_.Write(18 bytes)        Write command
  |     |     |-- serial_port_.DrainWriteBuffer()      Flush to wire
  |     |     |-- serial_port_.Read(rx_msg_, 17, 20)   Blocking read ACK, 20 ms timeout
  |     |     +-- Validate ACK format (echoed cmd)
  |     |
  |     +-- serial_port_.Read(rx_msg_, 15, 20)        Read joint position data, 20 ms timeout
  |     +-- Parse "JP" response (3 joint positions)
  |
  +-- CppDriver::get_joint_speeds()                   [cpp_driver.cpp:407-464]
  |     |
  |     +-- send_command("@ADjo..*\r")                [cpp_driver.cpp:1284-1339]
  |     |     |-- serial_port_.Write(18 bytes)
  |     |     |-- serial_port_.DrainWriteBuffer()
  |     |     |-- serial_port_.Read(rx_msg_, 17, 20)   Blocking read ACK, 20 ms timeout
  |     |     +-- Validate ACK format
  |     |
  |     +-- serial_port_.Read(rx_msg_, 12, 20)        Read joint speed data, 20 ms timeout
  |     +-- Parse "JS" response (3 joint speeds)
  |
  +-- CppDriver::get_finger_forces()                  [cpp_driver.cpp:466-553]
        |
        +-- send_command("@ADAo..*\r")                [cpp_driver.cpp:1284-1339]
        |     |-- serial_port_.Write(18 bytes)
        |     |-- serial_port_.DrainWriteBuffer()
        |     |-- serial_port_.Read(rx_msg_, 17, 20)   Blocking read ACK, 20 ms timeout
        |     +-- Validate ACK format
        |
        +-- serial_port_.Read(rx_msg_, 76, 20)        Read force data, 20 ms timeout
        +-- Parse "a" response (6 force values)
```

### Error suppression

Error returns from the three `get_*` calls are suppressed via commented-out TODO comments at `mia_hand_system_interface.cpp:366-367` and `mia_hand_system_interface.cpp:389`, meaning the ros2_control framework is **not notified** when serial reads fail. The `read()` method always returns `hardware_interface::return_type::OK`.

---

## 2. Timing Analysis

### Per `send_command()`

| Step | Bytes | Timeout | Best case |
|------|-------|---------|-----------|
| `Write()` | 18 bytes | — | < 1 ms |
| `DrainWriteBuffer()` | — | — | < 1 ms |
| `Read()` ACK | 17 bytes | 20 ms | 1–5 ms (typical serial round-trip) |

### Per `get_joint_positions()`

| Step | Bytes | Timeout | Best case |
|------|-------|---------|-----------|
| `send_command()` | 18 + 17 | 20 ms | ~5 ms |
| `Read()` data | 15 | 20 ms | ~2 ms |
| **Total** | | | **~7 ms** |

### Per `get_joint_speeds()`

| Step | Bytes | Timeout | Best case |
|------|-------|---------|-----------|
| `send_command()` | 18 + 17 | 20 ms | ~5 ms |
| `Read()` data | 12 | 20 ms | ~2 ms |
| **Total** | | | **~7 ms** |

### Per `get_finger_forces()`

| Step | Bytes | Timeout | Best case |
|------|-------|---------|-----------|
| `send_command()` | 18 + 17 | 20 ms | ~5 ms |
| `Read()` data | 76 | 20 ms | ~5 ms |
| **Total** | | | **~10 ms** |

### Aggregate per `read()` cycle

| Metric | Value |
|--------|-------|
| Serial write operations | 3 (54 bytes total) |
| Serial read operations | 6 (154 bytes total) |
| Timeout budget | 120 ms (6 × 20 ms) |
| Best-case wall clock | ~24 ms |
| Worst-case wall clock (all timeouts) | 120+ ms |
| ros2_control update period | 200 ms (5 Hz) |
| **Best-case duty cycle** | **12% of period** |
| **Worst-case duty cycle** | **60%+ of period** |

### Additional timing factors

- The MIA hand firmware is half-duplex: it processes one command at a time and needs to finish responding before accepting the next.
- Each 18-byte command takes ~1.5 ms to transmit at 115200 baud (8N1). The firmware must then parse the command, sample sensors, format the reply, and transmit it.
- Sending commands back-to-back without an idle gap risks the hand's receive buffer being ready for the next byte stream before the previous reply retrieval completes.
- With three `send_command` calls firing sequentially, the **second** command may arrive while the hand is still transmitting the reply to the **first**, causing framing errors or garbled ACKs.

---

## 3. Root Cause

**The `read()` method fires three independent serial read requests (positions, speeds, forces) back-to-back on every ros2_control update tick.** Each request involves a command transmission, an ACK handshake, and a data read — all blocking — totaling 6 serial I/O operations per cycle.

This overwhelms the half-duplex serial protocol:

1. The MIA hand's UART has a finite receive buffer. When commands arrive faster than the firmware can process them, bytes are lost or mis-framed.
2. ACK responses (17 bytes each) must arrive within 20 ms. If the hand is still transmitting the previous response, the ACK read times out or captures stale/garbled data.
3. The `send_command()` ACK validation (`cpp_driver.cpp:1331`) compares the echoed command payload. A corrupted ACK fails validation, returning `false` with "Invalid ACK" error.
4. On timeout, `FlushInputBuffer()` is called, which discards any in-flight data — potentially including the data response the next read expects.
5. The hand's firmware does not appear to support pipelined commands.

---

## 4. Why It Manifested With ros2_control But Not Standalone Driver

### Standalone `mia_hand_driver_node` (`ros_driver.cpp`)

The standalone driver exposes serial read operations through two mechanisms:

| Mechanism | Trigger | Frequency |
|-----------|---------|-----------|
| ROS2 services | User calls service (on-demand) | Sporadic, user-driven |
| Streaming timer (`stream_tmr_fun`) | User toggles individual stream on via service | Configurable, defaults to OFF |

**Key differences:**

- **Services** (`get_jnt_pos_srv_fun`, `get_fin_for_srv_fun`, etc.) are invoked **on demand** by a user's ROS2 service client. Between calls there is typically a human-scale gap (seconds), allowing the hand ample time to return to idle.
- **Streaming** is **disabled by default** (`stream_tmr_->cancel()` at `ros_driver.cpp:80`). The user must explicitly enable each data stream (motor position, speed, current, joint position, speed, finger force) via separate service calls. Even when enabled, the timer period scales with the number of active streams (`10ms * n_streams_`, `ros_driver.cpp:440`), spacing out the aggregate load, and typically only 1–2 streams are active simultaneously.
- **Action server loops** (grasp actions) make single `get_*` calls in their feedback loops and include `rate.sleep()` calls, adding natural pacing.

### ros2_control `MiaHandSystemInterface` (`system_interface.cpp`)

- The `read()` method is called by the ros2_control Controller Manager at a fixed 5 Hz rate (200 ms period), **unconditionally** and **automatically**.
- All three `get_*` calls are always invoked inside `read()`. There is no mechanism to skip or throttle specific reads.
- The `read()` method is called in the **real-time control thread**, making the blocking I/O particularly problematic as it stalls the entire control loop.

---

## 5. Recommended Fix Options

Ranked by effort and impact. Option A or B recommended depending on development bandwidth.

### Option A: Dedicated Serial Polling Thread (Best — High Effort)

**Design:** A background thread continuously polls the MIA hand (positions, speeds, forces) into a thread-safe cache. The `read()` method simply copies from the cache. The `write()` method sends commands directly to the serial port.

**Pros:**
- Eliminates blocking I/O from the real-time control thread entirely.
- Individual polling rates can be tuned per data type (e.g., positions at 50 Hz, forces at 20 Hz).
- Cache can be updated independently of control loop frequency.
- Thread can implement retry logic, backoff, and error recovery without affecting the controller.

**Cons:**
- Requires careful synchronization (mutex or lock-free ring buffer) between poller and `read()`.
- More complex implementation and testing.
- Race condition: `write()` commands sent concurrently with the polling thread's serial reads must be serialized.

**Implementation outline:**
```
Thread loop:
  while (running) {
    lock serial mutex
    get_joint_positions()   → write to cache
    sleep(5ms)
    get_joint_speeds()      → write to cache
    sleep(5ms)
    get_finger_forces()     → write to cache
    unlock serial mutex
    sleep(10ms)
  }

read():
  lock cache mutex
  copy cached values → state arrays
  unlock cache mutex

write():
  lock serial mutex
  set_joint_trajectory() or set_joint_speed()
  unlock serial mutex
```

---

### Option B: Interleaved Reads (Good — Low Effort)

**Design:** Instead of reading all three data types every cycle, cycle through them one per `read()` call, optionally with a counter or timer.

**Pros:**
- Minimal code change — modify only `mia_hand_system_interface.cpp:read()`.
- Virtually eliminates serial congestion (1 send_command + 1 data read per cycle).
- No threading complexity.

**Cons:**
- Each data type updates at 5/3 ≈ 1.7 Hz. This may be too slow for position feedback in tight control loops.
- Force data is especially slow to update (every 600 ms).
- Does not solve the underlying blocking I/O problem; each read still blocks the real-time thread for ~7–10 ms.

**Implementation outline (in `read()`):**
```
static int cycle = 0;
switch (cycle % 3) {
  case 0: get_joint_positions(...); break;
  case 1: get_joint_speeds(...);    break;
  case 2: get_finger_forces(...);   break;
}
cycle++;
```

---

### Option C: Flush Between Reads (Band-Aid — Trivial Effort)

**Design:** Call `serial_port_.FlushIOBuffers()` between each `get_*` call inside `read()`, or inside `send_command()` before writing.

**Pros:**
- One or two lines of code.
- Clears any stale data that might cause ACK corruption.

**Cons:**
- Does **not** address the root cause of back-to-back commands overwhelming the hand firmware.
- Flushing adds latency and may discard legitimate response data still in transit.
- May reduce error rate but will not eliminate it, especially under load.
- Flushing the input buffer after a timeout is already done in `send_command()` (`cpp_driver.cpp:1323,1335`), so the benefit of additional flushing is marginal.

---

### Option D: Combine Into Single Command (Ideal — Requires Firmware Support)

**Design:** If the MIA hand firmware supports a "read all sensor data" command, replace the three separate `get_*` calls with one serial exchange returning all positions, speeds, and forces in a single response packet.

**Pros:**
- Single send_command + single data read per cycle → ~10 ms total.
- Completely eliminates serial congestion.
- Simplest and cleanest architecture.

**Cons:**
- Requires firmware investigation and possibly a firmware update on the MIA hand itself.
- May not be feasible if the firmware does not support such a command.
- Requires coordination with the MIA hand vendor (Prensilia).

---

## 6. References

| Component | File | Lines |
|-----------|------|-------|
| `read()` method | `src/mia_hand_ros2_control/src/mia_hand_ros2_control/mia_hand_system_interface.cpp` | 319–371 |
| `send_command()` | `src/mia_hand_driver/src/mia_hand_driver/cpp_driver.cpp` | 1284–1339 |
| `get_joint_positions()` | `src/mia_hand_driver/src/mia_hand_driver/cpp_driver.cpp` | 345–405 |
| `get_joint_speeds()` | `src/mia_hand_driver/src/mia_hand_driver/cpp_driver.cpp` | 407–464 |
| `get_finger_forces()` | `src/mia_hand_driver/src/mia_hand_driver/cpp_driver.cpp` | 466–553 |
| `get_motor_positions()` | `src/mia_hand_driver/src/mia_hand_driver/cpp_driver.cpp` | 162–222 |
| `get_motor_speeds()` | `src/mia_hand_driver/src/mia_hand_driver/cpp_driver.cpp` | 224–281 |
| `get_grasp_refs()` | `src/mia_hand_driver/src/mia_hand_driver/cpp_driver.cpp` | 966–1077 |
| Standalone driver `stream_tmr_fun()` | `src/mia_hand_driver/src/mia_hand_driver/ros_driver.cpp` | 3621–3684 |
| Standalone driver service callbacks | `src/mia_hand_driver/src/mia_hand_driver/ros_driver.cpp` | 675–720 |
| Standalone driver streaming init/toggle | `src/mia_hand_driver/src/mia_hand_driver/ros_driver.cpp` | 74–80, 430–444 |
| Class declaration | `src/mia_hand_driver/include/mia_hand_driver/cpp_driver.hpp` | 155–249 |
