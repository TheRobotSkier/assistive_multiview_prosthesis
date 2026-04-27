# haptic_band — Bluetooth–ROS 2 Bridge

Bridges a single **Vibro8** Bluetooth Classic haptic module to the ROS 2 Jazzy network.
The Vibro8 drives 8 ERM vibro motors independently.

---

## Architecture

```
ROS 2 publisher
      │
      │  /haptic_band/motors
      │  std_msgs/Float32MultiArray  [8 values, 0.0–100.0]
      ▼
haptic_bridge_node   (Python, running in Docker)
      │
      └─ BT Classic RFCOMM channel 1 ─► Vibro8 device  (motors 1–8)
```

The node maintains a persistent Bluetooth connection in a background thread
and retries automatically on disconnect.

On first connection the bridge sends a short **buzz-buzz** pulse on all 8 motors
(2 × 0.2 s at 80 % intensity) to confirm the hardware is live before entering
normal ROS-driven operation.

---

## Communication Protocol

The bridge uses the **VBA** (Vibration — All) command from the Vibro8 v1.0
communication protocol:

```
>VBA;b1b2b3b4b5b6b7b8<
```

- `>` / `<` — start / end delimiters (0x3E / 0x3C)
- `VBA;` — ASCII command identifier + separator
- `b1`…`b8` — raw `uint8` intensity values, 0–100 (% of full drive)
- Total: **14 bytes**, no checksum required

On first connect the node also sends `>SH;0<` to disable the automatic search
for a Michelangelo prosthetic hand (required by the firmware).

---

## ROS 2 Interface

| Item | Value |
|---|---|
| Topic | `/haptic_band/motors` |
| Message type | `std_msgs/Float32MultiArray` |
| Data length | **8 elements** |
| Value range | `0.0` (off) – `100.0` (full intensity) |
| Indices 0–7 | Motors 1–8 on the Vibro8 |

**Example publish:**
```bash
ros2 topic pub /haptic_band/motors std_msgs/Float32MultiArray \
  '{data: [50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0]}'
```

---

## Running

```bash
# From docker_ws/docker-deployment/

# Start the bridge (first time or after code changes — rebuilds image):
docker compose run --build --rm haptic_band

# Subsequent runs (image already built):
docker compose run --rm haptic_band

# Run the motor sweep test (activates each motor 1–8 in sequence):
docker compose run --rm haptic_band_test
```

### Bluetooth address configuration

The default address matches the hardware unit used during development.
Override it per-run via an environment variable:

```bash
HAPTIC_BT_ADDR1=AA:BB:CC:DD:EE:FF docker compose run --rm haptic_band
```

Addresses may be given with or without colons; both formats are accepted.

---

## File layout

```
haptic_band/
├── Dockerfile                   # ros:jazzy-ros-base + bluez + rmw_cyclonedds_cpp
├── ros_entrypoint.sh            # sources ROS & workspace, execs CMD
├── README.md                    # this file
├── scripts/
│   └── motor_test.py            # sequential motor sweep test (8 motors)
└── haptic_bridge/               # ROS 2 ament_python package
    ├── package.xml
    ├── setup.py / setup.cfg
    ├── resource/haptic_bridge
    └── haptic_bridge/
        ├── __init__.py
        └── bridge_node.py       # BtDevice + HapticBridgeNode
```

---

## Notes

- The container uses `network_mode: host` and `privileged: true` so it can
  access the host Bluetooth adapter and join the same CycloneDDS ROS 2 network
  as the other services.
- The Vibro8 firmware ignores `VBD` (duration) and `VBW` (on/off bitmask)
  commands — only `VBA` and `VBI` (single-motor intensity) are reliable.
- If the Bluetooth connection drops, the bridge retries automatically every 3 s
  and silently drops any commands received while disconnected.
- The buzz-on-connect only fires on the **first** connection after node startup;
  subsequent reconnects after a drop skip the buzz.
