# USB Device Symlink Fix

## Problem

The env var change to `docker-compose.hw.yml` broke device access. `/dev/ttyMiaHand` and `/dev/ttyDynamixel` don't exist inside the container because:
- Host udev symlinks don't propagate into the container
- The entrypoint tried to create them from env vars that now point to the symlink names themselves (circular)
- Even if we revert, the entrypoint just maps `/dev/ttyUSB0` → `/dev/ttyMiaHand` blindly — it can't tell which device is which

## Solution

### Step 1: Revert docker-compose.hw.yml env vars

Change back from symlink paths to raw paths:
```yaml
environment:
  - MIA_SERIAL_PORT=/dev/ttyUSB0
  - WRIST_SERIAL_PORT=/dev/ttyUSB1
```
This fixes the immediate breakage — the entrypoint can find the devices again.

### Step 2: Create scripts/setup_usb_devices.sh

A detection script that runs **inside the container** and:
1. Uses `lsusb` to find FTDI devices (lsusb works inside the container — the user confirmed this)
2. Identifies which is the hand (`0403:6001`) and which is the wrist (`0403:6014`)
3. Maps each USB device to its `/dev/ttyUSB*` via sysfs
4. Creates stable symlinks: `/dev/ttyMiaHand` → correct ttyUSB, `/dev/ttyDynamixel` → correct ttyUSB
5. Sets low_latency via sysfs

This is the key piece — it works regardless of USB enumeration order.

### Step 3: Update launch file defaults to use symlinks

Change the fallback defaults in all launch files from `/dev/ttyUSB*` to the symlink paths:
- `os.environ.get("MIA_SERIAL_PORT", "/dev/ttyMiaHand")` instead of `"/dev/ttyUSB0"`
- `os.environ.get("WRIST_SERIAL_PORT", "/dev/ttyDynamixel")` instead of `"/dev/ttyUSB1"`

When the env vars are set (from docker-compose.hw.yml), they override the defaults. When running on bare metal with udev, the symlinks exist. Either way, the correct device is used.

### Step 4: Add setup-usb target to Makefile.workspace

Add a `setup-usb` target that runs the detection script. The user runs it after `make shell`:
```
make shell
make setup-usb
make run
```

Or we wire it into the `run` target so it happens automatically.

## Files to change

| File | Change |
|------|--------|
| `docker/docker-compose.hw.yml` | Revert env vars to `/dev/ttyUSB0` and `/dev/ttyUSB1` |
| `scripts/setup_usb_devices.sh` | NEW — detection script using lsusb |
| `Makefile.workspace` | Add `setup-usb` target |
| `src/mia_hand_driver/launch/mia_hand_driver_launch.py` | Change fallback to `/dev/ttyMiaHand` |
| `src/mia_hand_ros2_control/launch/mia_hand_system_interface_launch.py` | Same |
| `src/mia_hand_ros2_control/launch/mia_hand_with_wrist_system_interface_launch.py` | Same for both ports |
| `src/wrist_driver/wrist_driver/wrist_driver_node.py` | Change fallback to `/dev/ttyDynamixel` |
| `src/prosthesis_launch/launch/pipeline.launch.py` | Change fallbacks to symlink paths |

## Verification

- [ ] `make up-hw && make shell`
- [ ] `bash /prosthesis_ws/scripts/setup_usb_devices.sh` — should detect both devices and create symlinks
- [ ] `ls -la /dev/ttyMiaHand /dev/ttyDynamixel` — symlinks point to correct ttyUSB devices
- [ ] `make run` — no "Failed to open" errors from hand or wrist drivers
