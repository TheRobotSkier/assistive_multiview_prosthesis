# USB Device Path Consistency Fix

## Problem Statement

Two FTDI USB serial devices are connected:
- **MIA Hand** (FT232, serial `FTBY495J`) — always appears at `/dev/ttyUSB0`
- **Wrist Dynamixel** (FT232H, serial `FTAO4Z0Y`) — always appears at `/dev/ttyUSB1`

The host has udev rules (`/etc/udev/rules.d/99-prosthesis.rules`) that create stable symlinks:
- `/dev/ttyMiaHand` → `/dev/ttyUSB0` (MIA Hand)
- `/dev/ttyDynamixel` → `/dev/ttyUSB1` (Wrist)

The container (via `docker-compose.hw.yml`) maps `/dev/ttyUSB0` and `/dev/ttyUSB1` through. **The devices are accessible inside the container.** The user confirmed they can `cat` them without issues.

The entrypoint (already modified) creates matching symlinks inside the container: `/dev/ttyMiaHand` and `/dev/ttyDynamixel`.

**The actual problem:** The codebase uses hardcoded `/dev/ttyUSB0` and `/dev/ttyUSB1` in many places. If the devices ever swap enumeration order (e.g., different USB plug order), the MIA hand driver would try to talk to the Dynamixel and vice versa. The symlinks exist but are **never referenced** by any launch file, config, or script.

Additionally, there is one **active bug**: `config/emg_grasp_test.yaml:249` has `wrist_port: "/dev/ttyUSB0"` — this points the wrist driver at the MIA Hand's device instead of `/dev/ttyUSB1`.

## Current State of My Changes

I already made these changes (one rebuild already done):

1. **`docker/Dockerfile`** — Added `setserial` to apt packages (line 23)
2. **`docker/entrypoint.sh`** — Added USB device setup: `chmod 666`, low_latency via sysfs, creates `/dev/ttyMiaHand` and `/dev/ttyDynamixel` symlinks
3. **`docker/docker-compose.hw.yml`** — Added `MIA_SERIAL_PORT` and `WRIST_SERIAL_PORT` environment variables

These changes are **good and should be kept** — they ensure devices are accessible and symlinks exist. **No further container rebuilds are needed** since the entrypoint is bind-mounted at container start.

## Implementation Plan

The fix is purely about updating default device path references across the codebase. No rebuilds needed.

### Phase 1: Fix the active bug

- [ ] **1.1** Fix `config/emg_grasp_test.yaml:249` — change `wrist_port: "/dev/ttyUSB0"` to `wrist_port: "/dev/ttyUSB1"`
  This is an active bug: the wrist driver would try to open the MIA Hand's serial port.

### Phase 2: Update defaults in launch files to use environment variables

All launch files currently hardcode `/dev/ttyUSB0` and `/dev/ttyUSB1` as defaults. The pipeline.launch.py already uses `os.environ.get("MIA_SERIAL_PORT", ...)` — the others should follow the same pattern so the env vars from `docker-compose.hw.yml` propagate through.

- [ ] **2.1** Update `src/mia_hand_driver/launch/mia_hand_driver_launch.py:10` — change default from `'/dev/ttyUSB0'` to `os.environ.get("MIA_SERIAL_PORT", "/dev/ttyUSB0")`
  Add `import os` at the top of the file.

- [ ] **2.2** Update `src/mia_hand_ros2_control/launch/mia_hand_system_interface_launch.py:173` — change default from `'/dev/ttyUSB0'` to `os.environ.get("MIA_SERIAL_PORT", "/dev/ttyUSB0")`
  Add `import os` if not already present.

- [ ] **2.3** Update `src/mia_hand_ros2_control/launch/mia_hand_with_wrist_system_interface_launch.py:175` — change `serial_port` default from `'/dev/ttyUSB0'` to `os.environ.get("MIA_SERIAL_PORT", "/dev/ttyUSB0")`
  And line 181 — change `wrist_port` default from `'/dev/ttyUSB1'` to `os.environ.get("WRIST_SERIAL_PORT", "/dev/ttyUSB1")`
  Add `import os` if not already present.

- [ ] **2.4** Update `src/wrist_driver/wrist_driver_node.py:64` — change default from `'/dev/ttyUSB1'` to `os.environ.get("WRIST_SERIAL_PORT", "/dev/ttyUSB1")`
  Add `import os` if not already present.

### Phase 3: Update config files

- [ ] **3.1** Update `config/prosthesis_config.yaml:467-468` — These are documentation defaults read by `pipeline.launch.py` via yaml.safe_load. Change:
  - `mia_serial_port: "/dev/ttyUSB0"` → keep as-is (correct, matches env var default)
  - `wrist_port: "/dev/ttyUSB1"` → keep as-is (correct)

- [ ] **3.2** Update `config/emg_grasp_launch.yaml:25,29` — These are defaults for `emg_grasp_test.launch.py`. Already correct at `/dev/ttyUSB0` and `/dev/ttyUSB1`. No change needed.

- [ ] **3.3** Update `src/wrist_driver/config/wrist_params.yaml:7` — change `port: "/dev/ttyUSB1"` to `port: "/dev/ttyUSB1"`. Already correct, but note this is only used when launched standalone via `wrist_driver.launch.py` (which passes the params file directly). The pipeline.launch.py overrides this with its own parameter. No change needed.

### Phase 4: Update scripts

- [ ] **4.1** Update `scripts/emg_force_grasp.sh:36-37` — These already use env vars with `/dev/ttyUSB*` defaults:
  ```
  MIA_PORT="${MIA_PORT:-/dev/ttyUSB0}"
  WRIST_PORT="${WRIST_PORT:-/dev/ttyUSB1}"
  ```
  These are correct as-is since the env vars from `docker-compose.hw.yml` will propagate. No change needed.

### Phase 5: Verify docker-compose.hw.yml environment variables

- [ ] **5.1** Verify `docker-compose.hw.yml` passes the correct env vars. Already done in my earlier change:
  ```yaml
  environment:
    - MIA_SERIAL_PORT=/dev/ttyUSB0
    - WRIST_SERIAL_PORT=/dev/ttyUSB1
  ```
  These match the device mappings. No further change needed.

## Files NOT to change (documentation only)

These reference `/dev/ttyUSB*` in comments or docs — not worth changing as they describe the physical setup:
- `src/mia_hand_driver/README.md` — usage examples
- `docs/grasp_test_plan.md` — planning doc
- `plans/2026-05-13-hw-testing-force-controller-pipeline-v1.md` — historical plan
- `src/prosthesis_launch/launch/emg_grasp_test.launch.py:24` — comment in usage docstring

## Summary of actual changes needed

| File | Line | Current | New |
|------|------|---------|-----|
| `config/emg_grasp_test.yaml` | 249 | `wrist_port: "/dev/ttyUSB0"` | `wrist_port: "/dev/ttyUSB1"` |
| `src/mia_hand_driver/launch/mia_hand_driver_launch.py` | 10 | `'/dev/ttyUSB0'` | `os.environ.get("MIA_SERIAL_PORT", "/dev/ttyUSB0")` |
| `src/mia_hand_ros2_control/launch/mia_hand_system_interface_launch.py` | 173 | `'/dev/ttyUSB0'` | `os.environ.get("MIA_SERIAL_PORT", "/dev/ttyUSB0")` |
| `src/mia_hand_ros2_control/launch/mia_hand_with_wrist_system_interface_launch.py` | 175 | `'/dev/ttyUSB0'` | `os.environ.get("MIA_SERIAL_PORT", "/dev/ttyUSB0")` |
| `src/mia_hand_ros2_control/launch/mia_hand_with_wrist_system_interface_launch.py` | 181 | `'/dev/ttyUSB1'` | `os.environ.get("WRIST_SERIAL_PORT", "/dev/ttyUSB1")` |
| `src/wrist_driver/wrist_driver/wrist_driver_node.py` | 64 | `'/dev/ttyUSB1'` | `os.environ.get("WRIST_SERIAL_PORT", "/dev/ttyUSB1")` |

All changes are to Python source files that are bind-mounted into the container — **no rebuild required**.

## Verification Criteria

- [ ] `make up-hw` + `make shell` — verify `/dev/ttyMiaHand` and `/dev/ttyDynamixel` symlinks exist
- [ ] Inside container: `cat /dev/ttyMiaHand` and `cat /dev/ttyDynamixel` succeed (no permission error)
- [ ] Inside container: `echo $MIA_SERIAL_PORT` prints `/dev/ttyUSB0`
- [ ] Inside container: `echo $WRIST_SERIAL_PORT` prints `/dev/ttyUSB1`
- [ ] `ros2 launch prosthesis_launch pipeline.launch.py mia_hand:=true wrist:=true` — MIA hand driver connects on correct port, wrist driver connects on correct port (no cross-talk)

## Potential Risks

1. **`import os` missing** — Some launch files may not import `os`. Mitigation: check each file before editing and add the import if needed.
2. **Environment variables not set in non-hw mode** — When running `make dev` (without hw overlay), the env vars won't be set, so the hardcoded `/dev/ttyUSB*` fallback is used. This is fine — without hardware, the devices don't exist anyway.
3. **Existing entrypoint changes** — My earlier changes to `entrypoint.sh`, `docker-compose.hw.yml`, and `Dockerfile` are already baked into the current image. They should be kept as-is (they're correct and useful).
