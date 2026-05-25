# Fix USB Device Detection — Host-Side Detection (v2)

## Objective

Reliably identify which `ttyUSB*` is the MIA Hand and which is the Wrist Dynamixel, regardless of USB enumeration order. No container rebuild.

## Root Cause

1. `/dev/serial/by-id/` (the reliable detection path) does **not exist** inside the container
2. Sysfs USB attribute walking fails inside the container due to user namespace remapping
3. The entrypoint blindly creates symlinks from env var defaults (`/dev/ttyUSB0` → hand, `/dev/ttyUSB1` → wrist) — this is a guess that's often wrong
4. Host udev symlinks (`/dev/ttyMiaHand`, `/dev/ttyDynamixel`) can also be stale after replugging

## Solution: Host-Side Detection

The host has full udev, sysfs, and `/dev/serial/by-id/`. Run detection on the host, pass the resolved device paths into the container.

## Implementation Plan

### Task 1. Create `scripts/detect_usb_host.sh` (runs on the HOST)

- [ ] New script at `scripts/detect_usb_host.sh`
- [ ] Read `/dev/serial/by-id/` on the host
- [ ] Match by USB serial number in the filename:
  - `FTBY495J` (appears as `usb-FTDI_TTL-232R-3V3-AJ_FTBY495J-if00-port0`) → MIA Hand
  - `FTAO4Z0Y` (appears as `usb-FTDI_USB__-__Serial_Converter_FTAO4Z0Y-if00-port0`) → Wrist Dynamixel
- [ ] Resolve each symlink target to get the actual `/dev/ttyUSB*`
- [ ] Output shell `export` statements, e.g.:
  ```
  export DETECTED_MIA_PORT=/dev/ttyUSB1
  export DETECTED_WRIST_PORT=/dev/ttyUSB0
  ```
- [ ] If a device is not found, output a warning but still export an empty value
- [ ] Make executable

### Task 2. Simplify `scripts/setup_usb_devices.sh` (runs INSIDE the container)

- [ ] Remove all sysfs detection logic (it doesn't work inside the container)
- [ ] Read `MIA_SERIAL_PORT` and `WRIST_SERIAL_PORT` env vars (set by the host Makefile)
- [ ] Create symlinks: `/dev/ttyMiaHand` → `$MIA_SERIAL_PORT`, `/dev/ttyDynamixel` → `$WRIST_SERIAL_PORT`
- [ ] Also create alias symlinks: `/dev/mia_hand`, `/dev/wrist_motor`
- [ ] `chmod 666` both device nodes
- [ ] Set `low_latency` via sysfs (non-fatal)
- [ ] Print diagnostics

### Task 3. Update host `Makefile` — wire host detection into `run` target

- [ ] The `run` target currently does:
  ```makefile
  run: dev
  	cd $(COMPOSE_DIR) && $(COMPOSE) exec prosthesis /bin/bash -lc 'make setup-usb' && \
  	cd $(COMPOSE_DIR) && $(COMPOSE) exec --user prosthesis prosthesis /bin/bash -lc 'make run'
  ```
- [ ] Change to:
  ```makefile
  run: dev
  	$(eval DETECTED := $(shell bash scripts/detect_usb_host.sh))
  	$(eval DETECTED_MIA_PORT := $(shell bash scripts/detect_usb_host.sh | grep MIA_PORT | cut -d= -f2))
  	$(eval DETECTED_WRIST_PORT := $(shell bash scripts/detect_usb_host.sh | grep WRIST_PORT | cut -d= -f2))
  	cd $(COMPOSE_DIR) && $(COMPOSE) exec \
  		-e MIA_SERIAL_PORT=$(DETECTED_MIA_PORT) \
  		-e WRIST_SERIAL_PORT=$(DETECTED_WRIST_PORT) \
  		prosthesis /bin/bash -lc 'make setup-usb' && \
  	cd $(COMPOSE_DIR) && $(COMPOSE) exec --user prosthesis prosthesis /bin/bash -lc 'make run'
  ```

  Actually, a cleaner approach — have the detect script output a file or use `$(shell ...)` more cleanly:

  ```makefile
  run: dev
  	@DETECTED=$$(bash scripts/detect_usb_host.sh) && eval "$$DETECTED" && \
  	echo "[host] Detected: MIA=$$DETECTED_MIA_PORT  WRIST=$$DETECTED_WRIST_PORT" && \
  	cd $(COMPOSE_DIR) && $(COMPOSE) exec \
  		-e MIA_SERIAL_PORT="$$DETECTED_MIA_PORT" \
  		-e WRIST_SERIAL_PORT="$$DETECTED_WRIST_PORT" \
  		prosthesis /bin/bash -lc 'make setup-usb' && \
  	cd $(COMPOSE_DIR) && $(COMPOSE) exec --user prosthesis prosthesis /bin/bash -lc 'make run'
  ```

### Task 4. Verify the entrypoint doesn't interfere

- [ ] The entrypoint currently creates symlinks too (from earlier changes). This is harmless — `setup_usb_devices.sh` runs after and overwrites them with the correct mapping. No change needed.

## Verification Criteria

1. Run `bash scripts/detect_usb_host.sh` on the host — should output correct `DETECTED_MIA_PORT` and `DETECTED_WRIST_PORT`
2. Verify the values match reality: `cat $DETECTED_MIA_PORT` returns quickly, `cat $DETECTED_WRIST_PORT` hangs
3. Run `make run` — the host detects, passes to container, container creates symlinks, drivers connect
4. Unplug and replug devices in different order, repeat — detection should still be correct

## Potential Risks and Mitigations

1. **`/dev/serial/by-id/` doesn't exist on host**
   - Mitigation: Fallback to parsing `lsusb` + sysfs on the host (host has full sysfs access)
   - Also print a clear error message telling the user to check USB connections

2. **Devices not plugged in**
   - Mitigation: Script outputs empty values with warnings. Drivers will fail with clear errors.

3. **Multiple FTDI devices of same type**
   - Mitigation: Match by unique serial number (`FTBY495J` / `FTAO4Z0Y`), not by VID:PID

## Files to Modify

| File | Change | Rebuild? |
|------|--------|----------|
| `scripts/detect_usb_host.sh` | **NEW** — host-side USB detection | No |
| `scripts/setup_usb_devices.sh` | Simplify — just reads env vars and creates symlinks | No |
| `Makefile` | Wire host detection into `run` target | No |
