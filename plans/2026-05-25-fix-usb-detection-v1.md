# Fix USB Device Detection — Once and For All

## Objective

Make the MIA Hand and Wrist Dynamixel always resolve to the correct serial port, regardless of USB enumeration order, replugging, or WSL passthrough quirks. No container rebuild required.

## Root Cause

The current `scripts/setup_usb_devices.sh` walks `/sys/class/tty/ttyUSB*/device` to find USB VID:PID attributes. This **does not work** inside the container — the sysfs tree is truncated by user namespace remapping, so the walk-up never reaches the USB device level.

As a result, the script finds nothing, and the codebase falls back to whatever `/dev/ttyMiaHand` and `/dev/ttyDynamixel` happen to point to on the host. These host udev symlinks can be **stale** (pointing to old `ttyUSB*` numbers after a replug), causing the hand driver to talk to the wrist and vice versa.

## Key Insight

`/dev/serial/by-id/` is the most reliable detection path. It's created by udevd on the host and contains stable symlinks named with the USB serial number:

```
/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_FTBY495J-if00-port0  → ../../ttyUSB1  (MIA Hand)
/dev/serial/by-id/usb-FTDI_FT232H_USB_UART_FTAO4Z0Y-if00-port0  → ../../ttyUSB0  (Wrist)
```

These are visible inside the container (because `privileged: true` exposes all host devices) and they **always point to the correct device** because udev matches by USB serial number, not by enumeration order.

## Implementation Plan

### Task 1. Rewrite `scripts/setup_usb_devices.sh` with robust detection

- [ ] Replace the current sysfs walk-up with a three-strategy detection approach:

  **Strategy A (primary): `/dev/serial/by-id/`**
  - List all symlinks in `/dev/serial/by-id/`
  - Match by USB serial number in the filename: `FTBY495J` = MIA Hand, `FTAO4Z0Y` = Wrist
  - Resolve each symlink to get the actual `/dev/ttyUSB*` target
  - This is the most reliable method and should almost always work

  **Strategy B (fallback): `/sys/bus/usb-serial/devices/`**
  - Each `ttyUSB*` has an entry here that symlinks to its USB interface directory
  - Use `readlink -f` to get the real path, then walk up to find `idVendor`/`idProduct`
  - Match by VID:PID: `0403:6001` = hand, `0403:6014` = wrist

  **Strategy C (last resort): `/sys/bus/usb/devices/`**
  - Walk all USB devices, find ones matching VID:PID
  - Find their `ttyUSB*` child via the `tty/` subdirectory

- [ ] After detection, always **overwrite** `/dev/ttyMiaHand` and `/dev/ttyDynamixel` with the correct targets using `ln -sf`
- [ ] Set `chmod 666` on both device nodes
- [ ] Attempt to set `low_latency` via sysfs (non-fatal if it fails)
- [ ] Print clear diagnostics showing what was found and what each symlink points to
- [ ] Exit with code 0 even if devices not found (don't block launch) but print warnings

### Task 2. Ensure the script runs before every launch

- [ ] Verify the host `Makefile:145-147` already calls `make setup-usb` before `make run` (it does from earlier changes)
- [ ] Verify the `Makefile.workspace` has the `setup-usb` target (it does from earlier changes)
- [ ] The entrypoint also runs USB setup at container start (harmless belt-and-suspenders)

### Task 3. Verify all code references use the symlinks

- [ ] All launch files and config files already updated to use `/dev/ttyMiaHand` and `/dev/ttyDynamixel` as defaults (done in earlier changes)
- [ ] No active code references raw `/dev/ttyUSB0` or `/dev/ttyUSB1` (confirmed — only comments remain)

## Verification Criteria

1. Run `make down && make up-hw && make shell` inside the container
2. Run `make setup-usb` — should detect both devices and print correct mapping
3. Verify symlinks: `ls -la /dev/ttyMiaHand /dev/ttyDynamixel` — should point to correct `ttyUSB*`
4. Verify `cat /dev/ttyMiaHand` returns quickly (MIA Hand sends periodic data)
5. Verify `cat /dev/ttyDynamixel` hangs (Dynamixel is silent until commanded)
6. Unplug and replug devices in different order, repeat steps 1-5 — symlinks should still be correct
7. Run `make run` — both drivers should connect without "Failed to open" or "no status packet" errors

## Potential Risks and Mitigations

1. **`/dev/serial/by-id/` not visible inside container**
   - Mitigation: Three fallback strategies. Also, with `privileged: true`, this directory should always be visible since it's part of the host devtmpfs.

2. **USB serial numbers change (different hardware unit)**
   - Mitigation: Strategy B and C match by VID:PID instead of serial number. The serial numbers are hardcoded in the FTDI EEPROM and don't change for the same physical device.

3. **Script runs but devices are not plugged in**
   - Mitigation: Script prints warnings but exits 0. Launch will fail with clear "device not found" errors from the drivers — this is expected and correct behavior.

## Files to Modify

| File | Change | Rebuild? |
|------|--------|----------|
| `scripts/setup_usb_devices.sh` | Rewrite with three-strategy detection | No (bind-mounted) |

No other files need changes. All launch files, configs, and Makefiles were already updated in earlier steps.
