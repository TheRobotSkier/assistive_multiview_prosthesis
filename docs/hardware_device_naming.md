# Persistent Hardware Device Naming

Linux assigns `/dev/ttyUSB*` names in USB enumeration order, which can change
after reboot or reconnect. This document explains how to create stable symlinks
for the MIA hand and wrist Dynamixel.

## Known Hardware

| Device         | FTDI Serial  | Typical ttyUSB     |
|----------------|--------------|---------------------|
| MIA Hand       | FTAO4Z0Y     | `/dev/ttyUSB0`      |
| Wrist Dynamixel| FTBY495J     | `/dev/ttyUSB1`      |

## Identifying Devices

List serial-by-id symlinks:

```bash
ls -la /dev/serial/by-id/
```

Each FTDI device appears as `usb-FTDI_*-<serial>-if00-port0`. Match the serial
portion (`FTAO4Z0Y` or `FTBY495J`) to identify which is which.

Inspect a specific device in detail:

```bash
udevadm info -a -n /dev/ttyUSB0
```

## Creating Persistent Symlinks

Copy the udev rule below to `/etc/udev/rules.d/99-prosthesis.rules`:

```udev
# MIA Hand — FTDI FTAO4Z0Y
SUBSYSTEM=="tty", ATTRS{serial}=="FTAO4Z0Y", SYMLINK+="mia_hand"

# Wrist Dynamixel — FTDI FTBY495J
SUBSYSTEM=="tty", ATTRS{serial}=="FTBY495J", SYMLINK+="wrist_motor"
```

Then reload and trigger:

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

The persistent symlinks `/dev/mia_hand` and `/dev/wrist_motor` will now point to
the correct devices regardless of USB enumeration order.

## Verification

```bash
ls -la /dev/mia_hand /dev/wrist_motor
# lrwxrwxrwx 1 root root 7 ... /dev/mia_hand -> ttyUSB0
# lrwxrwxrwx 1 root root 7 ... /dev/wrist_motor -> ttyUSB1
```

## Using with Docker

When the udev rules are in place, override the environment variables:

```bash
export MIA_SERIAL_PORT=/dev/mia_hand
export WRIST_PORT=/dev/wrist_motor
```

If udev rules are not installed, fall back to the ttyUSB names (which may be
unstable across reboots):

```bash
export MIA_SERIAL_PORT=/dev/ttyUSB0
export WRIST_PORT=/dev/ttyUSB1
```
