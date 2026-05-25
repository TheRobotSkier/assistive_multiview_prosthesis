#!/usr/bin/env bash
# Prosthesis container entrypoint
# 1. Fixes ownership of named Docker volumes (build/, install/, log/) which are
#    created as root:root by Docker.  Without this, colcon build fails with
#    PermissionError when trying to write log directories.
# 2. Fixes USB serial device permissions that are masked by userns keep-id.
#    Device nodes appear as nobody:nogroup and need chmod 666 for access.
#    (chown is NOT possible under userns keep-id — container root can only chmod.)
# 3. Creates persistent device symlinks for known hardware.
# 4. Applies low_latency for responsive serial communication.
set -e

# ── Volume ownership ────────────────────────────────────────────────────────
VOLUME_DIRS=(/prosthesis_ws/build /prosthesis_ws/install /prosthesis_ws/log)

for dir in "${VOLUME_DIRS[@]}"; do
    if [ -d "$dir" ] && [ "$(stat -c '%U' "$dir")" = "root" ]; then
        echo "[entrypoint] Fixing ownership of $dir -> prosthesis:prosthesis"
        chown -R prosthesis:prosthesis "$dir"
    fi
done

# ── USB serial device setup ────────────────────────────────────────────────
# With userns_mode: keep-id, device nodes are remapped to nobody:nogroup
# inside the container, making them inaccessible to the prosthesis user.
# We fix ownership and create predictable symlinks here (since udev doesn't
# run inside containers).
#
# Known hardware (from host udev rules at /etc/udev/rules.d/99-prosthesis.rules):
#   FT232 (0403:6001, serial FTBY495J) — MIA Hand          → /dev/ttyUSB0
#   FT232H (0403:6014, serial FTAO4Z0Y) — Wrist Dynamixel  → /dev/ttyUSB1
#
# We identify devices by the order they are mapped in docker-compose.hw.yml
# because USB device attributes are not readable through the user namespace.
# Environment variables can override the defaults:
#   MIA_SERIAL_PORT  (default: /dev/ttyUSB0)
#   WRIST_SERIAL_PORT (default: /dev/ttyUSB1)

MIA_SERIAL_PORT="${MIA_SERIAL_PORT:-/dev/ttyUSB0}"
WRIST_SERIAL_PORT="${WRIST_SERIAL_PORT:-/dev/ttyUSB1}"

for dev in "$MIA_SERIAL_PORT" "$WRIST_SERIAL_PORT"; do
    [ -e "$dev" ] || { echo "[entrypoint] WARNING: $dev not found — skipping"; continue; }

    echo "[entrypoint] Setting up $dev for prosthesis user"

    # With userns keep-id, device nodes appear as nobody:nogroup.
    # Container root cannot chown them, but CAN chmod to make them accessible.
    chown prosthesis:prosthesis "$dev" 2>/dev/null || true
    chmod 666 "$dev" || true

    # Set 1 ms latency timer via sysfs for responsive serial communication.
    # This replaces what 'setserial low_latency' would do.
    latency="/sys/class/tty/$(basename "$dev")/device/latency_timer"
    if [ -w "$latency" ]; then
        echo 1 > "$latency"
        echo "[entrypoint]   low_latency set via $latency"
    fi
done

# Create persistent symlinks so ROS2 launch files can refer to devices by name.
# This mirrors what the host udev rules do with SYMLINK+=.
if [ -e "$MIA_SERIAL_PORT" ] && [ ! -e /dev/ttyMiaHand ]; then
    ln -sf "$MIA_SERIAL_PORT" /dev/ttyMiaHand
    ln -sf "$MIA_SERIAL_PORT" /dev/mia_hand
    echo "[entrypoint] Symlinked $MIA_SERIAL_PORT -> /dev/ttyMiaHand, /dev/mia_hand"
fi

if [ -e "$WRIST_SERIAL_PORT" ] && [ ! -e /dev/ttyDynamixel ]; then
    ln -sf "$WRIST_SERIAL_PORT" /dev/ttyDynamixel
    ln -sf "$WRIST_SERIAL_PORT" /dev/wrist_motor
    echo "[entrypoint] Symlinked $WRIST_SERIAL_PORT -> /dev/ttyDynamixel, /dev/wrist_motor"
fi

# Drop root privileges — run as the prosthesis user
exec gosu prosthesis "$@"
