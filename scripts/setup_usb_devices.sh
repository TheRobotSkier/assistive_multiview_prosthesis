#!/usr/bin/env bash
# setup_usb_devices.sh — create stable symlinks for MIA Hand and Wrist Dynamixel.
#
# This runs INSIDE the container (as root, before dropping privileges).
# It reads MIA_SERIAL_PORT and WRIST_SERIAL_PORT env vars that were set
# by the host Makefile via host-side USB detection (detect_usb_host.sh).
#
# Known hardware:
#   MIA Hand    — FT232  (serial FTBY495J) → /dev/ttyMiaHand
#   Wrist Motor — FT232H (serial FTAO4Z0Y) → /dev/ttyDynamixel
#
# Usage:
#   sudo bash /prosthesis_ws/scripts/setup_usb_devices.sh
#   # Or via make: make setup-usb

set -euo pipefail

# ── Read env vars (set by host Makefile from detect_usb_host.sh) ──────────
MIA_PORT="${MIA_SERIAL_PORT:-/dev/ttyUSB0}"
WRIST_PORT="${WRIST_SERIAL_PORT:-/dev/ttyUSB1}"

MIA_HAND_LINK="/dev/ttyMiaHand"
WRIST_LINK="/dev/ttyDynamixel"

echo "[setup-usb] Configuration:"
echo "[setup-usb]   MIA_SERIAL_PORT  = $MIA_PORT"
echo "[setup-usb]   WRIST_SERIAL_PORT = $WRIST_PORT"

# ── Create symlinks ───────────────────────────────────────────────────────
if [ -e "$MIA_PORT" ]; then
    if ln -sf "$MIA_PORT" "$MIA_HAND_LINK" \
        && ln -sf "$MIA_PORT" "/dev/mia_hand"; then
        echo "[setup-usb] Linked $MIA_PORT → $MIA_HAND_LINK, /dev/mia_hand"
    else
        echo "[setup-usb] WARNING: could not create MIA device symlinks"
    fi
else
    echo "[setup-usb] WARNING: $MIA_PORT does not exist — MIA Hand not available"
fi

if [ -e "$WRIST_PORT" ]; then
    if ln -sf "$WRIST_PORT" "$WRIST_LINK" \
        && ln -sf "$WRIST_PORT" "/dev/wrist_motor"; then
        echo "[setup-usb] Linked $WRIST_PORT → $WRIST_LINK, /dev/wrist_motor"
    else
        echo "[setup-usb] WARNING: could not create wrist device symlinks"
    fi
else
    echo "[setup-usb] WARNING: $WRIST_PORT does not exist — Wrist Dynamixel not available"
fi

# ── Set permissions ───────────────────────────────────────────────────────
for dev in "$MIA_PORT" "$WRIST_PORT"; do
    [ -e "$dev" ] || continue

    chmod 666 "$dev" 2>/dev/null || true

    # Attempt to set 1ms latency timer for responsive serial communication
    base=$(basename "$dev")
    latency="/sys/class/tty/$base/device/latency_timer"
    if [ -w "$latency" ]; then
        echo 1 > "$latency" 2>/dev/null && \
            echo "[setup-usb]   $base: low_latency set (1ms)" || \
            echo "[setup-usb]   $base: low_latency failed (non-fatal)"
    fi
done

# ── Summary ───────────────────────────────────────────────────────────────
echo "[setup-usb] Done."
if [ -e "$MIA_PORT" ] && [ -e "$WRIST_PORT" ]; then
    echo "[setup-usb] MIA Hand  = $MIA_HAND_LINK → $MIA_PORT"
    echo "[setup-usb] Wrist     = $WRIST_LINK → $WRIST_PORT"
else
    echo "[setup-usb] One or more devices missing — check USB connections"
fi
