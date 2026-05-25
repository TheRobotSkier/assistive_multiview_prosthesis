#!/usr/bin/env bash
# detect_usb_host.sh — Host-side USB serial device detection.
#
# Runs on the HOST (not inside the container) where /dev/serial/by-id/ is
# available.  Identifies which /dev/ttyUSB* belongs to the MIA Hand and
# which belongs to the Wrist Dynamixel by matching the USB serial number
# embedded in the by-id symlink filename.
#
# Known hardware:
#   MIA Hand    — FTDI TTL-232R-3V3, serial FTBY495J
#   Wrist Motor — FTDI USB-Serial Converter, serial FTAO4Z0Y
#
# Output: shell export statements (sourceable)
#   export DETECTED_MIA_PORT=/dev/ttyUSB1
#   export DETECTED_WRIST_PORT=/dev/ttyUSB0
#
# Usage:
#   eval "$(bash scripts/detect_usb_host.sh)"
#   echo "$DETECTED_MIA_PORT"

set -euo pipefail

BY_ID_DIR="/dev/serial/by-id"

# ── Guard: /dev/serial/by-id/ must exist ──────────────────────────────────
if [ ! -d "$BY_ID_DIR" ]; then
    echo "[detect-usb] WARNING: $BY_ID_DIR does not exist — no USB serial devices connected?" >&2
    echo "export DETECTED_MIA_PORT="
    echo "export DETECTED_WRIST_PORT="
    exit 0
fi

# ── Resolve a by-id symlink to its actual /dev/ttyUSB* path ───────────────
# Args: $1 = serial substring to grep for in the by-id filenames
resolve_by_serial() {
    local serial_pattern="$1"
    local link

    link=$(find "$BY_ID_DIR" -maxdepth 1 -lname '*' 2>/dev/null | grep "$serial_pattern" | head -n 1)

    if [ -z "$link" ]; then
        return 1
    fi

    # Resolve the symlink to get e.g. ../../ttyUSB0 → /dev/ttyUSB0
    local target
    target=$(readlink -f "$link")

    if [ -e "$target" ]; then
        echo "$target"
        return 0
    else
        echo "[detect-usb] WARNING: symlink $link → $target but target does not exist" >&2
        return 1
    fi
}

# ── Detect MIA Hand (serial FTBY495J) ─────────────────────────────────────
mia_port=""
if mia_port=$(resolve_by_serial "FTBY495J"); then
    echo "[detect-usb] MIA Hand found: $mia_port  (by-id serial FTBY495J)" >&2
else
    echo "[detect-usb] WARNING: MIA Hand not found (serial FTBY495J)" >&2
fi

# ── Detect Wrist Dynamixel (serial FTAO4Z0Y) ──────────────────────────────
wrist_port=""
if wrist_port=$(resolve_by_serial "FTAO4Z0Y"); then
    echo "[detect-usb] Wrist Dynamixel found: $wrist_port  (by-id serial FTAO4Z0Y)" >&2
else
    echo "[detect-usb] WARNING: Wrist Dynamixel not found (serial FTAO4Z0Y)" >&2
fi

# ── Output export statements ──────────────────────────────────────────────
echo "export DETECTED_MIA_PORT=$mia_port"
echo "export DETECTED_WRIST_PORT=$wrist_port"
