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

BY_ID_DIR="${DETECT_USB_BY_ID_DIR:-/dev/serial/by-id}"
DEV_DIR="${DETECT_USB_DEV_DIR:-/dev}"

list_by_id_links() {
    if [ -d "$BY_ID_DIR" ]; then
        find "$BY_ID_DIR" -maxdepth 1 -type l 2>/dev/null | sort
    fi
    return 0
}

# ── Resolve a by-id symlink to its actual /dev/ttyUSB* path ───────────────
# Args: $1 = serial substring to grep for in the by-id filenames
resolve_by_serial() {
    local serial_pattern="$1"
    local link

    link=$(list_by_id_links | grep "$serial_pattern" | head -n 1 || true)

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

detect_wrist_fallback() {
    local mia_target="$1"
    local candidates=()
    local target
    local link
    local dev

    while IFS= read -r link; do
        [ -n "$link" ] || continue
        target=$(readlink -f "$link" 2>/dev/null || true)
        [ -n "$target" ] && [ -e "$target" ] || continue
        [ "$target" != "$mia_target" ] || continue
        case "$target" in
            "$DEV_DIR"/ttyUSB*|"$DEV_DIR"/ttyACM*) ;;
            *) continue ;;
        esac
        candidates+=("$target")
    done < <(list_by_id_links)

    # If by-id is unavailable or incomplete, fall back to the tty devices
    # themselves. This handles hubs/adapters whose by-id name changed.
    while IFS= read -r dev; do
        [ -n "$dev" ] || continue
        [ "$dev" != "$mia_target" ] || continue
        candidates+=("$dev")
    done < <(find "$DEV_DIR" -maxdepth 1 \( -name 'ttyUSB*' -o -name 'ttyACM*' \) 2>/dev/null | sort)

    if [ "${#candidates[@]}" -eq 0 ]; then
        return 1
    fi

    mapfile -t candidates < <(printf '%s\n' "${candidates[@]}" | sort -u)

    if [ "${#candidates[@]}" -eq 1 ]; then
        echo "${candidates[0]}"
        return 0
    fi

    echo "[detect-usb] WARNING: multiple non-MIA serial candidates found for wrist:" >&2
    printf '[detect-usb]   %s\n' "${candidates[@]}" >&2
    echo "[detect-usb] Set WRIST_PORT=/dev/... to choose explicitly." >&2
    return 1
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
elif wrist_port=$(detect_wrist_fallback "$mia_port"); then
    echo "[detect-usb] Wrist Dynamixel inferred: $wrist_port  (non-MIA serial adapter)" >&2
else
    echo "[detect-usb] WARNING: Wrist Dynamixel not found (serial FTAO4Z0Y or fallback candidate)" >&2
fi

# ── Output export statements ──────────────────────────────────────────────
echo "export DETECTED_MIA_PORT=$mia_port"
echo "export DETECTED_WRIST_PORT=$wrist_port"
