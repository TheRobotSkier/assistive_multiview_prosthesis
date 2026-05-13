#!/bin/bash
# scripts/robotlab_connect.sh
# Ensure ethernet connection to Jetson (robotlab) is up.
# Fast-exits if SSH is already working.
# Uses NM profile ethernet-robotlab (MAC-matched, interface-agnostic).

set -e

JETSON_HOST="robotlab"
JETSON_MAC="30:de:4b:c6:23:1f"
HOST_ADDR="192.168.100.1/24"
NM_PROFILE="ethernet-robotlab"

# Fast path: already connected
if ssh -o ConnectTimeout=2 -o BatchMode=yes "$JETSON_HOST" true 2>/dev/null; then
    exit 0
fi

echo "Jetson not reachable. Setting up ethernet link..."

# Find the USB ethernet adapter by MAC
IFACE=$(ip -o link | grep -i "$JETSON_MAC" | awk -F': ' '{print $2}' | awk '{print $1}')
if [ -z "$IFACE" ]; then
    echo "ERROR: USB ethernet adapter (MAC $JETSON_MAC) not found."
    echo "       Plug the cable into any USB port and try again."
    exit 1
fi

echo "Found interface: $IFACE"

# Try activating the NM profile
if nmcli connection show "$NM_PROFILE" &>/dev/null; then
    nmcli connection up "$NM_PROFILE" 2>/dev/null || {
        # Fallback: direct IP assignment
        ip link set "$IFACE" up
        ip addr add "$HOST_ADDR" dev "$IFACE" 2>/dev/null || true
    }
else
    # Create profile on the fly
    nmcli connection add type ethernet con-name "$NM_PROFILE" \
        ifname "" \
        802-3-ethernet.mac-address "$JETSON_MAC" \
        ipv4.method manual \
        ipv4.addresses "$HOST_ADDR"
    nmcli connection up "$NM_PROFILE"
fi

# Wait for link and try ping
sleep 2
if ping -c 1 -W 2 192.168.100.2 &>/dev/null; then
    echo "Jetson reachable at 192.168.100.2"
    exit 0
fi

echo "ERROR: Jetson at 192.168.100.2 not responding."
echo "       Check that the cable is connected and Jetson is powered on."
exit 1
