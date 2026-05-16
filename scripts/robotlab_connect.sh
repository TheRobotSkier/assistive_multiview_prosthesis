#!/bin/bash
# scripts/robotlab_connect.sh
# Ensure ethernet connection to Jetson (robotlab) is up.
# Fast-exits if SSH is already working.
# Uses NM profile ethernet-robotlab (MAC-matched, interface-agnostic).

set -e

JETSON_HOST="robotlab"
JETSON_MAC="30:de:4b:c6:23:1f"
HOST_ADDR="192.168.100.1/24"
HOST_IP="192.168.100.1"
JETSON_IP="${JETSON_IP:-192.168.100.2}"
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

# Wait for link carrier before pinging.
for _ in 1 2 3 4 5; do
    CARRIER=$(cat "/sys/class/net/$IFACE/carrier" 2>/dev/null || echo 0)
    if [ "$CARRIER" = "1" ]; then
        break
    fi
    sleep 1
done

CARRIER=$(cat "/sys/class/net/$IFACE/carrier" 2>/dev/null || echo 0)
if [ "$CARRIER" != "1" ]; then
    echo "ERROR: Ethernet adapter $IFACE has no carrier."
    echo "       Host IP is configured as $HOST_IP, but no physical link is detected."
    echo "       Check Jetson power, adapter, and cable before retrying."
    exit 1
fi

if ping -c 1 -W 2 "$JETSON_IP" &>/dev/null; then
    echo "Jetson reachable at $JETSON_IP"
    exit 0
fi

echo "ERROR: Jetson at $JETSON_IP not responding."
echo "       Host $IFACE has carrier and $HOST_IP/24; check Jetson IP and firewall."
exit 1
