#!/usr/bin/env bash
# ─────────────────────────────────────────────────────────────
# robotlab-wifi Hotspot Setup — Linux / NetworkManager
# ─────────────────────────────────────────────────────────────
# Creates a WiFi access point on <interface> with SSID robotlab-wifi,
# static IP 10.42.0.1/24, internet sharing via NAT, and UFW forwarding.
#
# Robotlab expects:
#   SSID:      robotlab-wifi
#   Password:  labrobot123
#   Gateway:   10.42.0.1
#   Static IP: 10.42.0.2/24
#
# Usage:
#   sudo ./setup_robotlab_wifi_linux.sh <wifi_interface>
#
# Example:
#   sudo ./setup_robotlab_wifi_linux.sh wlan1
#
# Dependencies: nmcli, hostapd (installed via NetworkManager shared mode)
# ─────────────────────────────────────────────────────────────
set -euo pipefail

SSID="robotlab-wifi"
PASSWORD="labrobot123"
CONNECTION_NAME="robotlab-hotspot"

if [ $# -ne 1 ]; then
    echo "Usage: $0 <wifi_interface>"
    echo "Example: $0 wlan1"
    echo ""
    echo "Available interfaces:"
    iw dev 2>/dev/null | grep Interface | awk '{print $2}' || echo "(none)"
    exit 1
fi

IFACE="$1"

# ─── Sanity checks ───
if ! command -v nmcli &>/dev/null; then
    echo "ERROR: nmcli not found. Install NetworkManager."
    exit 1
fi

if ! iw dev "$IFACE" info &>/dev/null; then
    echo "ERROR: Interface '$IFACE' not found."
    exit 1
fi

if ! iw phy "$(iw dev "$IFACE" info | awk '/wiphy/{print $2}')" info 2>/dev/null | grep -q "\\* AP"; then
    echo "WARNING: Interface '$IFACE' may not support AP mode."
fi

# ─── Enable IP forwarding (persistent) ───
echo "net.ipv4.ip_forward=1" > /etc/sysctl.d/99-robotlab-forward.conf
sysctl -w net.ipv4.ip_forward=1
echo "[OK] IP forwarding enabled"

# ─── Delete any existing profile ───
nmcli connection delete "$CONNECTION_NAME" 2>/dev/null || true

# ─── Create hotspot ───
nmcli connection add type wifi ifname "$IFACE" con-name "$CONNECTION_NAME" \
    autoconnect yes ssid "$SSID" \
    802-11-wireless.mode ap 802-11-wireless.band bg \
    wifi-sec.key-mgmt wpa-psk wifi-sec.psk "$PASSWORD" \
    ipv4.method shared

echo "[OK] Hotspot profile '$CONNECTION_NAME' created"

# ─── Activate ───
nmcli connection up "$CONNECTION_NAME"
echo "[OK] Hotspot '$SSID' is now active on $IFACE"

# ─── UFW: allow forwarding from hotspot → internet ───
if command -v ufw &>/dev/null; then
    # Find the internet interface (has a default gateway)
    WAN_IFACE=$(ip route | awk '/^default/{print $5; exit}')
    if [ -n "$WAN_IFACE" ]; then
        ufw route allow in on "$IFACE" out on "$WAN_IFACE" 2>/dev/null || true
        echo "[OK] UFW forwarding allowed from $IFACE → $WAN_IFACE"
    fi
fi

# ─── Verify ───
echo ""
echo "=== Verification ==="
ip addr show "$IFACE" | grep -oP 'inet \K[\d./]+' | head -1
iw dev "$IFACE" info | grep -E "ssid|type|channel"
echo ""
echo "SSH to robotlab: ssh robotlab@10.42.0.2"
echo "Stop hotspot:    nmcli connection down $CONNECTION_NAME"
echo "Remove:          nmcli connection delete $CONNECTION_NAME"
