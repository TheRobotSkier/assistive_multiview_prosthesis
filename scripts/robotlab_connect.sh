#!/usr/bin/env bash
# ─────────────────────────────────────────────────────────────
# robotlab_connect.sh — Verify WSL2 can reach the Jetson
# ─────────────────────────────────────────────────────────────
# Checks that mirrored networking is active and the Jetson
# (10.42.0.2) is reachable. Exits with an error if not.
#
# This script is called by 'make robotlab-connect' and by
# other Makefile targets that need Jetson connectivity.
# ─────────────────────────────────────────────────────────────
set -euo pipefail

JETSON_IP="10.42.0.2"
HOST_IP="10.42.0.1"

echo "=== Robotlab Connectivity Check ==="

# ─── Check mirrored networking ───
if ! ip addr show 2>/dev/null | grep -q "$HOST_IP"; then
    echo "ERROR: Host IP $HOST_IP not found on any WSL2 interface." >&2
    echo "" >&2
    echo "This usually means one of:" >&2
    echo "  1. WSL2 mirrored networking is not enabled" >&2
    echo "     Fix: Create C:\\Users\\<you>\\.wslconfig with:" >&2
    echo "       [wsl2]" >&2
    echo "       networkingMode=mirrored" >&2
    echo "     Then run: wsl --shutdown  (from PowerShell)" >&2
    echo "" >&2
    echo "  2. The Windows Ethernet adapter does not have IP $HOST_IP" >&2
    echo "     Fix: Run scripts/setup_jetson_ethernet.ps1 from an elevated PowerShell" >&2
    exit 1
fi
echo "[OK] Host IP $HOST_IP is present on a WSL2 interface (mirrored networking active)"

# ─── Ping the Jetson ───
echo -n "Pinging Jetson at $JETSON_IP... "
if ping -c 1 -W 2 "$JETSON_IP" >/dev/null 2>&1; then
    echo "[OK]"
else
    echo "[FAIL]" >&2
    echo "ERROR: Jetson at $JETSON_IP did not respond to ping." >&2
    echo "" >&2
    echo "Make sure:" >&2
    echo "  - The Jetson is powered on" >&2
    echo "  - The Ethernet cable is connected" >&2
    echo "  - The Jetson has IP $JETSON_IP configured:" >&2
    echo "    sudo ip addr add $JETSON_IP/24 dev eth0 && sudo ip link set eth0 up" >&2
    exit 1
fi

# ─── Check SSH ───
echo -n "Checking SSH connectivity... "
if ssh -o ConnectTimeout=3 -o BatchMode=yes robotlab@"$JETSON_IP" true 2>/dev/null; then
    echo "[OK] (key-based auth working)"
elif ssh -o ConnectTimeout=3 -o BatchMode=yes -o PasswordAuthentication=yes robotlab@"$JETSON_IP" true 2>/dev/null; then
    echo "[OK] (password auth — consider setting up key-based auth)"
else
    echo "[WARN]" >&2
    echo "  SSH to robotlab@$JETSON_IP failed. You may need to:" >&2
    echo "  - Set up SSH keys: ssh-copy-id robotlab@$JETSON_IP" >&2
    echo "  - Or add to ~/.ssh/config: Host robotlab / HostName $JETSON_IP / User robotlab" >&2
fi

echo ""
echo "=== Connected ==="
echo "SSH:  ssh robotlab@$JETSON_IP"
echo "Make: make jetson-cameras   (sync + start cameras + local RViz)"
