#!/usr/bin/env bash
# tune_network_jetson.sh — Apply persistent UDP/kernel buffer tuning on the Jetson
#
# This script is meant to be scp'd to the Jetson and executed there via SSH.
# It applies the same four kernel parameters as tune_network_host.sh:
#   - rmem_max / wmem_max (socket buffer ceilings)
#   - ipfrag_high_thresh (IP fragment reassembly memory limit)
#   - ipfrag_time (fragment timeout)
#
# The Jetson publishes two PointCloud2 streams so wmem_max is critical for its
# send buffers.  rmem_max matters when Jetson-side tools subscribe to
# host-published topics like /fused_pointcloud.
#
# See: plans/2026-06-16-network-buffer-tuning-host-v2.md

set -euo pipefail

SYSCTL_CONF="/etc/sysctl.d/99-prosthesis-udp.conf"

declare -r RMEM_MAX=2147483647
declare -r WMEM_MAX=2147483647
declare -r IPFRAG_HIGH_THRESH=134217728
declare -r IPFRAG_TIME=3

if [[ $EUID -ne 0 ]]; then
    echo "ERROR: This script must be run as root (sudo)."
    exit 1
fi

echo "=== Prosthesis Jetson Network Buffer Tuning ==="
echo ""

echo "Current kernel buffer settings:"
echo "  net.core.rmem_max          = $(sysctl -n net.core.rmem_max)"
echo "  net.core.wmem_max          = $(sysctl -n net.core.wmem_max)"
echo "  net.ipv4.ipfrag_high_thresh = $(sysctl -n net.ipv4.ipfrag_high_thresh)"
echo "  net.ipv4.ipfrag_time       = $(sysctl -n net.ipv4.ipfrag_time)"
echo ""

cat > "$SYSCTL_CONF" <<EOF
# Prosthesis — UDP/IP fragment reassembly tuning for CycloneDDS
# Applied at boot by systemd-sysctl.
# See: scripts/tune_network_jetson.sh
#
# The Jetson publishes two RealSense D435i PointCloud2 streams (5-8 MB each,
# ~120 RTPS fragments per message) to the host over a direct Ethernet link.
# Large wmem_max ensures the send socket can buffer complete messages.
# Large rmem_max and ipfrag_high_thresh handle incoming traffic from the host
# (e.g., /fused_pointcloud subscriptions or service responses).
net.core.rmem_max = ${RMEM_MAX}
net.core.wmem_max = ${WMEM_MAX}
net.ipv4.ipfrag_high_thresh = ${IPFRAG_HIGH_THRESH}
net.ipv4.ipfrag_time = ${IPFRAG_TIME}
EOF

echo "Wrote persistent config to: $SYSCTL_CONF"
echo ""

sysctl -w "net.core.rmem_max=${RMEM_MAX}"
sysctl -w "net.core.wmem_max=${WMEM_MAX}"
sysctl -w "net.ipv4.ipfrag_high_thresh=${IPFRAG_HIGH_THRESH}"
sysctl -w "net.ipv4.ipfrag_time=${IPFRAG_TIME}"

echo ""
echo "=== Verification ==="
echo ""

echo "Active kernel buffer settings:"
printf "  %-30s %s\n" "net.core.rmem_max"           "$(sysctl -n net.core.rmem_max)"
printf "  %-30s %s\n" "net.core.wmem_max"           "$(sysctl -n net.core.wmem_max)"
printf "  %-30s %s\n" "net.ipv4.ipfrag_high_thresh" "$(sysctl -n net.ipv4.ipfrag_high_thresh)"
printf "  %-30s %s\n" "net.ipv4.ipfrag_time"        "$(sysctl -n net.ipv4.ipfrag_time)"

echo ""
echo "=== Done ==="
echo "Jetson network buffers are tuned and persistent across reboots."
