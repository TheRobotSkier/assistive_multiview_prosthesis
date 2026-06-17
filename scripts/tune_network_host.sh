#!/usr/bin/env bash
# tune_network_host.sh — Apply persistent UDP/kernel buffer tuning for ROS 2 CycloneDDS
#
# This script configures the Linux kernel to handle high-throughput fragmented
# UDP traffic from two RealSense PointCloud2 streams at 15-30 Hz, each 5-8 MB,
# arriving as ~120 fragments.
#
# These settings are PERSISTENT across reboots (written to /etc/sysctl.d/).
# Run with: sudo ./scripts/tune_network_host.sh

set -euo pipefail

SYSCTL_CONF="/etc/sysctl.d/99-prosthesis-udp.conf"

# ── Values ────────────────────────────────────────────────────────────────────
# net.core.rmem_max: maximum socket receive buffer size (bytes).
#   2147483647 = 2 GB.  The kernel grants setsockopt(SO_RCVBUF) up to this ceiling.
#   Two PointCloud2 streams (~5-8 MB each) are received locally, and each incoming
#   UDP fragment must be buffered in the socket while IP reassembly runs.
#   Default ~208 KB is far too small — a single PointCloud2 needs ~8 MB of buffer.
declare -r RMEM_MAX=2147483647

# net.core.wmem_max: maximum socket send buffer size (bytes).
#   2147483647 = 2 GB.  Symmetrical ceiling for the send side.  The pipeline
#   publishes /fused_pointcloud (8-15 MB) which may be subscribed by RViz/tools.
declare -r WMEM_MAX=2147483647

# net.ipv4.ipfrag_high_thresh: maximum memory (bytes) the kernel uses to reassemble
#   IP fragments.  When total fragment memory exceeds this, fragments are dropped.
#   134217728 = 128 MB.  Two PointCloud2s in-flight (~16 MB) + overhead fit easily.
#   Default is typically 4 MB — not enough for 150+ fragments from two concurrent clouds.
declare -r IPFRAG_HIGH_THRESH=134217728

# net.ipv4.ipfrag_time: maximum seconds to hold an incomplete fragment queue.
#   Default 30s wastes memory holding stale fragments.  3s matches pointcloud
#   cadence (~33-66 ms) with a generous safety margin.
declare -r IPFRAG_TIME=3

# ── Pre-flight checks ─────────────────────────────────────────────────────────
if [[ $EUID -ne 0 ]]; then
    echo "ERROR: This script must be run as root (sudo)."
    echo "  sudo ./scripts/tune_network_host.sh"
    exit 1
fi

echo "=== Prosthesis Host Network Buffer Tuning ==="
echo ""

# ── Show current values ───────────────────────────────────────────────────────
echo "Current kernel buffer settings:"
echo "  net.core.rmem_max          = $(sysctl -n net.core.rmem_max)"
echo "  net.core.wmem_max          = $(sysctl -n net.core.wmem_max)"
echo "  net.ipv4.ipfrag_high_thresh = $(sysctl -n net.ipv4.ipfrag_high_thresh)"
echo "  net.ipv4.ipfrag_time       = $(sysctl -n net.ipv4.ipfrag_time)"
echo ""

# ── Write persistent config ───────────────────────────────────────────────────
cat > "$SYSCTL_CONF" <<EOF
# Prosthesis — UDP/IP fragment reassembly tuning for CycloneDDS
# Applied at boot by systemd-sysctl.
# See: scripts/tune_network_host.sh
#
# Two RealSense D435i PointCloud2 streams are published locally.  Each cloud
# is 5-8 MB, fragmented by CycloneDDS into ~120 RTPS messages of 64 KB each.
# Without these settings the default Linux kernel buffers (~208 KB rmem_max,
# ~4 MB ipfrag) drop fragments under load, causing ros2 topic hz/echo to crash.
net.core.rmem_max = ${RMEM_MAX}
net.core.wmem_max = ${WMEM_MAX}
net.ipv4.ipfrag_high_thresh = ${IPFRAG_HIGH_THRESH}
net.ipv4.ipfrag_time = ${IPFRAG_TIME}
EOF

echo "Wrote persistent config to: $SYSCTL_CONF"
echo ""

# ── Apply immediately ─────────────────────────────────────────────────────────
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
echo "Host network buffers are tuned and persistent across reboots."
