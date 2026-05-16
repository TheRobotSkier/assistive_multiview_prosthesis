#!/bin/bash
# test_ros_network_contracts.sh — static checks for host/Jetson ROS network config.

set -euo pipefail

WS_ROOT="${PROSTHESIS_WS:-/prosthesis_ws}"
if [ ! -d "$WS_ROOT/src" ]; then
    WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
fi

python3 - "$WS_ROOT" <<'PY'
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

root = Path(sys.argv[1])
host_cfg = root / "config/cyclonedds_peer.xml"
jetson_cfg = root / "jetson/config/cyclonedds_robotlab.xml"
connect = (root / "scripts/robotlab_connect.sh").read_text()
make = (root / "Makefile").read_text()

def addresses(path: Path):
    tree = ET.parse(path)
    net = tree.find(".//NetworkInterfaceAddress")
    peers = [p.attrib["address"] for p in tree.findall(".//Peer")]
    return net.text.strip(), peers

host_net, host_peers = addresses(host_cfg)
jetson_net, jetson_peers = addresses(jetson_cfg)

assert host_net == "192.168.100.1"
assert jetson_net == "192.168.100.2"
assert host_peers == ["192.168.100.1", "192.168.100.2"]
assert jetson_peers == ["192.168.100.1", "192.168.100.2"]

assert 'HOST_ADDR="192.168.100.1/24"' in connect
assert 'JETSON_IP="${JETSON_IP:-192.168.100.2}"' in connect
assert "/sys/class/net/$IFACE/carrier" in connect
assert "check-ros-network" in make
PY

echo "ROS network contracts OK"
