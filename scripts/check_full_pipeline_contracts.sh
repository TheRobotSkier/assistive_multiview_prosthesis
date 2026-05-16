#!/usr/bin/env bash
# Static orchestration checks for final Jetson + host digital-twin pipeline.

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

python3 - "$ROOT_DIR" <<'PY'
from pathlib import Path
import sys
import yaml

root = Path(sys.argv[1])
make = (root / "Makefile").read_text()
start = (root / "scripts/start_full_digital_twin_pipeline.sh").read_text()
stop = (root / "scripts/stop_full_digital_twin_pipeline.sh").read_text()
runtime = (root / "scripts/check_full_pipeline_runtime.sh").read_text()
compose = yaml.safe_load((root / "docker/docker-compose.yml").read_text())

assert "up-full-digital-twin:" in make
assert "stop-full-digital-twin:" in make
assert "check-full-pipeline:" in make
assert "scripts/start_full_digital_twin_pipeline.sh" in make
assert "scripts/stop_full_digital_twin_pipeline.sh" in make
assert "scripts/check_full_pipeline_runtime.sh" in make

assert "make jetson-sync" in start
assert "make cameras-final && make openvins-final" in start
assert "--profile digital_twin" in start
assert "segmentation digital_twin" in start
assert "digital_twin_rviz" in start
assert "xhost +local:" in start
assert "SKIP_JETSON" in start

assert "make openvins-stop && make cameras-stop" in stop
assert "STOP_JETSON" in stop

digital = compose["services"]["digital_twin"]
rviz = compose["services"]["digital_twin_rviz"]
assert "segmentation" in digital["depends_on"]
assert not digital.get("devices")
assert any(v == "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" for v in digital["environment"])
assert "DT_CAMERA" in " ".join(digital["command"])
assert "DT_MOCK_EMG" in " ".join(digital["command"])
assert "rviz2 -d /prosthesis_ws/rviz/digital_twin.rviz" in " ".join(rviz["command"])
assert "digital_twin" in rviz["depends_on"]

for topic in (
    "/head/d435i_head/depth/color/points",
    "/arm/d435i_arm/depth/color/points",
    "/ov_msckf_head/odomimu",
    "/ov_msckf_arm/odomimu",
    "/fused_pointcloud",
    "/segmentation/object_cloud",
    "/joint_states",
    "/thumb_pos_ff_controller/commands",
    "/index_pos_ff_controller/commands",
    "/mrl_pos_ff_controller/commands",
):
    assert topic in runtime

print("Full pipeline orchestration contracts OK")
PY
