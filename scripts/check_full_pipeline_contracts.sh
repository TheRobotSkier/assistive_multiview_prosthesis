#!/usr/bin/env bash
# Static orchestration checks for final x86 full digital-twin pipeline.

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

assert "--profile digital_twin" in start
assert "services=(segmentation digital_twin digital_twin_rviz)" in start
assert "x86_cameras x86_openvins" in start
assert "digital_twin_rviz" in start
assert "xhost +local:" in start
assert "DT_PERCEPTION_BACKEND" in start
assert "make jetson-sync" in start
assert "make cameras-final && make openvins-final" in start

assert "make openvins-stop && make cameras-stop" in stop
assert "STOP_JETSON" in stop

x86_cameras = compose["services"]["x86_cameras"]
x86_openvins = compose["services"]["x86_openvins"]
digital = compose["services"]["digital_twin"]
rviz = compose["services"]["digital_twin_rviz"]
assert "/dev:/dev" in x86_cameras["volumes"]
assert "/run/udev:/run/udev:ro" in x86_cameras["volumes"]
assert "../src/sensor_fusion_bringup/config:/prosthesis_ws/install/sensor_fusion_bringup/share/sensor_fusion_bringup/config:ro" in x86_cameras["volumes"]
assert "../src/sensor_fusion_bringup/launch:/prosthesis_ws/install/sensor_fusion_bringup/share/sensor_fusion_bringup/launch:ro" in x86_cameras["volumes"]
assert "../src/sensor_fusion_bringup/scripts:/prosthesis_ws/install/sensor_fusion_bringup/lib/sensor_fusion_bringup:ro" in x86_openvins["volumes"]
assert "dual_d435i.launch.py" in " ".join(x86_cameras["command"])
assert "enable_pointcloud_neon_fix:=${DT_ENABLE_POINTCLOUD_NEON_FIX:-false}" in " ".join(x86_cameras["command"])
assert "dual_openvins_phase2.launch.py" in " ".join(x86_openvins["command"])
assert "start_camera:=false" in " ".join(x86_openvins["command"])
assert "use_marker_odometry_fallback" in " ".join(x86_openvins["command"])
assert "marker_tf_max_age_s:=${DT_TF_CACHE_MAX_AGE_S:-2.0}" in " ".join(x86_openvins["command"])
assert "x86_cameras" in x86_openvins["depends_on"]
assert "segmentation" in digital["depends_on"]
assert not digital.get("devices")
assert any(v == "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" for v in digital["environment"])
assert "DT_CAMERA" in " ".join(digital["command"])
assert "DT_MOCK_EMG" in " ".join(digital["command"])
assert "tf_cache_max_age_s:=${DT_TF_CACHE_MAX_AGE_S:-2.0}" in " ".join(digital["command"])
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
