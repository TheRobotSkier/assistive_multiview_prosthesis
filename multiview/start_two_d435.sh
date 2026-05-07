#!/bin/bash
set -euo pipefail

CAM1_SERIAL="829212072207"
CAM2_SERIAL="827112072033"
RVIZ_CONFIG="/home/robotlab/Documents/multiview_prosthesis/jetson_folder/two_d435_test.rviz"

TMPDIR="/tmp/two_d435_launch"
mkdir -p "$TMPDIR"

cat > "$TMPDIR/cam1.sh" <<EOF
#!/bin/bash
source /opt/ros/humble/setup.bash
ros2 launch realsense2_camera rs_launch.py \
  camera_namespace:=cam1 \
  camera_name:=d435_1 \
  serial_no:=${CAM1_SERIAL} \
  enable_sync:=true \
  align_depth.enable:=true \
  enable_color:=true \
  enable_infra1:=false \
  enable_infra2:=false \
  depth_module.depth_profile:=640x480x6 \
  rgb_camera.color_profile:=640x480x6 \
  pointcloud.enable:=true \
  pointcloud.stream_filter:=2
exec bash
EOF

cat > "$TMPDIR/cam2.sh" <<EOF
#!/bin/bash
source /opt/ros/humble/setup.bash
ros2 launch realsense2_camera rs_launch.py \
  camera_namespace:=cam2 \
  camera_name:=d435_2 \
  serial_no:=${CAM2_SERIAL} \
  enable_sync:=true \
  align_depth.enable:=true \
  enable_color:=true \
  enable_infra1:=false \
  enable_infra2:=false \
  depth_module.depth_profile:=640x480x6 \
  rgb_camera.color_profile:=640x480x6 \
  pointcloud.enable:=true \
  pointcloud.stream_filter:=2
exec bash
EOF

cat > "$TMPDIR/params_rviz.sh" <<EOF
#!/bin/bash
source /opt/ros/humble/setup.bash
sleep 4
rviz2 -d "$RVIZ_CONFIG"
exec bash
EOF

chmod +x "$TMPDIR"/cam1.sh "$TMPDIR"/cam2.sh "$TMPDIR"/params_rviz.sh

open_new_term() {
    local title="$1"
    local script="$2"

    if command -v terminator >/dev/null 2>&1; then
        terminator --new-tab -T "$title" -x bash "$script" &
    elif command -v gnome-terminal >/dev/null 2>&1; then
        gnome-terminal --title="$title" -- bash "$script" &
    else
        x-terminal-emulator -e bash "$script" &
    fi
}

echo "Launching two D435 cameras..."

# Detect whether this script was started from inside Terminator
if [[ "${TERMINATOR_UUID:-}" != "" ]]; then
    echo "Running inside Terminator: reusing current tab and opening 2 extra tabs."

    open_new_term "D435 Cam1" "$TMPDIR/cam1.sh"
    sleep 1
    open_new_term "D435 Cam2" "$TMPDIR/cam2.sh"
    sleep 1

    # Reuse current terminal/tab for RViz
    exec bash "$TMPDIR/params_rviz.sh"

else
    echo "Not launched from a Terminator tab: opening 3 terminals/tabs."

    open_new_term "D435 Cam1" "$TMPDIR/cam1.sh"
    sleep 1
    open_new_term "D435 Cam2" "$TMPDIR/cam2.sh"
    sleep 1
    open_new_term "RViz" "$TMPDIR/params_rviz.sh"
fi
