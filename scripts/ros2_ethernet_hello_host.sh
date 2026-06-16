#!/usr/bin/env bash
# ros2_ethernet_hello_host.sh — Launch ROS 2 tools in a temp container on the
# host network for Jetson Ethernet communication testing and RViz monitoring.
#
# NETWORK NOTE: When monitoring topics over the Jetson Ethernet link,
# always append --qos-reliability best_effort to ros2 topic echo/hz/bw
# to avoid saturating the link with RELIABLE ACK/NACK traffic.
# Pipeline nodes MUST use RELIABLE; monitoring tools should use BEST_EFFORT.
#
# See: plans/2026-06-16-network-buffer-tuning-host-v2.md
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

IMAGE="${ROS2_JAZZY_IMAGE:-localhost/rviz-robotlab:latest}"
DDS_CONFIG="${DDS_CONFIG:-$REPO_ROOT/config/cyclonedds_peer.xml}"
RVIZ_CONFIG="${RVIZ_CONFIG:-$REPO_ROOT/rviz/phase2_dual_openvins_head_preview.rviz}"
ROS_DOMAIN_ID_VALUE="${ROS_DOMAIN_ID:-0}"
CONTAINER_PREFIX="${ROS2_HOST_CONTAINER_PREFIX:-ros2-jazzy-host}"
HOST_XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"
read -r -a DOCKER_CMD <<< "${DOCKER_CMD:-podman}"

usage() {
  cat <<'EOF'
Usage: scripts/ros2_ethernet_hello_host.sh <command>

Commands:
  shell          Open an interactive ROS 2 Jazzy shell on the host network
  listen-jetson  Echo /jetson_hello from the Jetson
  pub-host       Publish "hello from host" on /host_hello
  topic-list     List visible ROS 2 topics
  node-list      List visible ROS 2 nodes
  rviz           Start RViz2 in the container

Environment overrides:
  ROS2_JAZZY_IMAGE              Docker image, default localhost/rviz-robotlab:latest
  DDS_CONFIG                    CycloneDDS XML, default config/cyclonedds_peer.xml
  RVIZ_CONFIG                   RViz config, default rviz/phase2_dual_openvins_head_preview.rviz
  ROS_DOMAIN_ID                 ROS domain, default 0
  ROS2_HOST_CONTAINER_PREFIX    Container name prefix, default ros2-jazzy-host
  DOCKER_CMD                    Docker command, for example "sudo docker"
  XAUTHORITY                    Host Xauthority file, default $HOME/.Xauthority
EOF
}

command="${1:-}"
case "$command" in
  -h|--help|help|"")
    usage
    exit 0
    ;;
esac

if [[ ! -f "$DDS_CONFIG" ]]; then
  echo "Missing CycloneDDS config: $DDS_CONFIG" >&2
  exit 1
fi

if ! command -v "${DOCKER_CMD[0]}" >/dev/null 2>&1; then
  echo "${DOCKER_CMD[0]} is required on the host." >&2
  exit 1
fi

if ! "${DOCKER_CMD[@]}" info >/dev/null 2>&1; then
  echo "Cannot access Docker. If your user is not in the docker group, retry with:" >&2
  echo "  DOCKER_CMD='sudo docker' $0 ${1:-shell}" >&2
  exit 1
fi

tty_args=(-i)
if [[ -t 0 && -t 1 ]]; then
  tty_args=(-it)
fi

# ── WSL2 GPU acceleration ──────────────────────────────────────────────────
# On WSL2, OpenGL is accelerated via Mesa's d3d12 Gallium driver, which talks
# to the Windows GPU through /dev/dxg and the libraries under /usr/lib/wsl.
# Detect that environment and expose the right flags to the container.
# On native Linux these stay empty, so the script stays portable.
gpu_device_args=()
gpu_env_args=()
if [[ -e /dev/dxg && -f /usr/lib/wsl/lib/libdxcore.so ]]; then
  gpu_device_args=(--device /dev/dri --device /dev/dxg -v /usr/lib/wsl:/usr/lib/wsl:ro)
  gpu_env_args+=(-e GALLIUM_DRIVER=d3d12 -e LD_LIBRARY_PATH=/usr/lib/wsl/lib -e MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA)
fi

container_cmd() {
  local name="$1"
  shift
  local volume_args=(-v "$DDS_CONFIG:/tmp/cyclonedds_peer.xml:ro")
  local env_args=(-e DISPLAY="${DISPLAY:-:0}")

  if [[ -f "$RVIZ_CONFIG" ]]; then
    volume_args+=(-v "$RVIZ_CONFIG:/rviz_config.rviz:rw")
  fi

  if [[ -f "$HOST_XAUTHORITY" ]]; then
    volume_args+=(-v "$HOST_XAUTHORITY:/tmp/.Xauthority:ro")
    env_args+=(-e XAUTHORITY=/tmp/.Xauthority)
  fi

  local nvidia_env_args=()
  if [[ "${DOCKER_CMD[0]}" == podman ]] && [[ -x /usr/bin/nvidia-container-runtime ]]; then
    runtime_args=(--runtime=/usr/bin/nvidia-container-runtime)
    nvidia_env_args=(-e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all)
  fi

  "${DOCKER_CMD[@]}" run --rm "${tty_args[@]}" \
    --name "$name" \
    --network host \
    --ipc host \
    "${gpu_device_args[@]}" \
    "${runtime_args[@]}" \
    "${env_args[@]}" \
    "${gpu_env_args[@]}" \
    -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    -e CYCLONEDDS_URI=/tmp/cyclonedds_peer.xml \
    -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID_VALUE" \
    "${nvidia_env_args[@]}" \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    "${volume_args[@]}" \
    "$IMAGE" \
    bash -lc "$*"
}

ros_prefix='
set -e
if ! ros2 pkg list 2>/dev/null | grep -qx rmw_cyclonedds_cpp; then
  echo "Installing ros-jazzy-rmw-cyclonedds-cpp inside temporary container..."
  apt-get update
  apt-get install -y ros-jazzy-rmw-cyclonedds-cpp
fi
source /opt/ros/jazzy/setup.bash
'

case "$command" in
  shell)
    container_cmd "$CONTAINER_PREFIX-shell" "$ros_prefix exec bash"
    ;;
  listen-jetson)
    container_cmd "$CONTAINER_PREFIX-listen-jetson" "$ros_prefix ros2 topic echo --qos-reliability best_effort /jetson_hello std_msgs/msg/String"
    ;;
  pub-host)
    container_cmd "$CONTAINER_PREFIX-pub-host" "$ros_prefix ros2 topic pub /host_hello std_msgs/msg/String \"{data: 'hello from host'}\" -r 1"
    ;;
  topic-list)
    container_cmd "$CONTAINER_PREFIX-topic-list" "$ros_prefix ros2 topic list"
    ;;
  node-list)
    container_cmd "$CONTAINER_PREFIX-node-list" "$ros_prefix ros2 node list"
    ;;
  rviz)
    container_cmd "$CONTAINER_PREFIX-rviz" "$ros_prefix if [[ -f /rviz_config.rviz ]]; then rviz2 -d /rviz_config.rviz; else rviz2; fi"
    ;;
  *)
    echo "Unknown command: $command" >&2
    usage >&2
    exit 1
    ;;
esac
