#!/usr/bin/env bash
# host_launch_multiview.sh — Launch the multiview camera pipeline from the host.
#
# Detects available RealSense cameras by USB serial, starts the Docker/Podman
# container(s) with correct device passthrough. Works in both single-camera and
# dual-camera modes.
#
# Usage:
#   ./host_launch_multiview.sh                          # Auto-detect cameras
#   ./host_launch_multiview.sh --rviz                   # With RViz visualisation
#   ./host_launch_multiview.sh --mode single            # Force single camera
#   ./host_launch_multiview.sh --mode dual --cam2-offset 0.5
#   ./host_launch_multiview.sh --help                   # This help
#
# Requirements:
#   - podman (or docker)
#   - multiview-humble-realsense:latest image built
#   - multiview-rviz2:latest image built (for --rviz)
#   - At least one Intel RealSense D435 camera plugged in
#
# Device passthrough:
#   Only --device /dev/bus/usb:/dev/bus/usb is needed. Librealsense enumerates
#   cameras via USB descriptors. Explicit /dev/videoN passthrough is NOT used
#   because video device numbers change on every reconnect/reboot.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
CONTAINER_TOOL="${CONTAINER_TOOL:-podman}"
IMAGE_CAM="${IMAGE_CAM:-localhost/multiview-humble-realsense:latest}"
IMAGE_RVIZ="${IMAGE_RVIZ:-localhost/multiview-rviz2:latest}"
MODE="auto"
WITH_RVIZ=false
CAM2_OFFSET_X="0.5"
CAM1_SERIAL=""
CAM2_SERIAL=""

# ── Parse arguments ────────────────────────────────────────────────────────

while [ $# -gt 0 ]; do
  case "$1" in
    --rviz)     WITH_RVIZ=true; shift ;;
    --mode)     MODE="$2"; shift 2 ;;
    --cam2-offset) CAM2_OFFSET_X="$2"; shift 2 ;;
    --help|-h)  sed -n '/^#$/q; /^#/p' "$0" | sed 's/^# \?//'; exit 0 ;;
    *)          echo "Unknown option: $1"; exit 1 ;;
  esac
done

# ── Detect cameras ─────────────────────────────────────────────────────────

detect_cameras() {
  local serials=()
  # Use librealsense's rs-enumerate-devices if available
  if command -v rs-enumerate-devices &>/dev/null; then
    while IFS= read -r line; do
      if [[ "$line" =~ Serial\ Number:\ ([0-9]+) ]]; then
        serials+=("${BASH_REMATCH[1]}")
      fi
    done < <(rs-enumerate-devices -s 2>/dev/null || true)
  fi

  # Fallback: check v4l symlinks for Intel RealSense cameras
  if [ ${#serials[@]} -eq 0 ]; then
    for link in /dev/v4l/by-id/usb-Intel_R__RealSense_TM__Depth_Camera_435_*/video-index0; do
      serial="$(basename "$link" | sed 's/.*Camera_435_//;s/-video-index0//')"
      serials+=("$serial")
    done
  fi

  # Remove duplicates (same camera may appear via multiple names)
  local unique=()
  while IFS= read -r -d '' s; do unique+=("$s"); done < <(printf "%s\0" "${serials[@]}" | sort -uz)

  echo "${unique[@]}"
}

if [ "$MODE" = "auto" ]; then
  SERIALS=($(detect_cameras))
  if [ ${#SERIALS[@]} -eq 0 ]; then
    echo "ERROR: No RealSense cameras detected. Check USB connection." >&2
    echo "  Tried: rs-enumerate-devices and /dev/v4l/by-id/ RealSense symlinks" >&2
    exit 1
  elif [ ${#SERIALS[@]} -eq 1 ]; then
    MODE="single"
    CAM1_SERIAL="${SERIALS[0]}"
  else
    MODE="dual"
    CAM1_SERIAL="${SERIALS[0]}"
    CAM2_SERIAL="${SERIALS[1]}"
  fi
elif [ "$MODE" = "single" ] && [ -z "$CAM1_SERIAL" ]; then
  CAM1_SERIAL="$(detect_cameras | head -1)"
  [ -z "$CAM1_SERIAL" ] && { echo "ERROR: Can't auto-detect camera serial, specify with --cam1-serial" >&2; exit 1; }
elif [ "$MODE" = "dual" ]; then
  if [ -z "$CAM1_SERIAL" ]; then
    SERIALS=($(detect_cameras))
    CAM1_SERIAL="${SERIALS[0]:-}"
    CAM2_SERIAL="${SERIALS[1]:-}"
  fi
  if [ -z "$CAM1_SERIAL" ] || [ -z "$CAM2_SERIAL" ]; then
    echo "ERROR: Dual mode requires two cameras. Found: CAM1=${CAM1_SERIAL:-none} CAM2=${CAM2_SERIAL:-none}" >&2
    exit 1
  fi
fi

# ── Build podman args ──────────────────────────────────────────────────────

PODMAN_ARGS=(
  --rm
  --privileged
  --network host
  --ipc host
  --device /dev/bus/usb:/dev/bus/usb
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp
)

if [ "$MODE" = "single" ]; then
  echo "=== Single-camera mode: ${CAM1_SERIAL} ==="
  PODMAN_ARGS+=(
    -e "CAM1_SERIAL=${CAM1_SERIAL}"
  )
  $CONTAINER_TOOL run "${PODMAN_ARGS[@]}" "$IMAGE_CAM"
else
  echo "=== Dual-camera mode: ${CAM1_SERIAL} + ${CAM2_SERIAL} ==="
  PODMAN_ARGS+=(
    -e "CAM1_SERIAL=${CAM1_SERIAL}"
    -e "CAM2_SERIAL=${CAM2_SERIAL}"
    -e "CAM2_OFFSET_X=${CAM2_OFFSET_X}"
  )
  # Start cameras in background
  $CONTAINER_TOOL run -d --name multiview_cameras "${PODMAN_ARGS[@]}" "$IMAGE_CAM" >/dev/null
  CAM_PID=$!
  echo "Camera container started PID=${CAM_PID}"

  if [ "$WITH_RVIZ" = true ]; then
    echo "Starting RViz2..."
    sleep 10  # Give cameras time to start publishing
    $CONTAINER_TOOL run -d --name multiview_rviz2 --rm \
      --network host --ipc host \
      -e DISPLAY="${DISPLAY:-:0}" -e QT_X11_NO_MITSHM=1 -e HOME=/tmp \
      -v /tmp/.X11-unix:/tmp/.X11-unix \
      -e XAUTHORITY=/tmp/.Xauthority \
      -v "${XAUTHORITY}:/tmp/.Xauthority:ro,z" \
      --userns=keep-id \
      "$IMAGE_RVIZ" >/dev/null
    echo "RViz2 started. Press Ctrl+C to stop all."
    # Wait for camera container to finish
    wait "$CAM_PID" 2>/dev/null || true
    # Clean up rviz
    $CONTAINER_TOOL stop -t 2 multiview_rviz2 2>/dev/null || true
  else
    echo "Camera container running. Press Ctrl+C to stop."
    wait "$CAM_PID" 2>/dev/null || true
  fi
fi
