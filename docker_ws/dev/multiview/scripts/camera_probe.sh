#!/usr/bin/env bash
set -euo pipefail

CAMERA_DEVICE="${CAMERA_DEVICE:-/dev/video4}"
STREAM_COUNT="${STREAM_COUNT:-1}"
OUTPUT_FILE="${OUTPUT_FILE:-/tmp/camera-frame.bin}"

echo "== USB devices =="
lsusb || true

echo
echo "== V4L devices =="
v4l2-ctl --list-devices || true

echo
echo "== Stable RealSense links =="
ls -l /dev/v4l/by-id /dev/v4l/by-path 2>/dev/null || true

echo
if [ ! -e "$CAMERA_DEVICE" ]; then
  echo "Camera device $CAMERA_DEVICE is not present inside the container." >&2
  exit 1
fi

echo "== Device details for $CAMERA_DEVICE =="
v4l2-ctl -d "$CAMERA_DEVICE" --all

echo
echo "== Supported formats for $CAMERA_DEVICE =="
v4l2-ctl -d "$CAMERA_DEVICE" --list-formats-ext

echo
echo "== Reading ${STREAM_COUNT} frame(s) from $CAMERA_DEVICE =="
rm -f "$OUTPUT_FILE"
v4l2-ctl -d "$CAMERA_DEVICE" --stream-mmap=3 --stream-count="$STREAM_COUNT" --stream-to="$OUTPUT_FILE"

echo
echo "== Probe result =="
ls -lh "$OUTPUT_FILE"
