#!/usr/bin/env bash
# scripts/run_ransac_fusion.sh
#
# Drop-in RANSAC point cloud fusion container for Jetson.
#
# This runs the ransac_pointcloud_fusion_node in a Docker container
# with host networking.  It subscribes to the head and arm marker_map
# point clouds (published by the existing pointcloud_to_frame_node
# instances) and publishes a fused + deduplicated cloud.
#
# The container uses the existing prosthesis:latest image — no build
# step required, just mount the overlay read-only and run.
#
# Usage (on Jetson):
#   ./scripts/run_ransac_fusion.sh
#
# Customise topics via environment variables:
#   HEAD_TOPIC=/head/d435i_head/points_marker_map
#   ARM_TOPIC=/arm/d435i_arm/points_marker_map
#   OUTPUT_TOPIC=/pointcloud_fused_ransac
#   MIN_CORRESPONDENCES=8
#
#   HEAD_TOPIC=/my/head/cloud ARM_TOPIC=/my/arm/cloud ./scripts/run_ransac_fusion.sh
#
# Stop:
#   docker rm -f ransac_fusion

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# ── Defaults ─────────────────────────────────────────────────────────────────
HEAD_TOPIC="${HEAD_TOPIC:-/head/d435i_head/points_marker_map}"
ARM_TOPIC="${ARM_TOPIC:-/arm/d435i_arm/points_marker_map}"
OUTPUT_TOPIC="${OUTPUT_TOPIC:-/pointcloud_fused_ransac}"
MIN_CORRESPONDENCES="${MIN_CORRESPONDENCES:-8}"
MAX_RATE_HZ="${MAX_RATE_HZ:-10.0}"
DOWNSAMPLE_LEAF_M="${DOWNSAMPLE_LEAF_M:-0.02}"
FUSE_VOXEL_LEAF_M="${FUSE_VOXEL_LEAF_M:-0.01}"
NORMAL_RADIUS_M="${NORMAL_RADIUS_M:-0.05}"
FEATURE_RADIUS_M="${FEATURE_RADIUS_M:-0.10}"
MAX_CLOUD_AGE_S="${MAX_CLOUD_AGE_S:-1.0}"
RANSAC_MAX_ITERATIONS="${RANSAC_MAX_ITERATIONS:-2000}"
RANSAC_MAX_CORRESPONDENCE_DIST_M="${RANSAC_MAX_CORRESPONDENCE_DIST_M:-0.05}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

# ── Auto-detect overlay path ─────────────────────────────────────────────────
OVERLAY_DIR="${OVERLAY_DIR:-${REPO_ROOT}/docker_ws/install_overlay}"
if [ ! -f "$OVERLAY_DIR/setup.bash" ]; then
    # Fall back to Jetson default path
    OVERLAY_DIR="/home/robotlab/openvins_overlay/install_overlay"
fi
if [ ! -f "$OVERLAY_DIR/setup.bash" ]; then
    echo "ERROR: Overlay not found at $OVERLAY_DIR"
    echo "Set OVERLAY_DIR to the install_overlay directory."
    exit 1
fi

OV_MSKF_BIN="$OVERLAY_DIR/sensor_fusion_bringup/lib/sensor_fusion_bringup/ransac_pointcloud_fusion_node"
if [ ! -f "$OV_MSKF_BIN" ]; then
    echo "ERROR: ransac_pointcloud_fusion_node binary not found at $OV_MSKF_BIN"
    echo "Build it first: ./scripts/jetson_build_openvins.sh"
    exit 1
fi

DOCKER_IMAGE="${DOCKER_IMAGE:-prosthesis:latest}"

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  RANSAC Point Cloud Fusion"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  Head cloud:     $HEAD_TOPIC"
echo "  Arm cloud:      $ARM_TOPIC"
echo "  Output:         $OUTPUT_TOPIC"
echo "  Min correspondences: $MIN_CORRESPONDENCES"
echo "  Rate:           $MAX_RATE_HZ Hz"
echo "  Downsample:     $DOWNSAMPLE_LEAF_M m"
echo "  Fuse voxel:     $FUSE_VOXEL_LEAF_M m"
echo "  Normal radius:  $NORMAL_RADIUS_M m"
echo "  Feature radius: $FEATURE_RADIUS_M m"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

# Stop existing container if any
docker rm -f ransac_fusion 2>/dev/null || true

docker run -d --name ransac_fusion \
    --network host \
    --runtime nvidia \
    --restart unless-stopped \
    -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID" \
    -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    -v "$OVERLAY_DIR:/overlay:ro" \
    "$DOCKER_IMAGE" \
    bash -c "
        source /opt/ros/humble/setup.bash && \
        source /overlay/setup.bash 2>/dev/null || true && \
        export LD_LIBRARY_PATH=/overlay/ov_core/lib:/overlay/ov_init/lib:/overlay/ov_msckf/lib:\$LD_LIBRARY_PATH && \
        exec /overlay/sensor_fusion_bringup/lib/sensor_fusion_bringup/ransac_pointcloud_fusion_node --ros-args \
            -p head_cloud_topic:=$HEAD_TOPIC \
            -p arm_cloud_topic:=$ARM_TOPIC \
            -p output_topic:=$OUTPUT_TOPIC \
            -p ransac_min_correspondences:=$MIN_CORRESPONDENCES \
            -p max_rate_hz:=$MAX_RATE_HZ \
            -p downsample_leaf_m:=$DOWNSAMPLE_LEAF_M \
            -p fuse_voxel_leaf_m:=$FUSE_VOXEL_LEAF_M \
            -p normal_radius_m:=$NORMAL_RADIUS_M \
            -p feature_radius_m:=$FEATURE_RADIUS_M \
            -p max_cloud_age_s:=$MAX_CLOUD_AGE_S \
            -p ransac_max_iterations:=$RANSAC_MAX_ITERATIONS \
            -p ransac_max_correspondence_dist_m:=$RANSAC_MAX_CORRESPONDENCE_DIST_M
    "

sleep 2
if docker ps --filter name=ransac_fusion --format '{{.Status}}' | grep -q Up; then
    echo "RANSAC fusion container started (ransac_fusion)"
    echo ""
    echo "Status topic: $OUTPUT_TOPIC/status"
    echo "Logs:         docker logs -f ransac_fusion"
    echo "Stop:         docker rm -f ransac_fusion"
else
    echo "Container failed to start. Logs:"
    docker logs --tail 20 ransac_fusion
    docker rm -f ransac_fusion 2>/dev/null || true
    exit 1
fi
