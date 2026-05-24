# RANSAC point cloud fusion — standalone Docker image
#
# Builds the ransac_pointcloud_fusion_node binary and packages it
# inside a minimal ROS 2 Humble image.  PCL 1.12 is included in the
# base image (prosthesis:latest / isaac-ros-cuvslam:humble-jetson).
#
# Build:
#   docker build -f docker-deployment/ransac_fusion/Dockerfile -t ransac_fusion:latest .
#
# Run directly (host networking, picks up ROS2 topics):
#   docker run --rm --network host \
#     -e RANSAC_HEAD_TOPIC=/head/d435i_head/points_marker_map \
#     -e RANSAC_ARM_TOPIC=/arm/d435i_arm/points_marker_map \
#     ransac_fusion:latest
#
# Or use docker-compose:
#   docker compose -f docker-deployment/ransac_fusion/docker-compose.yml up -d

FROM prosthesis:latest

USER root

# Copy only what is needed for building sensor_fusion_bringup
COPY docker_ws/multi_cam_localization/sensor_fusion_bringup/ /ws/src/sensor_fusion_bringup/
COPY docker_ws/multi_cam_localization/sensor_fusion_msgs/ /ws/src/sensor_fusion_msgs/
COPY docker_ws/src/open_vins/ /ws/src/open_vins/

# Build the fusion node + dependencies
RUN . /opt/ros/humble/setup.sh && \
    colcon build \
      --base-paths /ws/src \
      --install-base /ws/install \
      --build-base /ws/build \
      --packages-select sensor_fusion_msgs ov_core ov_init sensor_fusion_bringup \
      --cmake-args -DCMAKE_BUILD_TYPE=Release \
      --executor sequential --parallel-workers 1 && \
    rm -rf /ws/build /ws/src

# Source environment on exec
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /ws/install/setup.bash" >> /root/.bashrc

ENV LD_LIBRARY_PATH=/opt/ros/humble/lib:/ws/install/ov_core/lib:/ws/install/ov_init/lib:${LD_LIBRARY_PATH}

# Entrypoint: runs the fusion node with configurable params via env vars
ENTRYPOINT ["bash", "-c", "source /opt/ros/humble/setup.bash && source /ws/install/setup.bash 2>/dev/null || true && exec ransac_pointcloud_fusion_node --ros-args \
  -p head_cloud_topic:=${RANSAC_HEAD_TOPIC:-/head/d435i_head/points_marker_map} \
  -p arm_cloud_topic:=${RANSAC_ARM_TOPIC:-/arm/d435i_arm/points_marker_map} \
  -p output_topic:=${RANSAC_OUTPUT_TOPIC:-/pointcloud_fused_ransac} \
  -p ransac_min_correspondences:=${RANSAC_MIN_CORRESPONDENCES:-8} \
  -p max_rate_hz:=${RANSAC_MAX_RATE_HZ:-10.0} \
  -p downsample_leaf_m:=${RANSAC_DOWNSAMPLE_LEAF_M:-0.02} \
  -p fuse_voxel_leaf_m:=${RANSAC_FUSE_VOXEL_LEAF_M:-0.01}"]
