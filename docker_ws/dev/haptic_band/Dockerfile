FROM ros:jazzy-ros-base

# Install BlueZ (userspace BT stack) and Python ROS 2 tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    bluez \
    python3-pip \
    ros-jazzy-std-msgs \
    ros-jazzy-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# Build the haptic_bridge ROS 2 package
WORKDIR /haptic_ws
COPY haptic_bridge/ src/haptic_bridge/
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release"

COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["ros2", "run", "haptic_bridge", "bridge_node"]
