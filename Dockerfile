ARG ROS_DISTRO=kilted
FROM ros:${ROS_DISTRO}

# Install basic tools and ROS2 build tools
RUN apt-get update && apt-get install -y \
    neofetch \
    && rm -rf /var/lib/apt/lists/*

# Set working directory
WORKDIR /ros_container

# Source ROS setup
RUN echo "source /opt/ros/kilted/setup.bash" >> /root/.bashrc

# Default command
CMD ["/bin/bash"]
