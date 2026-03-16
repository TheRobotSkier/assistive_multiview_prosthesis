ARG ROS_DISTRO=kilted
FROM osrf/ros:${ROS_DISTRO}-desktop

ARG ROS_DISTRO
ARG MUJOCO_VERSION=3.6.0
ARG MUJOCO_PLATFORM=linux-x86_64

ENV ROS_DISTRO=${ROS_DISTRO}
ENV MUJOCO_DIR=/opt/mujoco
ENV LD_LIBRARY_PATH=${MUJOCO_DIR}/lib:${LD_LIBRARY_PATH}
ENV PATH=${MUJOCO_DIR}/bin:${PATH}

# Install basic tools and ROS2 build tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    curl \
    libglfw3-dev \
    neofetch \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-hardware-interface \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-xacro \
    tar \
    && rm -rf /var/lib/apt/lists/*

RUN set -eux; \
    archive="mujoco-${MUJOCO_VERSION}-${MUJOCO_PLATFORM}.tar.gz"; \
    base_url="https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}"; \
    curl -fL "${base_url}/${archive}" -o "/tmp/${archive}"; \
    curl -fL "${base_url}/${archive}.sha256" -o "/tmp/${archive}.sha256"; \
    cd /tmp; \
    sha256sum -c "${archive}.sha256"; \
    tar -xf "${archive}" -C /opt; \
    ln -sfn "/opt/mujoco-${MUJOCO_VERSION}" "${MUJOCO_DIR}"; \
    rm -f "/tmp/${archive}" "/tmp/${archive}.sha256"

# Set working directory
WORKDIR /ros_container

# Source ROS setup
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc \
    && echo "if [ -f /ros_container/install/setup.bash ]; then source /ros_container/install/setup.bash; fi" >> /root/.bashrc \
    && echo "export MUJOCO_DIR=${MUJOCO_DIR}" >> /root/.bashrc

# Default command
CMD ["/bin/bash"]
