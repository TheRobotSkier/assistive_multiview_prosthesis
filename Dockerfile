ARG ROS_DISTRO=humble
FROM osrf/ros:${ROS_DISTRO}-desktop

ARG MUJOCO_VERSION=3.6.0
ARG MUJOCO_PLATFORM=linux-x86_64

ENV ROS_DISTRO=${ROS_DISTRO}
ENV MUJOCO_DIR=/opt/mujoco
ENV LD_LIBRARY_PATH=${MUJOCO_DIR}/lib:${LD_LIBRARY_PATH}
ENV PATH=${MUJOCO_DIR}/bin:${PATH}

# Install basic tools and ROS2 build tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    libglfw3-dev \
    neofetch \
    build-essential \
    curl \
    libclang-dev \
    python3-pip \
    python3-colcon-common-extensions \
    tar \
    && rm -rf /var/lib/apt/lists/*

# MuJoCo Installation
RUN set -eux; \
    archive="mujoco-${MUJOCO_VERSION}-${MUJOCO_PLATFORM}.tar.gz"; \
    base_url="https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}"; \
    curl -fL "${base_url}/${archive}" -o "/tmp/${archive}"; \
    curl -fL "${base_url}/${archive}.sha256" -o "/tmp/${archive}.sha256"; \
    cd /tmp; \
    echo "$(cat ${archive}.sha256 | cut -d' ' -f1) ${archive}" > ${archive}.sha256; \
    sha256sum -c "${archive}.sha256"; \
    tar -xf "${archive}" -C /opt; \
    ln -sfn "/opt/mujoco-${MUJOCO_VERSION}" "${MUJOCO_DIR}"; \
    rm -f "/tmp/${archive}" "/tmp/${archive}.sha256"

# Rust Installation
ENV RUSTUP_HOME=/usr/local/rustup \
    CARGO_HOME=/usr/local/cargo \
    PATH=/usr/local/cargo/bin:$PATH

RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y --no-modify-path \
    && chmod -R a+w $RUSTUP_HOME $CARGO_HOME

# Python packages
RUN if pip3 install --help | grep -q -- '--break-system-packages'; then \
      pip3 install --break-system-packages colcon-cargo colcon-ros-bundle; \
    else \
      pip3 install colcon-cargo colcon-ros-bundle; \
    fi

# System dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-hardware-interface \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-joint-limits \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-xacro \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /laptop_ws

# Shell setup
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc \
    && echo "if [ -f /laptop_ws/install/setup.bash ]; then source /laptop_ws/install/setup.bash; fi" >> /root/.bashrc

CMD ["/bin/bash"]