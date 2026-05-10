FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=jazzy

ARG USER_UID=1000
ARG USER_GID=1000
ARG USERNAME=user

ARG MUJOCO_VERSION=3.3.4
ARG CPU_ARCH=aarch64

RUN if id -u $USER_UID >/dev/null 2>&1; then userdel "$(id -un $USER_UID)"; fi

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        bash-completion \
        git \
        nano \
        python3-colcon-argcomplete \
        python3-colcon-common-extensions \
        sudo \
        vim \
        libserial-dev \
        setserial \
        python3-pip \
        wget \
        curl \
        libglfw3-dev \
        libegl1 \
        libgl1 \
        libgomp1 \
        libeigen3-dev \
        libboost-all-dev \
        libceres-dev \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

ENV MUJOCO_VERSION=${MUJOCO_VERSION}
ENV MUJOCO_DIR="/opt/mujoco/mujoco-${MUJOCO_VERSION}"
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${MUJOCO_DIR}/lib

RUN mkdir -p ${MUJOCO_DIR}

RUN wget https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}/mujoco-${MUJOCO_VERSION}-linux-${CPU_ARCH}.tar.gz
RUN mkdir -p /home/mujoco
RUN tar -xzf "mujoco-${MUJOCO_VERSION}-linux-${CPU_ARCH}.tar.gz" -C $(dirname "${MUJOCO_DIR}")

ENV RUSTUP_HOME=/usr/local/rustup \
    CARGO_HOME=/usr/local/cargo \
    PATH=/usr/local/cargo/bin:$PATH

RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y --no-modify-path \
    && chmod -R a+w $RUSTUP_HOME $CARGO_HOME

RUN groupadd -f video \
    && groupadd -f plugdev \
    && groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && usermod -a -G dialout,video,plugdev $USERNAME \
    && echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && printf '%s\n' \
        "source /opt/ros/$ROS_DISTRO/setup.bash" \
        "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" \
        "export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:$MUJOCO_DIR/lib" \
        "export PS1='\\u@\\h:\\[\\e[32m\\]\\w\\[\\e[0m\\]\\$ '" \
        >> /home/$USERNAME/.bashrc

USER $USERNAME

RUN sudo apt-get update \
    && sudo apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-joint-state-publisher-gui \
        ros-${ROS_DISTRO}-hardware-interface \
        ros-${ROS_DISTRO}-transmission-interface \
        ros-${ROS_DISTRO}-xacro \
        ros-${ROS_DISTRO}-joint-limits \
        ros-${ROS_DISTRO}-rviz2 \
        ros-${ROS_DISTRO}-rqt-controller-manager \
        ros-${ROS_DISTRO}-rqt-joint-trajectory-controller \
        ros-${ROS_DISTRO}-ros2-control \
        ros-${ROS_DISTRO}-ros2-controllers \
        ros-${ROS_DISTRO}-ros2-controllers-test-nodes \
        ros-${ROS_DISTRO}-moveit \
        ros-${ROS_DISTRO}-moveit-setup-assistant \
        ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
        ros-${ROS_DISTRO}-rqt \
        ros-${ROS_DISTRO}-rqt-graph \
        ros-${ROS_DISTRO}-librealsense2 \
        ros-${ROS_DISTRO}-realsense2-camera \
        ros-${ROS_DISTRO}-realsense2-description \
        libopencv-dev \
        ros-${ROS_DISTRO}-cv-bridge \
        ros-${ROS_DISTRO}-image-transport \
    && sudo apt-get clean \
    && sudo rm -rf /var/lib/apt/lists/*

RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /home/$USERNAME/.bashrc

RUN sudo python3 -m pip install --no-cache-dir --break-system-packages \
    mujoco==${MUJOCO_VERSION} \
    smbus2

WORKDIR /miahand_ws

COPY . /miahand_ws/src/

RUN sudo chown -R $USERNAME:$USERNAME /miahand_ws \
    && sudo apt-get update \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y \
    && sudo apt-get clean \
    && sudo rm -rf /var/lib/apt/lists/* \
    && . "/opt/ros/${ROS_DISTRO}/setup.sh" \
    && MAKEFLAGS="-j1" CMAKE_BUILD_PARALLEL_LEVEL=1 colcon build --executor sequential \
    && echo "source install/setup.bash" >> /home/$USERNAME/.bashrc

COPY ./docker-deployment/ros_entrypoint.sh /ros_entrypoint.sh
RUN sudo chmod +x /ros_entrypoint.sh