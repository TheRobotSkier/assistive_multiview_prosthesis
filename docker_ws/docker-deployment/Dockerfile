FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=jazzy

ARG USER_UID=1000
ARG USER_GID=1000
ARG USERNAME=user

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
        libeigen3-dev \
        libboost-all-dev \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

RUN groupadd -f video \
    && groupadd -f plugdev \
    && groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && usermod -a -G dialout,video,plugdev,i2c $USERNAME \
    && echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && printf "%s\n" \
        "source /opt/ros/$ROS_DISTRO/setup.bash" \
        "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" \
        "export PS1=\\u@\\h:\\[\\e[32m\\]\\w\\[\\e[0m\\]\\\$ " \
        >> /home/$USERNAME/.bashrc

USER $USERNAME

RUN sudo apt-get update \
    && sudo apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-hardware-interface \
        ros-${ROS_DISTRO}-transmission-interface \
        ros-${ROS_DISTRO}-xacro \
        ros-${ROS_DISTRO}-joint-limits \
        ros-${ROS_DISTRO}-ros2-control \
        ros-${ROS_DISTRO}-ros2-controllers \
        ros-${ROS_DISTRO}-ros2-controllers-test-nodes \
        ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
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
    && MAKEFLAGS="-j1" CMAKE_BUILD_PARALLEL_LEVEL=1 colcon build --executor sequential --packages-select imu_driver sensor_fusion_bringup sensor_fusion_msgs \
    && echo "source install/setup.bash" >> /home/$USERNAME/.bashrc

COPY ./docker-deployment/ros_entrypoint.sh /ros_entrypoint.sh
RUN sudo chmod +x /ros_entrypoint.sh
