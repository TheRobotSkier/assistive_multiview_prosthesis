FROM ros:jazzy-ros-base

# # Set environment variables
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=jazzy 

ARG USER_UID=1000
ARG USER_GID=1000
ARG USERNAME=user

## MuJuCo version
ARG MUJOCO_VERSION=3.3.4
ARG CPU_ARCH=x86_64

# Delete user if it exists in container (skip if USER_UID is 0/root or unset)
RUN if [ -n "$USER_UID" ] && [ "$USER_UID" != "0" ] && id -u "$USER_UID" 2>/dev/null; then \
        userdel "$(id -un "$USER_UID")"; \
    fi

# Install essential packages and ROS development tools
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
        libglfw3-dev \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

### OPEN3D
# Install Open3D system dependencies and pip
RUN apt-get update && apt-get install --no-install-recommends -y \
    libegl1 \
    libgl1 \
    libgomp1 \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*
###

# Configure and install MuJoCo
ENV MUJOCO_VERSION=${MUJOCO_VERSION}
ENV MUJOCO_DIR="/opt/mujoco/mujoco-${MUJOCO_VERSION}"
## add also LD_LIBRARY_PATH as ENV for running the docker compose service for mujoco simulation
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${MUJOCO_DIR}/lib
RUN mkdir -p ${MUJOCO_DIR}

RUN wget https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}/mujoco-${MUJOCO_VERSION}-linux-${CPU_ARCH}.tar.gz
RUN mkdir -p /home/mujoco
RUN tar -xzf "mujoco-${MUJOCO_VERSION}-linux-${CPU_ARCH}.tar.gz" -C $(dirname "${MUJOCO_DIR}")


### ADDED: Rust Installation
ENV RUSTUP_HOME=/usr/local/rustup \
    CARGO_HOME=/usr/local/cargo \
    PATH=/usr/local/cargo/bin:$PATH

RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y --no-modify-path \
    && chmod -R a+w $RUSTUP_HOME $CARGO_HOME
###


# Setup user configuration
# usermod --append --groups dialout $USERNAME 
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && usermod -a -G dialout $USERNAME \ 
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/$USERNAME/.bashrc \
    && echo "export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:$MUJOCO_DIR/lib"  >> /home/$USERNAME/.bashrc \
    && echo "export PS1='\u@\h:\[\e[32m\]\w\[\e[0m\]\$ '" >> /home/$USERNAME/.bashrc 

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
        ros-${ROS_DISTRO}-tf2-geometry-msgs \
        ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
        ros-${ROS_DISTRO}-rqt \
        ros-${ROS_DISTRO}-rqt-graph \
    && sudo apt-get clean \
    && sudo rm -rf /var/lib/apt/lists/*

RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /home/$USERNAME/.bashrc

RUN sudo python3 -m pip install --no-cache-dir --break-system-packages mujoco==${MUJOCO_VERSION}



### OPEN3D + HTTP client for segmentation ROS2 node
# Install Open3D from the PyPI repositories
RUN sudo python3 -m pip install --no-cache-dir --break-system-packages --upgrade open3d requests
###

WORKDIR /miahand_ws

COPY . /miahand_ws/src/

RUN sudo chown -R $USERNAME:$USERNAME /miahand_ws \
    && sudo apt-get update \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y \
    && sudo apt-get clean \
    && sudo rm -rf /var/lib/apt/lists/* \
    && . "/opt/ros/${ROS_DISTRO}/setup.sh" \
    && colcon build \
    && echo "source install/setup.bash" >> /home/$USERNAME/.bashrc

COPY ./docker-deployment/ros_entrypoint.sh /ros_entrypoint.sh
RUN sudo chmod +x /ros_entrypoint.sh
