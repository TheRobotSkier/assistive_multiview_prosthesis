FROM ros:jazzy-ros-base

# ============================================================================
# LAYER 1: METADATA - All ENV & ARG declarations at top
# ============================================================================
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=jazzy

ENV RUSTUP_HOME=/usr/local/rustup \
    CARGO_HOME=/usr/local/cargo \
    PATH=/usr/local/cargo/bin:$PATH

ARG USER_UID=1000
ARG USER_GID=1000
ARG USERNAME=user

ARG MUJOCO_VERSION=3.3.4
ARG CPU_ARCH=x86_64

ENV MUJOCO_VERSION=${MUJOCO_VERSION}
ENV MUJOCO_DIR="/opt/mujoco/mujoco-${MUJOCO_VERSION}"
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${MUJOCO_DIR}/lib

# ============================================================================
# LAYER 2: BASE SYSTEM - User setup & sudoers (as root)
# ============================================================================
RUN if id -u $USER_UID 2>/dev/null ; then userdel `id -un $USER_UID` ; fi && \
    groupadd --gid $USER_GID $USERNAME && \
    useradd --uid $USER_UID --gid $USER_GID -m $USERNAME && \
    usermod -a -G dialout $USERNAME && \
    echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

# ============================================================================
# LAYER 3: STATIC DEPENDENCIES - Consolidated apt layer (as root)
# ============================================================================
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        bash-completion \
        git \
        nano \
        vim \
        python3-colcon-argcomplete \
        python3-colcon-common-extensions \
        python3-pip \
        wget \
        curl \
        sudo \
        libserial-dev \
        setserial \
        libglfw3-dev \
        xauth \
        xvfb \
        libegl1 \
        libgl1 \
        libgomp1 \
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
        ros-${ROS_DISTRO}-example-interfaces \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# ============================================================================
# LAYER 4: EXTERNAL TOOLCHAINS - MuJoCo (download, extract, cleanup in one RUN)
# ============================================================================
RUN mkdir -p ${MUJOCO_DIR} && \
    wget -q https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}/mujoco-${MUJOCO_VERSION}-linux-${CPU_ARCH}.tar.gz -O /tmp/mujoco.tar.gz && \
    tar -xzf /tmp/mujoco.tar.gz -C $(dirname "${MUJOCO_DIR}") && \
    rm -f /tmp/mujoco.tar.gz

# ============================================================================
# LAYER 5: EXTERNAL TOOLCHAINS - Rust (download, install, cleanup in one RUN)
# ============================================================================
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y --no-modify-path && \
    chmod -R a+w $RUSTUP_HOME $CARGO_HOME

# ============================================================================
# LAYER 6: PYTHON DEPENDENCIES - Consolidated pip layer
# ============================================================================
#RUN python3 -m pip install --no-cache-dir --default-timeout=1000 --break-system-packages \
#    open3d

RUN python3 -m pip install --no-cache-dir --default-timeout=1000 --break-system-packages \
    mujoco==${MUJOCO_VERSION}

# ============================================================================
# LAYER 7: BASHRC SETUP - User environment configuration
# ============================================================================
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$USERNAME/.bashrc && \
    echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/$USERNAME/.bashrc && \
    echo "export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:$MUJOCO_DIR/lib" >> /home/$USERNAME/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /home/$USERNAME/.bashrc && \
    echo "export PS1='\u@\h:\[\e[32m\]\w\[\e[0m\]\$ '" >> /home/$USERNAME/.bashrc && \
    chown -R $USERNAME:$USERNAME /home/$USERNAME

# ============================================================================
# LAYER 8: SWITCH TO USER - All subsequent operations run as non-root
# ============================================================================
USER $USERNAME

WORKDIR /miahand_ws

# ============================================================================
# LAYER 9: SOURCE CODE - COPY late to prevent cache invalidation on code changes
# ============================================================================
COPY --chown=$USERNAME:$USERNAME . /miahand_ws/src/

# ============================================================================
# LAYER 10: DEPENDENCY RESOLUTION & BUILD - ROS packages + colcon build (stuff was changed here)
# ============================================================================

RUN sudo chown -R $USERNAME:$USERNAME /miahand_ws

# ============================================================================
# ROSDEP INITIALIZATION (separate layer for clarity and debugging)
# ============================================================================
RUN rosdep update && \
    sudo rosdep install --from-paths /miahand_ws/src --ignore-src --rosdistro $ROS_DISTRO -y \
        --dependency-types buildtool \
        --dependency-types buildtool_export \
        --dependency-types build \
        --dependency-types exec \
        --skip-keys "test_interface_files" && \
    . "/opt/ros/${ROS_DISTRO}/setup.sh" && \
    cd /miahand_ws && colcon build --parallel-workers $(nproc) && \
    echo "source install/setup.bash" >> /home/$USERNAME/.bashrc

# ============================================================================
# LAYER 11: ENTRYPOINT SETUP
# ============================================================================
COPY --chown=$USERNAME:$USERNAME ./docker-deployment/ros_entrypoint.sh /ros_entrypoint.sh

RUN sudo chmod +x /ros_entrypoint.sh
