# Minimal EMG armband image — ROS2 Jazzy
#
# Build context must be the mia_hand_ros2_pkgs/ directory so that both
# mia_hand_msgs/ and emg_armband/ are reachable.
# The docker-compose.yml next to this file sets that up automatically.
#
# What this image contains (and nothing more):
#   • ros:jazzy-ros-base          — core ROS2 runtime
#   • python3 + rclpy             — Python node support
#   • rmw_cyclonedds_cpp          — DDS implementation matching the rest of the stack
#   • colcon                      — build tool (needed at image-build time only)
#   • mia_hand_msgs               — EmgData message definition
#   • emg_armband                 — emg_node + emg_parser_node + launch + config
#
# To add your EMG hardware SDK (e.g. pyomyo, pyserial, …) uncomment/add a line
# in the "EMG hardware SDK" section below and rebuild.

FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=jazzy

ARG USER_UID=1000
ARG USER_GID=1000
ARG USERNAME=user

# Remove conflicting UID if present
RUN if id -u $USER_UID 2>/dev/null; then userdel "$(id -un $USER_UID)"; fi

# ── system dependencies ───────────────────────────────────────────────────────
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions \
        python3-pip \
        python3-yaml \
        sudo \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# ── user setup ────────────────────────────────────────────────────────────────
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /home/$USERNAME/.bashrc

USER $USERNAME

# ── ROS2 runtime extras ───────────────────────────────────────────────────────
RUN sudo apt-get update && \
    sudo apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    && sudo apt-get clean \
    && sudo rm -rf /var/lib/apt/lists/*

# ── EMG hardware SDK + ML dependencies ───────────────────────────────────────
# MindRove WiFi board SDK + the Python stack used by emg_classifier.
RUN sudo python3 -m pip install --no-cache-dir --break-system-packages \
        mindrove \
        numpy \
        scipy \
        scikit-learn \
        joblib

WORKDIR /emg_ws

# Copy only the two packages that this image needs
COPY mia_hand_msgs/ /emg_ws/src/mia_hand_msgs/
COPY emg_armband/   /emg_ws/src/emg_armband/

RUN sudo chown -R $USERNAME:$USERNAME /emg_ws \
    && sudo apt-get update \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y \
    && sudo apt-get clean \
    && sudo rm -rf /var/lib/apt/lists/* \
    && . "/opt/ros/${ROS_DISTRO}/setup.sh" \
    && colcon build \
    && echo "source /emg_ws/install/setup.bash" >> /home/$USERNAME/.bashrc

COPY emg_armband/docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN sudo chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/bin/bash"]
